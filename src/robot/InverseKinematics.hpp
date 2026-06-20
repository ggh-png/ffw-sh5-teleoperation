#pragma once
#include "ForwardKinematics.hpp"
#include "math/Math.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <utility>

struct IKConfig {
    int   maxIter        = 100;    // iterations per frame
    float tolerance      = 1e-3f;  // position convergence threshold (m)
    float dampingLambda  = 0.01f;  // DLS damping λ — low = fast convergence away from singularity
    bool  useOrientation = false;
    float orientWeight   = 1.0f;
    float maxJointDelta  = 0.20f;  // max joint change per iteration (rad)
};

// ── Damped Least Squares (DLS / Levenberg-Marquardt) IK ──────────────────────
//
//   Δθ = J^T (J J^T + λ²I)^{-1} e
//
// J is the geometric Jacobian (eDim × N).
// Position Jacobian column i  = axis_i × (eePos − jointPos_i)
// Orientation Jacobian col i  = axis_i   (scaled by orientWeight)
//
// Workspace helpers:
//   computeMaxReach  — sets chain joints to 0, runs FK, measures shoulder→EE dist
//   clampToWorkspace — clamps world-space target inside the reachable sphere
class InverseKinematics {
public:
    using Config = IKConfig;

    // ── Workspace helpers ─────────────────────────────────────────────────────

    // Walk from eeNode toward the tree root, stopping at chainStop.
    // Returns the joint node farthest from eeNode (the "shoulder").
    static SceneNode* findShoulder(SceneNode* eeNode, SceneNode* chainStop) {
        SceneNode* shoulder = nullptr;
        for(SceneNode* n = eeNode; n && n != chainStop; n = n->parent)
            if(n->joint.type == JointType::Revolute ||
               n->joint.type == JointType::Prismatic)
                shoulder = n;
        return shoulder;
    }

    // Sum the MJCF body-frame offsets (|localPos|) from eeNode up to chainStop.
    // This is the THEORETICAL maximum reach regardless of joint angles — the
    // upper bound of the reachable workspace sphere centred at the shoulder.
    // Does NOT modify joint values; O(N) in chain length.
    static float computeChainLength(SceneNode* eeNode, SceneNode* chainStop) {
        float total = 0.f;
        for(SceneNode* n = eeNode; n && n != chainStop; n = n->parent)
            total += n->localPos.length();
        return total;
    }

    // Compute the reach at a specific reference pose (joints = 0) to verify
    // that computeChainLength is a valid upper bound.  Useful for debugging.
    static float computeMaxReach(SceneNode* root,
                                  SceneNode* eeNode,
                                  SceneNode* chainStop)
    {
        std::vector<std::pair<SceneNode*, float>> saved;
        for(SceneNode* n = eeNode; n && n != chainStop; n = n->parent) {
            if(n->joint.type == JointType::Revolute ||
               n->joint.type == JointType::Prismatic) {
                saved.push_back({n, n->joint.value});
                n->joint.value = 0.f;
            }
        }
        ForwardKinematics::compute(*root);

        SceneNode* shoulder = findShoulder(eeNode, chainStop);
        float reach = 0.f;
        if(shoulder) {
            Vec3 s = shoulder->worldTransform.transformPoint({0,0,0});
            Vec3 e = eeNode->worldTransform.transformPoint({0,0,0});
            reach  = (e - s).length();
        }

        for(auto& [node, val] : saved) node->joint.value = val;
        ForwardKinematics::compute(*root);
        return reach;
    }

    // Clamp a world-space target to within the reachable sphere.
    // margin ∈ (0,1]: keep target slightly inside to avoid full-extension singularity.
    static Vec3 clampToWorkspace(const Vec3& target,
                                  const Vec3& shoulderPos,
                                  float maxReach,
                                  float margin = 0.95f)
    {
        Vec3  d    = target - shoulderPos;
        float dist = d.length();
        float cap  = maxReach * margin;
        if(dist > cap && dist > 1e-5f)
            return shoulderPos + d * (cap / dist);
        return target;
    }

    // ── Solver ────────────────────────────────────────────────────────────────

    // Position-only wrapper (backwards compat).
    static bool solve(SceneNode*   root,
                      SceneNode*   endEffector,
                      const Vec3&  target,
                      IKConfig     cfg       = {},
                      SceneNode*   chainStop = nullptr)
    {
        return solve6(root, endEffector, target,
                      Quaternion::identity(), false, cfg, chainStop);
    }

    // Full DLS IK (position + optional orientation).
    //
    // chainStop: exclusive upper bound of the kinematic chain.
    //   The loop traverses from endEffector toward root and STOPS at chainStop,
    //   so only joints between endEffector and chainStop (exclusive) are moved.
    //   Pass nullptr to let the solver traverse all the way to root.
    static bool solve6(SceneNode*        root,
                       SceneNode*        endEffector,
                       const Vec3&       targetPos,
                       const Quaternion& targetRot,
                       bool              useOrientation,
                       IKConfig          cfg       = {},
                       SceneNode*        chainStop = nullptr)
    {
        const int eDim = useOrientation ? 6 : 3;

        static constexpr int kMaxJoints = 16;

        for(int iter = 0; iter < cfg.maxIter; ++iter) {
            ForwardKinematics::compute(*root);

            Vec3 eePos = ForwardKinematics::worldPosition(*endEffector);

            // ── Error vector e (eDim × 1) ─────────────────────────────────
            Vec3 ePos = targetPos - eePos;

            float eRot3[3] = {0.f, 0.f, 0.f};
            if(useOrientation) {
                Quaternion eeQ  = endEffector->worldTransform.extractRotation();
                Quaternion qErr = (targetRot * eeQ.conjugate()).normalized();
                if(qErr.w < 0.f) {
                    qErr.w=-qErr.w; qErr.x=-qErr.x; qErr.y=-qErr.y; qErr.z=-qErr.z;
                }
                // Scaled axis-angle error, weighted
                eRot3[0] = 2.f * qErr.x * cfg.orientWeight;
                eRot3[1] = 2.f * qErr.y * cfg.orientWeight;
                eRot3[2] = 2.f * qErr.z * cfg.orientWeight;
            }

            float posErr = ePos.length();
            float rotErr = useOrientation
                ? std::sqrt(eRot3[0]*eRot3[0] + eRot3[1]*eRot3[1] + eRot3[2]*eRot3[2])
                  / cfg.orientWeight
                : 0.f;

            if(posErr < cfg.tolerance &&
               (!useOrientation || rotErr < cfg.tolerance * 2.f))
                return true;

            float e[6] = { ePos.x, ePos.y, ePos.z,
                           eRot3[0], eRot3[1], eRot3[2] };

            // ── Build Jacobian J (eDim × N), column-major: J[col*eDim+row] ──
            float J[kMaxJoints * 6] = {};
            std::array<SceneNode*, kMaxJoints> joints{};
            int N = 0;

            SceneNode* node = endEffector;
            while(node && node != chainStop && N < kMaxJoints) {
                if(node->joint.type == JointType::Revolute) {
                    Vec3 jPos = node->worldTransform.transformPoint({0,0,0});
                    Vec3 axis = node->worldTransform
                                    .transformDir(node->joint.axis).normalized();
                    Vec3 r    = eePos - jPos;
                    Vec3 Jp   = axis.cross(r);

                    J[N*eDim+0] = Jp.x;
                    J[N*eDim+1] = Jp.y;
                    J[N*eDim+2] = Jp.z;
                    if(useOrientation) {
                        J[N*eDim+3] = axis.x * cfg.orientWeight;
                        J[N*eDim+4] = axis.y * cfg.orientWeight;
                        J[N*eDim+5] = axis.z * cfg.orientWeight;
                    }
                    joints[N++] = node;

                } else if(node->joint.type == JointType::Prismatic) {
                    Vec3 axis = node->worldTransform
                                    .transformDir(node->joint.axis).normalized();
                    J[N*eDim+0] = axis.x;
                    J[N*eDim+1] = axis.y;
                    J[N*eDim+2] = axis.z;
                    joints[N++] = node;
                }
                node = node->parent;
            }
            if(N == 0) break;

            // ── Build A = J J^T + λ²I  (eDim × eDim) ────────────────────
            float A[6*6]    = {};
            float lambda2   = cfg.dampingLambda * cfg.dampingLambda;
            for(int r = 0; r < eDim; ++r) {
                for(int c = 0; c < eDim; ++c) {
                    float v = (r == c) ? lambda2 : 0.f;
                    for(int k = 0; k < N; ++k)
                        v += J[k*eDim+r] * J[k*eDim+c];
                    A[r*eDim+c] = v;
                }
            }

            // ── Solve A x = e via Gauss-Jordan with partial pivoting ──────
            // augmented matrix [A | e], size eDim × (eDim+1)
            float aug[6*7] = {};
            const int W = eDim + 1;
            for(int r = 0; r < eDim; ++r) {
                for(int c = 0; c < eDim; ++c) aug[r*W+c] = A[r*eDim+c];
                aug[r*W+eDim] = e[r];
            }

            for(int col = 0; col < eDim; ++col) {
                // Partial pivot
                int piv = col;
                for(int r = col+1; r < eDim; ++r)
                    if(std::fabs(aug[r*W+col]) > std::fabs(aug[piv*W+col]))
                        piv = r;
                if(piv != col)
                    for(int c = 0; c <= eDim; ++c)
                        std::swap(aug[col*W+c], aug[piv*W+c]);

                float d = aug[col*W+col];
                if(std::fabs(d) < 1e-9f) continue;
                for(int c = col; c <= eDim; ++c) aug[col*W+c] /= d;

                for(int r = 0; r < eDim; ++r) {
                    if(r == col) continue;
                    float f = aug[r*W+col];
                    for(int c = col; c <= eDim; ++c) aug[r*W+c] -= f * aug[col*W+c];
                }
            }
            // Solution x is in aug[r*W+eDim]

            // ── Δθ = J^T x, clamped ───────────────────────────────────────
            for(int k = 0; k < N; ++k) {
                float dq = 0.f;
                for(int r = 0; r < eDim; ++r)
                    dq += J[k*eDim+r] * aug[r*W+eDim];

                // Per-iteration delta cap prevents large jumps and oscillation
                float cap = cfg.maxJointDelta;
                dq = dq < -cap ? -cap : (dq > cap ? cap : dq);

                joints[k]->joint.value += dq;
                joints[k]->joint.clamp();
            }
        }

        ForwardKinematics::compute(*root);
        return (ForwardKinematics::worldPosition(*endEffector) - targetPos).length()
               < cfg.tolerance;
    }

    // ── Hierarchical IK: position first, orientation in null space ────────────
    //
    // Per iteration:
    //   1. Position DLS:   Δθ_pos = Jp^T (Jp Jp^T + λ²I)^{-1} e_pos
    //   2. Orientation in null space of position:
    //      g_ori  = Jo^T * e_ori
    //      Δθ_ori = g_ori − Jp^T (Jp Jp^T + λ²I)^{-1} Jp g_ori
    //   3. Δθ = Δθ_pos + Δθ_ori   (clamped)
    //
    // This guarantees (to first order) that orientation adjustments don't
    // disturb position.  Used when the user drags an RPY slider while the
    // arm is already holding a position target.
    static bool solveHierarchical(SceneNode*        root,
                                  SceneNode*        endEffector,
                                  const Vec3&       targetPos,
                                  const Quaternion& targetRot,
                                  IKConfig          cfg       = {},
                                  SceneNode*        chainStop = nullptr)
    {
        static constexpr int kMaxJ = 16;

        // Helper: 3×3 Gauss-Jordan, solves Ax=b, writes solution into b column
        auto solve3 = [](float A[9], float b[3]) {
            float aug[3*4];
            for(int r = 0; r < 3; ++r) {
                for(int c = 0; c < 3; ++c) aug[r*4+c] = A[r*3+c];
                aug[r*4+3] = b[r];
            }
            for(int col = 0; col < 3; ++col) {
                int piv = col;
                for(int r = col+1; r < 3; ++r)
                    if(std::fabs(aug[r*4+col]) > std::fabs(aug[piv*4+col])) piv = r;
                if(piv != col)
                    for(int c = 0; c <= 3; ++c) std::swap(aug[col*4+c], aug[piv*4+c]);
                float d = aug[col*4+col];
                if(std::fabs(d) < 1e-9f) continue;
                for(int c = col; c <= 3; ++c) aug[col*4+c] /= d;
                for(int r = 0; r < 3; ++r) {
                    if(r == col) continue;
                    float f = aug[r*4+col];
                    for(int c = col; c <= 3; ++c) aug[r*4+c] -= f * aug[col*4+c];
                }
            }
            b[0] = aug[0*4+3]; b[1] = aug[1*4+3]; b[2] = aug[2*4+3];
        };

        for(int iter = 0; iter < cfg.maxIter; ++iter) {
            ForwardKinematics::compute(*root);

            Vec3       eePos = ForwardKinematics::worldPosition(*endEffector);
            Quaternion eeQ   = endEffector->worldTransform.extractRotation();

            // Error vectors
            Vec3 ePos = targetPos - eePos;

            Quaternion qErr = (targetRot * eeQ.conjugate()).normalized();
            if(qErr.w < 0.f) {
                qErr.w=-qErr.w; qErr.x=-qErr.x; qErr.y=-qErr.y; qErr.z=-qErr.z;
            }
            float eOri[3] = {
                2.f * qErr.x * cfg.orientWeight,
                2.f * qErr.y * cfg.orientWeight,
                2.f * qErr.z * cfg.orientWeight
            };

            float posErr = ePos.length();
            float rotErr = std::sqrt(eOri[0]*eOri[0]+eOri[1]*eOri[1]+eOri[2]*eOri[2])
                           / std::max(cfg.orientWeight, 1e-6f);
            if(posErr < cfg.tolerance && rotErr < cfg.tolerance * 2.f) return true;

            // Build Jp (position, 3×N) and Jo (orientation, 3×N), col-major
            float Jp[kMaxJ*3] = {};
            float Jo[kMaxJ*3] = {};
            std::array<SceneNode*, kMaxJ> joints{};
            int N = 0;

            for(SceneNode* nd = endEffector;
                nd && nd != chainStop && N < kMaxJ;
                nd = nd->parent)
            {
                if(nd->joint.type != JointType::Revolute &&
                   nd->joint.type != JointType::Prismatic) continue;
                Vec3 axis = nd->worldTransform
                                .transformDir(nd->joint.axis).normalized();
                if(nd->joint.type == JointType::Revolute) {
                    Vec3 jPos = nd->worldTransform.transformPoint({0,0,0});
                    Vec3 r    = eePos - jPos;
                    Vec3 Jp_  = axis.cross(r);
                    Jp[N*3+0] = Jp_.x; Jp[N*3+1] = Jp_.y; Jp[N*3+2] = Jp_.z;
                } else {
                    Jp[N*3+0] = axis.x; Jp[N*3+1] = axis.y; Jp[N*3+2] = axis.z;
                }
                Jo[N*3+0] = axis.x; Jo[N*3+1] = axis.y; Jo[N*3+2] = axis.z;
                joints[N++] = nd;
            }
            if(N == 0) break;

            // A = Jp Jp^T + λ²I  (3×3)
            float lam2 = cfg.dampingLambda * cfg.dampingLambda;
            float A[9] = {};
            for(int r = 0; r < 3; ++r)
                for(int c = 0; c < 3; ++c) {
                    float v = (r==c) ? lam2 : 0.f;
                    for(int k = 0; k < N; ++k) v += Jp[k*3+r] * Jp[k*3+c];
                    A[r*3+c] = v;
                }

            // Step 1 — position DLS:  x_pos = A^{-1} e_pos
            float xp[3] = { ePos.x, ePos.y, ePos.z };
            { float Atmp[9]; for(int i=0;i<9;++i) Atmp[i]=A[i]; solve3(Atmp, xp); }

            // Δθ_pos = Jp^T x_pos
            float dq_pos[kMaxJ] = {};
            for(int k = 0; k < N; ++k)
                for(int r = 0; r < 3; ++r)
                    dq_pos[k] += Jp[k*3+r] * xp[r];

            // Step 2 — orientation in null space
            // g_ori = Jo^T e_ori
            float g_ori[kMaxJ] = {};
            for(int k = 0; k < N; ++k)
                for(int r = 0; r < 3; ++r)
                    g_ori[k] += Jo[k*3+r] * eOri[r];

            // v = Jp * g_ori  (3-vector)
            float v3[3] = {};
            for(int r = 0; r < 3; ++r)
                for(int k = 0; k < N; ++k)
                    v3[r] += Jp[k*3+r] * g_ori[k];

            // y = A^{-1} v
            { float Atmp[9]; for(int i=0;i<9;++i) Atmp[i]=A[i]; solve3(Atmp, v3); }

            // proj = Jp^T y
            float proj[kMaxJ] = {};
            for(int k = 0; k < N; ++k)
                for(int r = 0; r < 3; ++r)
                    proj[k] += Jp[k*3+r] * v3[r];

            // Δθ = Δθ_pos + (g_ori − proj),  clamped
            float cap = cfg.maxJointDelta;
            for(int k = 0; k < N; ++k) {
                float dq = dq_pos[k] + (g_ori[k] - proj[k]);
                dq = dq < -cap ? -cap : (dq > cap ? cap : dq);
                joints[k]->joint.value += dq;
                joints[k]->joint.clamp();
            }
        }

        ForwardKinematics::compute(*root);
        return true;
    }
};
