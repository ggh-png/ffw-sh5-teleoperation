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
};
