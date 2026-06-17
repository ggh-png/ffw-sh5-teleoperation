#pragma once
#include "ForwardKinematics.hpp"
#include "math/Math.hpp"
#include <vector>

// Jacobian Transpose IK solver.
// Iterative, works well for teleoperation (smooth, doesn't need matrix inversion).
class InverseKinematics {
public:
    struct Config {
        int   maxIter      = 20;
        float tolerance    = 1e-3f;   // stop when |error| < tolerance (meters)
        float alpha        = 0.5f;    // step size (tune per robot)
        float dampingLambda = 0.01f;  // for damped least squares fallback
    };

    // Solve IK for a single chain end-effector → target position.
    // Nodes in `chain` are from end-effector toward base (ForwardKinematics::chain).
    // Only revolute/prismatic joints are moved.
    static bool solve(SceneNode*              root,
                      SceneNode*              endEffector,
                      const Vec3&             target,
                      const Config&           cfg = Config{})
    {
        for(int iter = 0; iter < cfg.maxIter; ++iter) {
            // Recompute FK with current joint values
            ForwardKinematics::compute(*root);

            Vec3 eePos = ForwardKinematics::worldPosition(*endEffector);
            Vec3 error = target - eePos;

            if(error.length() < cfg.tolerance) return true;

            // Walk up the chain and apply Jacobian-Transpose step
            SceneNode* node = endEffector;
            while(node && node != root->parent) {
                if(node->joint.type == JointType::Revolute) {
                    // World-space joint axis
                    Vec3 jointPos  = ForwardKinematics::worldPosition(*node);
                    Vec3 jointAxis = node->worldTransform.transformDir(
                                        node->joint.axis).normalized();

                    // J column = axis × (ee - jointPos)
                    Vec3 r      = eePos - jointPos;
                    Vec3 jCol   = jointAxis.cross(r);

                    // Jacobian-Transpose: Δθ = α * (Jᵀ e)
                    float delta = cfg.alpha * jCol.dot(error);
                    node->joint.value += delta;
                    node->joint.clamp();

                } else if(node->joint.type == JointType::Prismatic) {
                    Vec3 axis  = node->worldTransform.transformDir(
                                     node->joint.axis).normalized();
                    float delta = cfg.alpha * axis.dot(error);
                    node->joint.value += delta;
                    node->joint.clamp();
                }
                node = node->parent;
            }
        }

        ForwardKinematics::compute(*root);
        Vec3 finalErr = target - ForwardKinematics::worldPosition(*endEffector);
        return finalErr.length() < cfg.tolerance;
    }
};
