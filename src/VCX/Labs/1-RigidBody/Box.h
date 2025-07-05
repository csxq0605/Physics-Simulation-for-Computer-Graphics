#pragma once

#include <Eigen/Dense>

namespace VCX::Labs::RigidBody {
    struct Box {
        Box();
        Box(bool fixed);

        Eigen::Matrix3f GetInertiaMatrix();

        Eigen::Vector3f                     Center { 0.f, 0.f, 0.f };
        Eigen::Vector3f                     Dim { 1.f, 2.f, 3.f };
        Eigen::Vector3f                     Velocity { 0.f, 0.f, 0.f };
        Eigen::Vector3f                     AngularVelocity { 0.f, 0.f, 0.f };
        Eigen::Quaternionf                  Orientation { 1.f, 0.f, 0.f, 0.f };
        float                               Mass { 1.f };
        bool                                Fixed { false };
    };
} // namespace VCX::Labs::RigidBody
