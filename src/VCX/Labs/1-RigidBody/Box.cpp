#include <Labs/1-RigidBody/Box.h>

namespace VCX::Labs::RigidBody {
    Box::Box() {}
    Box::Box(bool fixed): Fixed(fixed) {}

    Eigen::Matrix3f Box::GetInertiaMatrix() {
        float w = Dim[0], h = Dim[1], d=Dim[2];
        Eigen::Matrix3f Iref;
        Iref << 1.f / 12.f * Mass * (h * h + d * d), 0, 0,
                0, 1.f / 12.f * Mass * (d * d + w * w), 0,
                0, 0, 1.f / 12.f * Mass * (w * w + h * h);
        Eigen::Matrix3f R = Orientation.matrix();
        return R * Iref * R.transpose();
    }
} // namespace VCX::Labs::RigidBody
