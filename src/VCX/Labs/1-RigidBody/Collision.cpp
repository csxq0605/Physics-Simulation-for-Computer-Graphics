#include "Labs/1-RigidBody/Collision.h"

namespace VCX::Labs::RigidBody {
    void DetectCollision(Box &box0, Box &box1, fcl::CollisionResult<float> &collisionResult) {
        // Eigen::Vector3f Box::Dim - size of a box
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        CollisionGeometryPtr_t box_geometry_A(new fcl::Box<float>(box0.Dim[0], box0.Dim[1], box0.Dim[2]));
        CollisionGeometryPtr_t box_geometry_B(new fcl::Box<float>(box1.Dim[0], box1.Dim[1], box1.Dim[2]));
        // Eigen::Vector3f Box::Center - position of a box, Eigen::Quaternionf Box::q - rotation of a box
        fcl::CollisionObject<float> box_A(box_geometry_A, fcl::Transform3f(Eigen::Translation3f(box0.Center)*box0.Orientation));
        fcl::CollisionObject<float> box_B(box_geometry_B, fcl::Transform3f(Eigen::Translation3f(box1.Center)*box1.Orientation));
        // Compute collision - at most 1 contacts and return contact information.
        fcl::CollisionRequest<float> collisionRequest(1, true);
        fcl::collide(&box_A, &box_B, collisionRequest, collisionResult);
    }

    void SolveCollision(Box &box0, Box &box1, fcl::CollisionResult<float> &collisionResult, float restitution) {
        if(!collisionResult.isCollision()) {
            collisionResult.clear();
            return;
        }

        std::vector<fcl::Contact<float>> contacts;
        collisionResult.getContacts(contacts);
        Eigen::Vector3f contact_pos = contacts[0].pos;
        Eigen::Vector3f contact_normal = -contacts[0].normal;

        Eigen::Vector3f v0 = box0.Velocity + box0.AngularVelocity.cross(contact_pos - box0.Center);
        Eigen::Vector3f v1 = box1.Velocity + box1.AngularVelocity.cross(contact_pos - box1.Center);
        float relativeVelocity = contact_normal.dot(v0 - v1);

        if (relativeVelocity > 0) {
            collisionResult.clear();
            return;
        }

        float impulse = - (1 + restitution) * relativeVelocity / (1 / box0.Mass + 1 / box1.Mass + contact_normal.dot(
            box0.GetInertiaMatrix().inverse() * (contact_pos - box0.Center).cross(contact_normal).cross(contact_pos - box0.Center) +
            box1.GetInertiaMatrix().inverse() * (contact_pos - box1.Center).cross(contact_normal).cross(contact_pos - box1.Center)
        ));

        if (!box0.Fixed) {
            box0.Velocity += impulse / box0.Mass * contact_normal;
            box0.AngularVelocity += impulse * box0.GetInertiaMatrix().inverse() * (contact_pos - box0.Center).cross(contact_normal);
        }

        if (!box1.Fixed) {
            box1.Velocity -= impulse / box1.Mass * contact_normal;
            box1.AngularVelocity -= impulse * box1.GetInertiaMatrix().inverse() * (contact_pos - box1.Center).cross(contact_normal);
        }

        collisionResult.clear();
    }
} // namespace VCX::Labs::RigidBody
