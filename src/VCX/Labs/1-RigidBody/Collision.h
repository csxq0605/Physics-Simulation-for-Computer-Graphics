#pragma once

#include <fcl/narrowphase/collision.h>

#include "Labs/1-RigidBody/Box.h"

namespace VCX::Labs::RigidBody {
    void DetectCollision(Box &box0, Box &box1, fcl::CollisionResult<float> &collisionResult);

    void SolveCollision(Box &box0, Box &box1, fcl::CollisionResult<float> &collisionResult, float restitution);
} // namespace VCX::Labs::RigidBody
