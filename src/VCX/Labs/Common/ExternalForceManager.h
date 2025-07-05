#pragma once

#include "Engine/Camera.hpp"
#include "Labs/1-RigidBody/Box.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Common {
    class ExternalForceManager {
    public:
        ExternalForceManager() {}

        void ProcessInput(Engine::Camera const & camera, ImVec2 const & mousePos);

        std::pair<glm::vec3, glm::vec3>       getForce(RigidBody::Box const & box);
        std::tuple<glm::vec3, glm::vec3, int> getForce(std::vector<RigidBody::Box> const & box);
        std::pair<Eigen::Vector3f, int>       getForce(std::vector<Eigen::Vector3f> points);

    private:
        glm::vec3 _forceDelta   = glm::vec3(0.f);
        glm::vec3 _rayDirection = glm::vec3(0.f);
        glm::vec3 _cameraEye    = glm::vec3(0.f);
    };
} // namespace VCX::Labs::Common
