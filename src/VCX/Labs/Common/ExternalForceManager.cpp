#pragma once

#include <imgui_internal.h>
#include <iostream>

#include "Engine/app.h"
#include "Labs/Common/ExternalForceManager.h"

static glm::vec3 eigen2glm(Eigen::Vector3f const & eigen_v) {
    return glm::vec3(eigen_v[0], eigen_v[1], eigen_v[2]);
}

static Eigen::Vector3f glm2eigen(glm::vec3 const & glm_v) {
    return Eigen::Vector3f(glm_v[0], glm_v[1], glm_v[2]);
}

glm::vec3 GetRayDirection(glm::vec2 const & screenPoint, VCX::Engine::Camera const & camera, ImVec2 const & windowSize) {
    // convert screen point to NDC
    float ndcX = (2.0f * screenPoint.x) / windowSize.x - 1.0f;
    float ndcY = 1.0f - (2.0f * screenPoint.y) / windowSize.y;
    glm::vec4 clipSpacePos(ndcX, ndcY, -1.0f, 1.0f);

    // transform to world space
    glm::mat4 invProjMatrix = glm::inverse(camera.GetProjectionMatrix(windowSize.x / windowSize.y));
    glm::mat4 invViewMatrix = glm::inverse(camera.GetViewMatrix());
    glm::vec4 viewSpacePos = invProjMatrix * clipSpacePos;
    viewSpacePos.z = -1.0f; // Set to -1 for a forward-facing ray
    viewSpacePos.w = 0.0f; // Set to 0 for a direction vector
    glm::vec4 worldSpacePos = invViewMatrix * viewSpacePos;

    // normalize the direction
    glm::vec3 rayDirection = glm::normalize(glm::vec3(worldSpacePos));
    return rayDirection;
}

namespace VCX::Labs::Common {
    void ExternalForceManager::ProcessInput(Engine::Camera const & camera, ImVec2 const & mousePos) {
        auto            window  = ImGui::GetCurrentWindow();
        ImGuiIO const & io      = ImGui::GetIO();
        bool            anyHeld = false;
        bool            hover   = false;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hover, &anyHeld);
        bool         leftHeld  = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Left);
        ImVec2 const delta     = io.MouseDelta;
        bool         moving    = (delta.x != 0.f || delta.y != 0.f) && hover;
        bool         altKey    = io.KeyAlt;

        float heightNorm     = 1.f / window->Rect().GetHeight();

        bool applyingForceByMouse = moving && altKey && leftHeld;

        if (applyingForceByMouse) {
            // perspective
            glm::vec3 direction      = camera.Target - camera.Eye;
            float     targetDistance = glm::length(direction);
            glm::quat q              = glm::quatLookAt(direction / targetDistance, camera.Up);
            // half of the fov is center to top of screen
            targetDistance *= glm::tan(glm::radians(camera.Fovy) * 0.5f);
            float forceX = -delta.x * heightNorm;
            float forceY = delta.y * heightNorm;

            // we use only clientHeight here so aspect ratio does not distort speed
            _forceDelta = -(q * glm::vec3(2 * forceX  * targetDistance, 2 * forceY  * targetDistance, 0.f));

            ImVec2 windowSize = window->Rect().GetSize();
            glm::vec2 screenPoint = glm::vec2(io.MousePos.x - window->Rect().Min.x, io.MousePos.y - window->Rect().Min.y);
            _rayDirection = GetRayDirection(screenPoint, camera, windowSize);
            _cameraEye = camera.Eye;
        } else {
            _forceDelta = glm::vec3(0.f);
        }
    }

    std::pair<glm::vec3,glm::vec3> ExternalForceManager::getForce(RigidBody::Box const & box) {
        Eigen::Vector3f rayOrigin = glm2eigen(_cameraEye);
        Eigen::Vector3f rayDirection = glm2eigen(_rayDirection);

        // convert to the box space
        Eigen::Vector3f localOrigin = box.Orientation.conjugate() * (rayOrigin - box.Center);
        Eigen::Vector3f localDirection = box.Orientation.conjugate() * rayDirection;

        Eigen::Vector3f halfExtents = box.Dim * 0.5f;

        bool intersect = true;
        float tEnter = -std::numeric_limits<float>::infinity();
        float tExit = std::numeric_limits<float>::infinity();

        const float epsilon = 1e-6f;

        for (int i = 0; i< 3; ++i) {
            if (std::abs(localDirection[i]) < epsilon) {
                // ray is vertical to the current axis
                if (localOrigin[i] < (-halfExtents[i] - epsilon) || localOrigin[i] > (-halfExtents[i] + epsilon)) {
                    intersect = false;
                    break;
                }
            } else {
                float invDirection = 1.f / localDirection[i];
                float t1 = (-halfExtents[i] - localOrigin[i]) * invDirection;
                float t2 = (halfExtents[i] - localOrigin[i]) * invDirection;
                float tMin = std::min(t1, t2);
                float tMax = std::max(t1, t2);

                if (tMin > tEnter) tEnter = tMin;
                if (tMax < tEnter) tExit = tMax;

                if (tEnter > tExit) {
                    intersect = false;
                    break;
                }
            }
        }

        if (intersect && tExit >= 0.f && tEnter <= tExit) {
            float t = std::max(tEnter, 0.f);
            Eigen::Vector3f localHit = localOrigin + localDirection * t;

            bool inside = true;
            for (int i = 0; i < 3; ++i) {
                if (localHit[i] < (-halfExtents[i] - epsilon) || localHit[i] > (halfExtents[i] + epsilon)) {
                    inside = false;
                    break;
                }
            }

            if (inside) {
                // convert to world space
                Eigen::Vector3f globalHit = box.Orientation * localHit + box.Center;
                return std::make_pair(_forceDelta, eigen2glm(globalHit));
            }
        }
        return std::make_pair(glm::vec3(0.f), glm::vec3(0.f));
    }

    std::tuple<glm::vec3,glm::vec3, int> ExternalForceManager::getForce(std::vector<RigidBody::Box> const & box) {
        Eigen::Vector3f rayOrigin = glm2eigen(_cameraEye);
        Eigen::Vector3f rayDirection = glm2eigen(_rayDirection);

        float T = std::numeric_limits<float>::infinity();
        int BoxIdx = -1;
        Eigen::Vector3f LocalHit;

        for (int i = 0; i < box.size(); ++i) {
            // convert to the box space
            Eigen::Vector3f localOrigin = box[i].Orientation.conjugate() * (rayOrigin - box[i].Center);
            Eigen::Vector3f localDirection = box[i].Orientation.conjugate() * rayDirection;

            Eigen::Vector3f halfExtents = box[i].Dim * 0.5f;

            bool intersect = true;
            float tEnter = -std::numeric_limits<float>::infinity();
            float tExit = std::numeric_limits<float>::infinity();

            const float epsilon = 1e-6f;

            for (int i = 0; i< 3; ++i) {
                if (std::abs(localDirection[i]) < epsilon) {
                    // ray is vertical to the current axis
                    if (localOrigin[i] < (-halfExtents[i] - epsilon) || localOrigin[i] > (-halfExtents[i] + epsilon)) {
                        intersect = false;
                        break;
                    }
                } else {
                    float invDirection = 1.f / localDirection[i];
                    float t1 = (-halfExtents[i] - localOrigin[i]) * invDirection;
                    float t2 = (halfExtents[i] - localOrigin[i]) * invDirection;
                    float tMin = std::min(t1, t2);
                    float tMax = std::max(t1, t2);

                    if (tMin > tEnter) tEnter = tMin;
                    if (tMax < tEnter) tExit = tMax;

                    if (tEnter > tExit) {
                        intersect = false;
                        break;
                    }
                }
            }

            if (intersect && tExit >= 0.f && tEnter <= tExit) {
                float t = std::max(tEnter, 0.f);
                Eigen::Vector3f localHit = localOrigin + localDirection * t;

                bool inside = true;
                for (int i = 0; i < 3; ++i) {
                    if (localHit[i] < (-halfExtents[i] - epsilon) || localHit[i] > (halfExtents[i] + epsilon)) {
                        inside = false;
                        break;
                    }
                }

                if (inside && t < T) {
                    T = t;
                    BoxIdx = i;
                    LocalHit = localHit;
                }
            }
        }
        if (BoxIdx != -1) {
            // convert to world space
            Eigen::Vector3f globalHit = box[BoxIdx].Orientation * LocalHit + box[BoxIdx].Center;
            return std::make_tuple(_forceDelta, eigen2glm(globalHit), BoxIdx);
        }
        return std::make_tuple(glm::vec3(0.f), glm::vec3(0.f), 0);
    }

    std::pair<Eigen::Vector3f, int> ExternalForceManager::getForce(std::vector<Eigen::Vector3f> points) {
        Eigen::Vector3f rayOrigin = glm2eigen(_cameraEye);
        Eigen::Vector3f rayDirection = glm2eigen(_rayDirection);

        int nearestPoint = 0;
        float minDistance = std::numeric_limits<float>::max();
        for (int i=0; i < points.size(); i++) {
            Eigen::Vector3f point = points[i];
            Eigen::Vector3f originToPoint = point - rayOrigin;

            float projectionLength = originToPoint.dot(rayDirection);
            if (projectionLength <= 0) {
                continue;
            }
            Eigen::Vector3f nearestPointOnRay = rayOrigin + projectionLength * rayDirection;
            float distance = (nearestPointOnRay - point).norm();

            if (distance < minDistance) {
                minDistance = distance;
                nearestPoint = i;
            }
        }
        if (minDistance < 0.25f) {
            return std::make_pair(glm2eigen(_forceDelta), nearestPoint);
        }
        return std::make_pair(Eigen::Vector3f::Zero(), 0);
    }
} // namespace VCX::Labs::Common
