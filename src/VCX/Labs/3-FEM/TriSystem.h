#pragma once

#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>

namespace VCX::Labs::FEM {
    struct TriSystem {
        std::vector<glm::ivec3> Tris;
        std::vector<Eigen::Vector3f> Positions;
        std::vector<Eigen::Vector3f> Velocities;
        std::vector<Eigen::Vector3f> Forces;

        std::vector<Eigen::Vector3f> Positions_;
        std::vector<Eigen::Vector3f> Velocities_;

        std::vector<Eigen::Matrix2f> Bm;
        std::vector<float> W;

        int modelId = 0;

        int wx;
        int wz;
        float delta;

        float young = 50.0f;
        float poison = 0.3f;
        float density = 0.5f;
        float gravity = 9.8f;
        float friction = 0.01f;

        float lambda;
        float mu;

        float particleMass;

        std::vector<Eigen::Vector3f> computeForce(int const idx);
        void AdvanceTetSystem(float const dt);
        void setupSceneSimple();
        void setupScene(int res);

        inline int GetID(std::size_t const i, std::size_t const j) {
            return i * (wz + 1) + j;
        }

        inline glm::ivec2 GetCoord(std::size_t const id) {
            int x = id / (wz + 1);
            int z = id % (wz + 1);
            return {x, z};
        }

        inline bool IsOnEdge(glm::ivec2 const coord1, glm::ivec2 const coord2) {
            if (coord1[0] == coord2[0] && (coord1[0] == 0 || coord1[0] == wx)) {
                return true;
            }
            if (coord1[1] == coord2[1] && (coord1[1] == 0 || coord1[1] == wz)) {
                return true;
            }
            return false;
        }

        void AddParticle(Eigen::Vector3f const & position, Eigen::Vector3f const & velocity = Eigen::Vector3f::Zero()) {
            Positions_.push_back(position);
            Velocities_.push_back(velocity);
        }

        void AddTri(int const a, int const b, int const c) {
            Tris.push_back({a, b, c});
        }
    };
} // namespace VCX::Labs::FEM