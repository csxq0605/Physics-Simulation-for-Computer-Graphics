#pragma once

#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>

namespace VCX::Labs::FEM {
    struct TetSystem {
        std::vector<glm::ivec4> Tets;
        std::vector<Eigen::Vector3f> Positions;
        std::vector<Eigen::Vector3f> Velocities;
        std::vector<Eigen::Vector3f> Forces;

        std::vector<Eigen::Vector3f> Positions_;
        std::vector<Eigen::Vector3f> Velocities_;

        std::vector<Eigen::Matrix3f> Bm;
        std::vector<float> W;

        int modelId = 0;

        int wx;
        int wy;
        int wz;
        float delta;

        float young = 20000.0f;
        float poison = 0.2f;
        float density = 400.f;
        float gravity = 0.05f;
        float friction = 10.0f;

        float lambda;
        float mu;

        float particleMass;

        std::vector<Eigen::Vector3f> computeForce(int const idx);
        void AdvanceTetSystem(float const dt);
        void setupSceneSimple();
        void setupScene(int res);

        inline int GetID(std::size_t const i, std::size_t const j, std::size_t const k) {
            return i * (wy + 1) * (wz + 1) + j * (wz + 1) + k;
        }

        inline glm::ivec3 GetCoord(std::size_t const id) {
            int x = id / ((wy + 1) * (wz + 1));
            int y = (id % ((wy + 1) * (wz + 1))) / (wz + 1);
            int z = id % (wz + 1);
            return {x, y, z};
        }

        inline bool IsOnEdge(glm::ivec3 const coord1, glm::ivec3 const coord2) {
            if (coord1[0] == coord2[0] && coord1[1] == coord2[1] && (coord1[0] == 0 || coord1[0] == wx) && (coord1[1] == 0 || coord1[1] == wy)) {
                return true;
            }
            if (coord1[1] == coord2[1] && coord1[2] == coord2[2] && (coord1[1] == 0 || coord1[1] == wy) && (coord1[2] == 0 || coord1[2] == wz)) {
                return true;
            }
            if (coord1[0] == coord2[0] && coord1[2] == coord2[2] && (coord1[0] == 0 || coord1[0] == wx) && (coord1[2] == 0 || coord1[2] == wz)) {
                return true;
            }
            return false;
        }

        void AddParticle(Eigen::Vector3f const & position, Eigen::Vector3f const & velocity = Eigen::Vector3f::Zero()) {
            Positions_.push_back(position);
            Velocities_.push_back(velocity);
        }

        void AddTet(int const a, int const b, int const c, int const d) {
            Tets.push_back({a, b, c, d});
        }
    };
} // namespace VCX::Labs::FEM