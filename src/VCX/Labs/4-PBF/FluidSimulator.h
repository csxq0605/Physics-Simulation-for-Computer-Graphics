#pragma once

#include "Labs/4-PBF/parallel.h"

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include <iostream>
#include <utility>
#include <vector>


namespace VCX::Labs::PBF {
    struct Simulator {
        std::vector<glm::vec3> m_particlePos; // Particle Position
        std::vector<glm::vec3> m_particleVel; // Particle Velocity
        std::vector<glm::vec3> m_particleColor;

        std::vector<glm::vec3> m_particlePos_;
        std::vector<std::vector<int>> m_particleNeighbor;

        int m_iCellX;
        int m_iCellY;
        int m_iCellZ;
        int m_iNumCells;
        std::vector<int> m_hashtable;
        std::vector<int> m_hashtableindex;

        int sceneId = 0;
        glm::vec3 tank;
        glm::vec3 relWater;
        glm::vec3 offset;
        float slidePos;
        float slideVel;
        int slideDir;

        int   m_iNumSpheres;
        float m_particleRadius = 0.015f;

        float m_particleRestDensity;
        int NUM = 9;
        float ratio = 3.0f;// ratio between max neighbor distance and particle radius
        float h;// max neighbor distance

        int solverIterations = 5;
        float relaxation = 1e3f;
        float damping = 0.999f;
        float viscosity_coeff = 1e-8f;
        glm::vec3 gravity { 0, -9.81f, 0 };

        glm::vec3   obstaclePos = glm::vec3(0.0f, -0.8f, 0.2f); // obstacle can be moved with mouse, as a user interaction
        const float obstacleRadius = 0.1f;

        float calcDensity(int index);
        float calcConstraint(int index);
        glm::vec3 calcGradConstraint(int index, int neighbor_index);
        void handleCollisions();

        inline int index2GridOffset(glm::ivec3 index);
        void buildHashtable();

        void integrateParticles(float dt);
        void detectNeighbor();
        void constraintSolve();
        void velocityUpdate(float dt);
        void updateParticleColors();

        void SimulateTimestep(float const dt);
        void setupScene();
    };
} // namespace VCX::Labs::PBF