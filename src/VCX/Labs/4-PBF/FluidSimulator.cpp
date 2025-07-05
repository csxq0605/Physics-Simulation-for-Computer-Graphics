#include "Labs/4-PBF/FluidSimulator.h"
#include "Labs/4-PBF/parallel.h"

namespace VCX::Labs::PBF {
    static constexpr float invPI = 0.318301;

    static inline float squaredNorm(const glm::vec3 &x) {
        return x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
    }

    static inline float Poly6(const glm::vec3 &r, float h) {
        float r2 = squaredNorm(r);
        float h2 = h * h;
        float diff = h2 - r2;
        if (diff < 0) {
            return 0.0f;
        }
        constexpr float coeff = 315 * invPI / 64;
        float diff3 = diff * diff * diff;
        float h4 = h2 * h2;
        float h9 = h4 * h4 * h;
        return coeff * diff3 / h9;
    }

    static inline glm::vec3 GradSpiky(const glm::vec3 &r, float h) {
        float r2 = squaredNorm(r);
        float h2 = h * h;
        if (r2 > h2) {
            return glm::vec3(0.0f);
        }
        constexpr float coeff = -45 * invPI;
        float r_norm = sqrt(r2);
        float diff = h - r_norm;
        float h3 = h * h * h;
        float h6 = h3 * h3;
        return coeff * diff * diff / (h6 * std::max(r_norm, 1e-24f)) * r;
    }

    float Simulator::calcDensity(int index) {
        float density = 0.0f;
        glm::vec3 pos = m_particlePos_[index];
        for (int i: m_particleNeighbor[index]) {
            density += Poly6(pos - m_particlePos_[i], h);
        }
        return density;
    }

    float Simulator::calcConstraint(int index) {
        return calcDensity(index) / m_particleRestDensity - 1;
    }

    glm::vec3 Simulator::calcGradConstraint(int index, int neighbor_index) {
        glm::vec3 gradC(0.0f);
        if (neighbor_index == index) {
            for (int i: m_particleNeighbor[index]) {
                gradC += GradSpiky(m_particlePos_[index] - m_particlePos_[i], h);
            }
        } else {
            gradC -= GradSpiky(m_particlePos_[index] - m_particlePos_[neighbor_index], h);
        }
        return gradC / m_particleRestDensity;
    }

    void Simulator::handleCollisions() {
        for (int i = 0; i < m_iNumSpheres; i++) {
            if (sceneId == 0) {
                glm::vec3 delta = m_particlePos_[i] - obstaclePos;
                float dist = glm::length(delta);
                if (dist < m_particleRadius + obstacleRadius) {
                    glm::vec3 normal = delta / dist;
                    m_particlePos_[i] = obstaclePos + (m_particleRadius + obstacleRadius) * normal;
                }
            }

            if (m_particlePos_[i][0] < -0.5f * tank[0] + m_particleRadius) {
                m_particlePos_[i][0] = -0.5f * tank[0] + m_particleRadius;
            }
            if (m_particlePos_[i][0] > 0.5f * tank[0] * slidePos - m_particleRadius) {
                m_particlePos_[i][0] = 0.5f * tank[0] * slidePos - m_particleRadius;
            }

            if (m_particlePos_[i][1] < -0.5f * tank[1] + m_particleRadius) {
                m_particlePos_[i][1] = -0.5f * tank[1] + m_particleRadius;
            }
            if (m_particlePos_[i][1] > 0.5f * tank[1] - m_particleRadius) {
                m_particlePos_[i][1] = 0.5f * tank[1] - m_particleRadius;
            }

            if (m_particlePos_[i][2] < -0.5f * tank[2] + m_particleRadius) {
                m_particlePos_[i][2] = -0.5f * tank[2] + m_particleRadius;
            }
            if (m_particlePos_[i][2] > 0.5f * tank[2] - m_particleRadius) {
                m_particlePos_[i][2] = 0.5f * tank[2] - m_particleRadius;
            }
        }
    }

    inline int Simulator::index2GridOffset(glm::ivec3 index) {
        return index.x * m_iCellY * m_iCellZ + index.y * m_iCellZ + index.z;
    }

    void Simulator::buildHashtable() {
        std::fill(m_hashtable.begin(), m_hashtable.end(), 0);
        std::fill(m_hashtableindex.begin(), m_hashtableindex.end(), 0);

        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos_[i];
            glm::ivec3 index = glm::ivec3((pos + (0.5f * tank)) / h) + 1;
            m_hashtableindex[index2GridOffset(index)]++;
        }

        int prefix_sum = 0;
        for (int i = 0; i < m_iNumCells; i++) {
            prefix_sum += m_hashtableindex[i];
            m_hashtableindex[i] = prefix_sum;
        }
        m_hashtableindex[m_iNumCells] = prefix_sum;

        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos_[i];
            glm::ivec3 index = glm::ivec3((pos + (0.5f * tank)) / h) + 1;
            int offset = index2GridOffset(index);
            m_hashtableindex[offset]--;
            m_hashtable[m_hashtableindex[offset]] = i;
        }
    }

    void Simulator::integrateParticles(float dt) {
        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particleVel[i] += gravity * dt;
            m_particlePos_[i] += m_particleVel[i] * dt;
        }
    }

    void Simulator::detectNeighbor() {
        handleCollisions();
        buildHashtable();
        m_particleNeighbor.clear();
        m_particleNeighbor.resize(m_iNumSpheres);
        parallel_for(m_iNumSpheres, [&](const int p) {
            glm::vec3 pos = m_particlePos_[p];
            glm::ivec3 grid_index = glm::ivec3((pos + (0.5f * tank)) / h) + 1;
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    for (int k = -1; k <= 1; k++) {
                        int offset = index2GridOffset(grid_index + glm::ivec3(i, j, k));
                        int start = m_hashtableindex[offset];
                        int end = m_hashtableindex[offset + 1];

                        for (int idx = start; idx < end; idx++) {
                            int q = m_hashtable[idx];
                            if (q == p) {
                                continue;
                            }
                            glm::vec3 d = m_particlePos_[q] - pos;
                            float dist2 = squaredNorm(d);
                            if (dist2 < h * h) {
                                m_particleNeighbor[p].push_back(m_hashtable[idx]);
                            }
                        }
                    }
                }
            }
            // for (int j = 0; j < m_iNumSpheres; j++) {
            //     if (j == p) {
            //         continue;
            //     }
            //     glm::vec3 d = m_particlePos_[j] - pos;
            //     float dist2 = squaredNorm(d);
            //     if (dist2 < h * h) {
            //         m_particleNeighbor[p].push_back(j);
            //     }
            // }
        });
    }

    void Simulator::constraintSolve() {
        std::vector<float> lambda(m_iNumSpheres, 0.0f);
        std::vector<glm::vec3> deltaPos(m_iNumSpheres, glm::vec3(0.0f));
        parallel_for(m_iNumSpheres, [&](const int i) {
            float numerator = calcConstraint(i);
            float denominator = 0.0f;
            for (int j: m_particleNeighbor[i]) {
                glm::vec3 gradC = calcGradConstraint(i, j);
                denominator += squaredNorm(gradC);
            }
            denominator += squaredNorm(calcGradConstraint(i, i));
            denominator += relaxation;
            lambda[i] = -numerator / denominator;
        });

        constexpr float k = 1e-5f;
        constexpr float n = 4.0f;
        const float w = Poly6({0.3f * h, 0.0f, 0.0f}, h);
        parallel_for(m_iNumSpheres, [&](const int i) {
            glm::vec3 pos = m_particlePos_[i];
            for (int j: m_particleNeighbor[i]) {
                if (j == i) {
                    continue;
                }
                float ratio = Poly6(pos - m_particlePos_[j], h) / w;
                float s_corr = -k * pow(ratio, n);
                deltaPos[i] += (lambda[i] + lambda[j] + s_corr) * GradSpiky(pos - m_particlePos_[j], h);
            }
            deltaPos[i] /= m_particleRestDensity;
        });

        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particlePos_[i] += deltaPos[i];
        }
        handleCollisions();
    }

    void Simulator::velocityUpdate(float dt) {
        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particleVel[i] = damping * (m_particlePos_[i] - m_particlePos[i]) / dt;
            m_particlePos[i] = m_particlePos_[i];
        }

        // XSPH viscosity
        // std::vector<glm::vec3> deltaVel(m_iNumSpheres, glm::vec3(0.0f));
        // for (int i = 0; i < m_iNumSpheres; i++) {
        //     glm::vec3 pos = m_particlePos_[i];
        //     glm::vec3 vel = m_particleVel[i];
        //     glm::vec3 sum(0.0f);
        //     for (int j: m_particleNeighbor[i]) {
        //         sum += (vel - m_particleVel[j]) * Poly6(pos - m_particlePos_[j], h);
        //     }
        //     deltaVel[i] = viscosity_coeff * sum;
        // }
        // for (int i = 0; i < m_iNumSpheres; i++) {
        //     m_particleVel[i] -= deltaVel[i];
        // }
    }

    void Simulator::updateParticleColors() {
        for (int i = 0; i < m_iNumSpheres; i++) {
            float relDensity = std::clamp(std::sqrt(m_particleNeighbor[i].size() / 13.0f), 0.7f, 1.0f);
            m_particleColor[i].x = 1.0f - relDensity;
            m_particleColor[i].y = 1.0f - (1.0f - 30.0f / 255) * relDensity;
        }
    }

    void Simulator::SimulateTimestep(float const dt) {
        if (sceneId == 1) {
            slidePos += slideDir * slideVel * dt;
            if (slidePos > 1.0f) {
                slideDir = -1;
                slidePos = 2.0f - slidePos;
            } else if (slidePos < 0.5f) {
                slideDir = 1;
                slidePos = 1.0f - slidePos;
            }
        }
        integrateParticles(dt);
        detectNeighbor();
        for (int iter = 0; iter < solverIterations; iter++) {
            constraintSolve();
        }
        velocityUpdate(dt);
        updateParticleColors();
    }

    void Simulator::setupScene() {
        glm::vec3 base = -tank * 0.5f + offset * (glm::vec3(1.0f) - relWater) * tank;

        float dx = 2.0 * m_particleRadius;
        float dy = sqrt(3.0) / 2.0 * dx;
        float dz = dx;

        int numX = floor(relWater.x * tank.x / dx);
        int numY = floor(relWater.y * tank.y / dy);
        int numZ = floor(relWater.z * tank.z / dz);

        // update object member attributes
        m_iNumSpheres = numX * numY * numZ;
        h = m_particleRadius * ratio;
        m_iCellX = ceil(tank[0] / h) + 2;
        m_iCellY = ceil(tank[1] / h) + 2;
        m_iCellZ = ceil(tank[2] / h) + 2;
        m_iNumCells = m_iCellX * m_iCellY * m_iCellZ;

        // update particle array
        m_particlePos.clear();
        m_particlePos.resize(m_iNumSpheres, glm::vec3(0.0f));
        m_particleVel.clear();
        m_particleVel.resize(m_iNumSpheres, glm::vec3(0.0f));
        m_particleColor.clear();
        m_particleColor.resize(m_iNumSpheres, glm::vec3(0.0f, 30.0f / 255, 1.0f));

        m_particlePos_.clear();
        m_particlePos_.resize(m_iNumSpheres, glm::vec3(0.0f));
        m_particleNeighbor.clear();
        m_particleNeighbor.resize(m_iNumSpheres);

        m_hashtable.clear();
        m_hashtable.resize(m_iNumSpheres, 0);
        m_hashtableindex.clear();
        m_hashtableindex.resize(m_iNumCells + 1, 0);

        // the rest density can be assigned after scene initialization
        constexpr float factor = invPI * 315 * 5 * 5 * 5 / (64 * 9 * 9 * 9);
        m_particleRestDensity = factor * NUM / (h * h * h);

        // create particles
        int p = 0;
        for (int i = 0; i < numX; i++) {
            for (int j = 0; j < numY; j++) {
                for (int k = 0; k < numZ; k++) {
                    m_particlePos[p++] = glm::vec3(m_particleRadius + dx * i + (j % 2 == 0 ? 0.0 : m_particleRadius), m_particleRadius + dy * j, m_particleRadius + dz * k + (j % 2 == 0 ? 0.0 : m_particleRadius)) + base;
                }
            }
        }

        m_particlePos_ = m_particlePos;
    }
}