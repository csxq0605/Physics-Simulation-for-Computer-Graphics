#include "Labs/2-FluidSimulation/FluidSimulator.h"

namespace VCX::Labs::Fluid {
    void Simulator::integrateParticles(float timeStep) {
        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particleVel[i] += gravity * timeStep;
            m_particlePos[i] += m_particleVel[i] * timeStep;
        }
    }

    void Simulator::pushParticlesApart(int numIters) {
        buildHashtable();
        m_pos.assign(m_particlePos.begin(), m_particlePos.end());
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < m_iNumSpheres; i++) {
                query(i);
                for (int idx = 0; idx < querySize; idx++) {
                    int j = m_queryIndex[idx];
                    if (i == j) continue;
                    glm::vec3 d = m_pos[i] - m_pos[j];
                    float dist = glm::length(d + 1e-3f);
                    if (dist < 2.0f * m_particleRadius) {
                        glm::vec3 s = 0.5f * (2.0f * m_particleRadius - dist) / dist * d;
                        m_pos[i] += s;
                        m_pos[j] -= s;
                    }
                }
            }
        }
        m_particlePos.assign(m_pos.begin(), m_pos.end());
    }

    void Simulator::handleParticleCollisions() {
        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 delta = m_particlePos[i] - obstaclePos;
            float dist = glm::length(delta);
            if (dist < m_particleRadius + obstacleRadius) {
                glm::vec3 normal = delta / dist;
                m_particlePos[i] = obstaclePos + (m_particleRadius + obstacleRadius) * normal;

                glm::vec3 deltaVel = m_particleVel[i] - obstacleVel;
                float deltaVelN = glm::dot(deltaVel, normal);
                if (deltaVelN < 0) {
                    m_particleVel[i] -= deltaVelN * normal;

                    // a self-created trick to avoid a continuous vortex around the obstacle whatever
                    m_particleVel[i].x *= 0.5f;
                    m_particleVel[i].z *= 0.5f;
                }

                // This will make it difficult for particles to slide down the obstacle.
                // m_particleVel[i] = obstacleVel;
            }

            if (m_particlePos[i].x < 0.5f * m_h + m_particleRadius - 0.5f) {
                m_particlePos[i].x = 0.5f * m_h + m_particleRadius - 0.5f;
                m_particleVel[i].x = 0.0f;
            }

            if (m_particlePos[i].x > 0.5f - 0.5f * m_h - m_particleRadius) {
                m_particlePos[i].x = 0.5f - 0.5f * m_h - m_particleRadius;
                m_particleVel[i].x = 0.0f;
            }

            if (m_particlePos[i].y < 0.5f * m_h + m_particleRadius - 0.5f) {
                m_particlePos[i].y = 0.5f * m_h + m_particleRadius - 0.5f;
                m_particleVel[i].y = 0.0f;
            }

            if (m_particlePos[i].y > 0.5f - 0.5f * m_h - m_particleRadius) {
                m_particlePos[i].y = 0.5f - 0.5f * m_h - m_particleRadius;
                m_particleVel[i].y = 0.0f;
            }

            if (m_particlePos[i].z < 0.5f * m_h + m_particleRadius - 0.5f) {
                m_particlePos[i].z = 0.5f * m_h + m_particleRadius - 0.5f;
                m_particleVel[i].z = 0.0f;
            }

            if (m_particlePos[i].z > 0.5f - 0.5f * m_h - m_particleRadius) {
                m_particlePos[i].z = 0.5f - 0.5f * m_h - m_particleRadius;
                m_particleVel[i].z = 0.0f;
            }
        }
    }

    void Simulator::updateParticleDensity() {
        for (int i = 0; i < m_iNumCells; i++) {
            m_particleDensity[i] = 0.0f;
        }
        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            glm::vec3 rel_pos = pos + (0.5f + 0.5f * m_h);
            glm::ivec3 index = glm::ivec3(rel_pos / m_h);
            glm::vec3 delta = (rel_pos - glm::vec3(index) * m_h) / m_h;
            glm::vec3 deltaComplement = glm::vec3(1.0f) - delta;
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 0, 0))] += deltaComplement.x * deltaComplement.y * deltaComplement.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 0, 0))] += delta.x * deltaComplement.y * deltaComplement.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 1, 0))] += deltaComplement.x * delta.y * deltaComplement.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 1, 0))] += delta.x * delta.y * deltaComplement.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 0, 1))] += deltaComplement.x * deltaComplement.y * delta.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 0, 1))] += delta.x * deltaComplement.y * delta.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 1, 1))] += deltaComplement.x * delta.y * delta.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 1, 1))] += delta.x * delta.y * delta.z;
        }
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        if (toGrid) {
            for (int i = 0; i < m_iNumCells; i++) {
                m_vel[i] = glm::vec3(0.0f);
                m_near_num[0][i] = 0.0f;
                m_near_num[1][i] = 0.0f;
                m_near_num[2][i] = 0.0f;
            }

            int n = m_iCellY * m_iCellZ;
            int m = m_iCellZ;

            for (int i = 0; i < m_iCellX; i++) {
                for (int j = 0; j < m_iCellY; j++) {
                    for (int k = 0; k < m_iCellZ; k++) {
                        int index = i * n + j * m + k;
                        if (m_s[index] == 0.0f) {
                            m_type[index] = SOLID_CELL;
                        } else {
                            glm::vec3 pos = glm::vec3(i, j, k) * m_h - 0.5f;
                            glm::vec3 delta = pos - obstaclePos;
                            float dist = glm::length(delta);
                            if (dist < m_particleRadius + obstacleRadius) {
                                m_type[index] = SOLID_CELL;
                            } else {
                                m_type[index] = EMPTY_CELL;
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            glm::ivec3 index = glm::ivec3((pos + (0.5f + 0.5f * m_h)) / m_h);
            if (m_type[index2GridOffset(index)] == EMPTY_CELL) {
                m_type[index2GridOffset(index)] = FLUID_CELL;
            }

            for (int dir = 0; dir < 3; dir++) {
                glm::vec3 offset = glm::vec3(-0.5f);
                offset[dir] -= m_h * 0.5f;

                glm::vec3 rel_pos = pos - offset;
                glm::ivec3 index = glm::ivec3(rel_pos / m_h);

                glm::vec3 delta = (rel_pos - glm::vec3(index) * m_h) / m_h;
                glm::vec3 deltaComplement = glm::vec3(1.0f) - delta;

                if (toGrid) {
                    glm::vec3 vel = m_particleVel[i];
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 0, 0))] += deltaComplement.x * deltaComplement.y * deltaComplement.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 0, 0))] += delta.x * deltaComplement.y * deltaComplement.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 1, 0))] += deltaComplement.x * delta.y * deltaComplement.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 1, 0))] += delta.x * delta.y * deltaComplement.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 0, 1))] += deltaComplement.x * deltaComplement.y * delta.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 0, 1))] += delta.x * deltaComplement.y * delta.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 1, 1))] += deltaComplement.x * delta.y * delta.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 1, 1))] += delta.x * delta.y * delta.z;

                    m_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir] += vel[dir] * deltaComplement.x * deltaComplement.y * deltaComplement.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir] += vel[dir] * delta.x * deltaComplement.y * deltaComplement.z;
                    m_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir] += vel[dir] * deltaComplement.x * delta.y * deltaComplement.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir] += vel[dir] * delta.x * delta.y * deltaComplement.z;
                    m_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir] += vel[dir] * deltaComplement.x * deltaComplement.y * delta.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir] += vel[dir] * delta.x * deltaComplement.y * delta.z;
                    m_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir] += vel[dir] * deltaComplement.x * delta.y * delta.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir] += vel[dir] * delta.x * delta.y * delta.z;
                }
                else {
                    float vel = 0;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir] * deltaComplement.x * deltaComplement.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir] * delta.x * deltaComplement.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir] * deltaComplement.x * delta.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir] * delta.x * delta.y * deltaComplement.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir] * deltaComplement.x * deltaComplement.y * delta.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir] * delta.x * deltaComplement.y * delta.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir] * deltaComplement.x * delta.y * delta.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir] * delta.x * delta.y * delta.z;

                    float deltaVel = 0;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir]) * deltaComplement.x * deltaComplement.y * deltaComplement.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir]) * delta.x * deltaComplement.y * deltaComplement.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir]) * deltaComplement.x * delta.y * deltaComplement.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir]) * delta.x * delta.y * deltaComplement.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir]) * deltaComplement.x * deltaComplement.y * delta.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir]) * delta.x * deltaComplement.y * delta.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir]) * deltaComplement.x * delta.y * delta.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir]) * delta.x * delta.y * delta.z;

                    m_particleVel[i][dir] = (deltaVel + m_particleVel[i][dir]) * flipRatio + (1-flipRatio) * vel;
                }
            }
        }

        if (toGrid) {
            for (int i = 0; i < m_iCellX; i++) {
                for (int j = 0; j < m_iCellY; j++) {
                    for (int k = 0; k < m_iCellZ; k++) {
                        for (int dir = 0; dir < 3; dir++) {
                            if (m_near_num[dir][index2GridOffset(glm::ivec3(i, j, k))] > 0.0f && isValidVelocity(i, j, k, dir)) {
                                m_vel[index2GridOffset(glm::ivec3(i, j, k))][dir] /= m_near_num[dir][index2GridOffset(glm::ivec3(i, j, k))];
                            }
                            else {
                                m_vel[index2GridOffset(glm::ivec3(i, j, k))][dir] = 0.0f;
                            }
                        }
                    }
                }
            }
        }
    }

    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        for (int i = 0; i < m_iNumCells; i++) {
            m_pre_vel[i] = m_vel[i];
        }
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < m_iCellX; i++) {
                for (int j = 0; j < m_iCellY; j++) {
                    for (int k = 0; k < m_iCellZ; k++) {
                        if (m_type[index2GridOffset(glm::vec3(i, j, k))] == FLUID_CELL) {
                            float d = overRelaxation * (
                                m_vel[index2GridOffset(glm::ivec3(i + 1, j, k))].x - m_vel[index2GridOffset(glm::ivec3(i, j, k))].x +
                                m_vel[index2GridOffset(glm::ivec3(i, j + 1, k))].y - m_vel[index2GridOffset(glm::ivec3(i, j, k))].y +
                                m_vel[index2GridOffset(glm::ivec3(i, j, k + 1))].z - m_vel[index2GridOffset(glm::ivec3(i, j, k))].z
                            );
                            if (compensateDrift) {
                                float k = 0.99f;
                                d -= k * (m_particleDensity[index2GridOffset(glm::ivec3(i, j, k))] - m_particleRestDensity);
                            }
                            float s =
                                m_s[index2GridOffset(glm::ivec3(i + 1, j, k))] + m_s[index2GridOffset(glm::ivec3(i - 1, j, k))] +
                                m_s[index2GridOffset(glm::ivec3(i, j + 1, k))] + m_s[index2GridOffset(glm::ivec3(i, j - 1, k))] +
                                m_s[index2GridOffset(glm::ivec3(i, j, k + 1))] + m_s[index2GridOffset(glm::ivec3(i, j, k - 1))]
                            ;
                            m_vel[index2GridOffset(glm::ivec3(i, j, k))].x += (d * m_s[index2GridOffset(glm::ivec3(i - 1, j, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i + 1, j, k))].x -= (d * m_s[index2GridOffset(glm::ivec3(i + 1, j, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j, k))].y += (d * m_s[index2GridOffset(glm::ivec3(i, j - 1, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j + 1, k))].y -= (d * m_s[index2GridOffset(glm::ivec3(i, j + 1, k))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j, k))].z += (d * m_s[index2GridOffset(glm::ivec3(i, j, k - 1))]) / s;
                            m_vel[index2GridOffset(glm::ivec3(i, j, k + 1))].z -= (d * m_s[index2GridOffset(glm::ivec3(i, j, k + 1))]) / s;
                        }
                    }
                }
            }
        }
    }

    void Simulator::updateParticleColors() {
        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            glm::ivec3 index = glm::ivec3((pos + (0.5f + 0.5f * m_h)) / m_h);
            float relDensity = std::clamp(std::sqrt(m_particleDensity[index2GridOffset(index)] * 0.25f), 0.2f, 1.0f);
            m_particleColor[i].x = 1.0f - relDensity;
            m_particleColor[i].y = 1.0f - relDensity;
        }
    }

    inline bool Simulator::isValidVelocity(int i, int j, int k, int dir) {
        glm::ivec3 index(i, j, k);
        if (m_type[index2GridOffset(index)] == SOLID_CELL) {
            return false;
        }
        index[dir] += 1;
        if (m_type[index2GridOffset(index)] == SOLID_CELL) {
            return false;
        }
        return true;
    }

    inline int Simulator::index2GridOffset(glm::ivec3 index) {
        return index.x * m_iCellY * m_iCellZ + index.y * m_iCellZ + index.z;
    }

    inline void Simulator::buildHashtable() {
        std::fill(m_hashtable.begin(), m_hashtable.end(), 0);
        std::fill(m_hashtableindex.begin(), m_hashtableindex.end(), 0);

        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            glm::ivec3 index = glm::ivec3((pos + (0.5f + 0.5f * m_h)) / m_h);
            m_hashtableindex[index2GridOffset(index)]++;
        }

        int prefix_sum = 0;
        for (int i = 0; i < m_iNumCells; i++) {
            prefix_sum += m_hashtableindex[i];
            m_hashtableindex[i] = prefix_sum;
        }
        m_hashtableindex[m_iNumCells] = prefix_sum;

        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            glm::ivec3 index = glm::ivec3((pos + (0.5f + 0.5f * m_h)) / m_h);
            int offset = index2GridOffset(index);
            m_hashtableindex[offset]--;
            m_hashtable[m_hashtableindex[offset]] = i;
        }
    }

    inline void Simulator::query(int index) {
        glm::vec3 pos = m_particlePos[index];
        glm::ivec3 grid_index = glm::ivec3((pos + (0.5f + 0.5f * m_h)) / m_h);
        int grid_offset = index2GridOffset(grid_index);
        querySize = 0;
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = -1; k <= 1; k++) {
                    int offset = index2GridOffset(grid_index + glm::ivec3(i, j, k));
                    int start = m_hashtableindex[offset];
                    int end = m_hashtableindex[offset + 1];

                    for (int idx = start; idx < end; idx++) {
                        m_queryIndex[querySize] = m_hashtable[idx];
                        querySize++;
                    }
                }
            }
        }
    }
}