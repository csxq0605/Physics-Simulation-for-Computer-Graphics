#include "Labs/3-FEM/TetSystem.h"

namespace VCX::Labs::FEM {
    std::vector<Eigen::Vector3f> TetSystem::computeForce(int const idx) {
        Eigen::Vector3f x0 = Positions[Tets[idx][0]];
        Eigen::Vector3f x1 = Positions[Tets[idx][1]];
        Eigen::Vector3f x2 = Positions[Tets[idx][2]];
        Eigen::Vector3f x3 = Positions[Tets[idx][3]];

        Eigen::Matrix3f Ds;
        Ds << x1 - x0, x2 - x0, x3 - x0;

        Eigen::Matrix3f F = Ds * Bm[idx];

        // Eigen::Matrix3f G = 0.5 * (F.transpose() * F - Eigen::Matrix3f::Identity());
        // Eigen::Matrix3f S = 2 * mu * G + lambda * G.trace() * Eigen::Matrix3f::Identity();
        // Eigen::Matrix3f P = F * S;

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(F, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();
        Eigen::Vector3f Diag = svd.singularValues();

        float Ic = Diag.squaredNorm();
        Eigen::Vector3f dIc = 2 * Diag;
        Eigen::Vector3f dW;
        if (modelId == 0) {
            // StVK
            Eigen::Vector3f dIIc = 4 * Diag.array().pow(3);
            dW = lambda * (Ic - 3) * dIc + 0.25 * mu * (dIIc - 2 * dIc);
        } else {
            // Neo-Hookean
            dW = 0.5 * mu * dIc + (lambda * log(Diag.prod()) - mu) * Diag.cwiseInverse();
        }

        Eigen::Matrix3f P = U * dW.asDiagonal() * V.transpose();

        Eigen::Matrix3f f = - W[idx] * P * Bm[idx].transpose();
        Eigen::Vector3f f1 = f.col(0);
        Eigen::Vector3f f2 = f.col(1);
        Eigen::Vector3f f3 = f.col(2);
        Eigen::Vector3f f0 = - f1 - f2 - f3;

        return { f0, f1, f2, f3 };
    }

    void TetSystem::AdvanceTetSystem(float const dt) {
        lambda = young * poison / ((1 + poison) * (1 - 2 * poison));
        mu = young / (2 * (1 + poison));

        for (int i = 0; i < Positions.size(); i++) {
            Forces[i][1] -= gravity * particleMass;
            Forces[i] -= friction * Velocities[i];
        }

        for (int i = 0; i < Tets.size(); i++) {
            std::vector<Eigen::Vector3f> force = computeForce(i);
            for (int j = 0; j < 4; j++) {
                Forces[Tets[i][j]] += force[j];
            }
        }

        for (int i = 0; i < Velocities.size(); i++) {
            if(i >= (wy + 1) * (wz + 1)) {
                // not fixed
                Velocities[i] += Forces[i] / particleMass * dt;
            }
        }

        for (int i = 0; i < Positions.size(); i++) {
            Positions[i] += Velocities[i] * dt;
        }

        for (int i = 0; i < Forces.size(); i++) {
            Forces[i].setZero();
        }
    }

    void TetSystem::setupSceneSimple() {
        Positions_ = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        Velocities_ = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        wx = wy = wz = 0;
        particleMass = 100;
        AddTet(0,1,2,3);
        
        for (int i = 0; i < Tets.size(); i++) {
            Eigen::Vector3f X0 = Positions_[Tets[i][0]];
            Eigen::Vector3f X1 = Positions_[Tets[i][1]];
            Eigen::Vector3f X2 = Positions_[Tets[i][2]];
            Eigen::Vector3f X3 = Positions_[Tets[i][3]];

            Eigen::Matrix3f Dm;
            Dm << X1 - X0, X2 - X0, X3 - X0;

            Bm.push_back(Dm.inverse());
            W.push_back(abs(Dm.determinant()) / 6);
        }

        Positions = Positions_;
        Velocities = Velocities_;
        Forces.resize(Positions.size(), Eigen::Vector3f::Zero());
    }
    void TetSystem::setupScene(int res) {
        float delta = 1.0f / res;
        wx = 8 * res;
        wy = 2 * res;
        wz = 2 * res;
        particleMass = delta * delta * delta * density;

        for (std::size_t i = 0; i <= wx; i++) {
            for (std::size_t j = 0; j <= wy; j++) {
                for (std::size_t k = 0; k <= wz; k++) {
                    AddParticle({ i * delta, j * delta, k * delta});
                }
            }
        }

        for (std::size_t i = 0; i < wx; i++) {
            for (std::size_t j = 0; j < wy; j++) {
                for (std::size_t k = 0; k < wz; k++) {
                    AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                }
            }
        }

        for (int i = 0; i < Tets.size(); i++) {
            Eigen::Vector3f X0 = Positions_[Tets[i][0]];
            Eigen::Vector3f X1 = Positions_[Tets[i][1]];
            Eigen::Vector3f X2 = Positions_[Tets[i][2]];
            Eigen::Vector3f X3 = Positions_[Tets[i][3]];

            Eigen::Matrix3f Dm;
            Dm << X1 - X0, X2 - X0, X3 - X0;

            Bm.push_back(Dm.inverse());
            W.push_back(abs(Dm.determinant()) / 6);
        }

        Positions = Positions_;
        Velocities = Velocities_;
        Forces.resize(Positions.size(), Eigen::Vector3f::Zero());
    }
}