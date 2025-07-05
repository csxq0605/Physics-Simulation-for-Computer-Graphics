#include "Labs/3-FEM/TriSystem.h"

namespace VCX::Labs::FEM {
    std::vector<Eigen::Vector3f> TriSystem::computeForce(int const idx) {
        Eigen::Vector3f x0 = Positions[Tris[idx][0]];
        Eigen::Vector3f x1 = Positions[Tris[idx][1]];
        Eigen::Vector3f x2 = Positions[Tris[idx][2]];

        Eigen::MatrixXf Ds(3, 2);
        Ds << x1 - x0, x2 - x0;

        Eigen::MatrixXf F = Ds * Bm[idx];

        Eigen::MatrixXf P;

        if (modelId == 0) {
            // StVK
            Eigen::Matrix2f G = 0.5 * (F.transpose() * F - Eigen::Matrix2f::Identity());
            Eigen::Matrix2f S = 2 * mu * G + lambda * G.trace() * Eigen::Matrix2f::Identity();
            P = F * S;
        } else {
            // Neo-Hookean
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(F, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::MatrixXf U = svd.matrixU();
            Eigen::MatrixXf V = svd.matrixV();
            Eigen::VectorXf Diag = svd.singularValues();

            float Ic = Diag.squaredNorm();
            Eigen::VectorXf dIc = 2 * Diag;
            Eigen::VectorXf dW = 0.5 * mu * dIc + (lambda * log(Diag.prod()) - mu) * Diag.cwiseInverse();

            P = U * dW.asDiagonal() * V.transpose();
        }

        Eigen::MatrixXf f = - W[idx] * P * Bm[idx].transpose();
        Eigen::Vector3f f1 = f.col(0);
        Eigen::Vector3f f2 = f.col(1);
        Eigen::Vector3f f0 = - f1 - f2;

        return { f0, f1, f2 };
    }

    void TriSystem::AdvanceTetSystem(float const dt) {
        lambda = young * poison / ((1 + poison) * (1 - 2 * poison));
        mu = young / (2 * (1 + poison));

        for (int i = 0; i < Positions.size(); i++) {
            Forces[i][1] -= gravity * particleMass;
            Forces[i] -= friction * Velocities[i];
        }

        for (int i = 0; i < Tris.size(); i++) {
            std::vector<Eigen::Vector3f> force = computeForce(i);
            for (int j = 0; j < 3; j++) {
                Forces[Tris[i][j]] += force[j];
            }
        }

        for (int i = 0; i < Velocities.size(); i++) {
            if(i != 0 && i != wz) {
                // not fixed
                Velocities[i] += Forces[i] / particleMass * dt;
            }
        }

        for (int i = 0; i < Positions.size(); i++) {
            Positions[i] += Velocities[i] * dt;
        }

        // printf("%f, %f, %f\n", Positions[2][0], Positions[2][1], Positions[2][2]);
        // printf("%f, %f, %f\n", Velocities[2][0], Velocities[2][1], Velocities[2][2]);

        for (int i = 0; i < Forces.size(); i++) {
            Forces[i].setZero();
        }
    }

    void TriSystem::setupSceneSimple() {
        Positions_ = {{0, 0, 0}, {0, 0, 1}, {1, 0, 0.5}};
        Velocities_ = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        wx = wz = 0;
        particleMass = 0.2;
        AddTri(0,1,2);

        for (int i = 0; i < Tris.size(); i++) {
            Eigen::Vector3f X0 = Positions_[Tris[i][0]];
            Eigen::Vector3f X1 = Positions_[Tris[i][1]];
            Eigen::Vector3f X2 = Positions_[Tris[i][2]];

            Eigen::Matrix2f Dm;
            Dm << X1[0] - X0[0], X1[2] - X0[2], X2[0] - X0[0], X2[2] - X0[2];

            Bm.push_back(Dm.inverse());
            W.push_back(abs(Dm.determinant()) / 2);
        }

        Positions = Positions_;
        Velocities = Velocities_;
        Forces.resize(Positions.size(), Eigen::Vector3f::Zero());
    }

    void TriSystem::setupScene(int res) {
        float delta = 1.0f / res;
        wx = 2 * res;
        wz = 2 * res;
        particleMass = delta * delta * density;

        for (std::size_t i = 0; i <= wx; i++) {
            for (std::size_t j = 0; j <= wz; j++) {
                AddParticle({ i * delta, 0, j * delta});
            }
        }

        for (std::size_t i = 0; i < wx; i++) {
            for (std::size_t j = 0; j < wz; j++) {
                AddTri(GetID(i, j), GetID(i, j + 1), GetID(i + 1, j));
                AddTri(GetID(i + 1, j), GetID(i, j + 1), GetID(i + 1, j + 1));
            }
        }

        for (int i = 0; i < Tris.size(); i++) {
            Eigen::Vector3f X0 = Positions_[Tris[i][0]];
            Eigen::Vector3f X1 = Positions_[Tris[i][1]];
            Eigen::Vector3f X2 = Positions_[Tris[i][2]];

            Eigen::Matrix2f Dm;
            Dm << X1[0] - X0[0], X2[0] - X0[0], X1[2] - X0[2], X2[2] - X0[2];

            Bm.push_back(Dm.inverse());
            W.push_back(abs(Dm.determinant()) / 2);
        }

        Positions = Positions_;
        Velocities = Velocities_;
        Forces.resize(Positions.size(), Eigen::Vector3f::Zero());
    }
}