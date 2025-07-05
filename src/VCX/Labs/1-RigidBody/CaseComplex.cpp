#include <random>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseComplex.h"
#include "Labs/1-RigidBody/Collision.h"
#include "Labs/Common/ImGuiHelper.h"

static glm::vec3 eigen2glm(Eigen::Vector3f const & eigen_v) {
    return glm::vec3(eigen_v[0], eigen_v[1], eigen_v[2]);
}

static Eigen::Vector3f glm2eigen(glm::vec3 const & glm_v) {
    return Eigen::Vector3f(glm_v[0], glm_v[1], glm_v[2]);
}

static std::random_device rd;
static std::mt19937 gen(rd());

static Eigen::Vector3f randomVector(Eigen::Vector3f low, Eigen::Vector3f high) {
    std::uniform_real_distribution<float> distX(low[0], high[0]);
    std::uniform_real_distribution<float> distY(low[1], high[1]);
    std::uniform_real_distribution<float> distZ(low[2], high[2]);
    return Eigen::Vector3f(distX(gen), distY(gen), distZ(gen));
}

static Eigen::Quaternionf randomQuaternionf() {
    static std::normal_distribution<float> dist;
    Eigen::Vector3f randomOrientation(dist(gen), dist(gen), dist(gen));
    randomOrientation.normalize();
    return Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), randomOrientation);
}

namespace VCX::Labs::RigidBody {
    CaseComplex::CaseComplex():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        _box.reserve(11);
        _box.push_back(Box(true));
        _collisionResult.reserve(55);

        _box[0].Center = Eigen::Vector3f{0, -5, 0};
        _box[0].Dim = Eigen::Vector3f{8, 0.5, 8};

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetSystem();
    }

    void CaseComplex::OnSetupPropsUI() {
        if (ImGui::Button("Reset System")) ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
            ImGui::SliderInt("Box Num", &_boxNum, 4, 10);
        }
        if (ImGui::CollapsingHeader("Physics Parameter", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Restitution", &_restitution, 0, 1);
            ImGui::SliderFloat("Gravity", &_gravity, 0.1, 10);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseComplex::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) {
            // apply mouse control first
            OnProcessMouseControl(_forceManager.getForce(_box), Engine::GetDeltaTime());

            AdvanceComplexRB(Engine::GetDeltaTime());
        }

        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        std::vector<Eigen::Vector3f> VertsPosition;
        VertsPosition.resize(8 * _box.size());
        for (int i = 0; i < _box.size(); ++i) {
            Eigen::Vector3f        new_x = _box[i].Orientation * Eigen::Vector3f(_box[i].Dim[0] / 2, 0.f, 0.f);
            Eigen::Vector3f        new_y = _box[i].Orientation * Eigen::Vector3f(0.f, _box[i].Dim[1] / 2, 0.f);
            Eigen::Vector3f        new_z = _box[i].Orientation * Eigen::Vector3f(0.f, 0.f, _box[i].Dim[2] / 2);
            VertsPosition[0 + 8 * i] = _box[i].Center - new_x + new_y + new_z;
            VertsPosition[1 + 8 * i] = _box[i].Center + new_x + new_y + new_z;
            VertsPosition[2 + 8 * i] = _box[i].Center + new_x + new_y - new_z;
            VertsPosition[3 + 8 * i] = _box[i].Center - new_x + new_y - new_z;
            VertsPosition[4 + 8 * i] = _box[i].Center - new_x - new_y + new_z;
            VertsPosition[5 + 8 * i] = _box[i].Center + new_x - new_y + new_z;
            VertsPosition[6 + 8 * i] = _box[i].Center + new_x - new_y - new_z;
            VertsPosition[7 + 8 * i] = _box[i].Center - new_x - new_y - new_z;
        }

        auto span_bytes = Engine::make_span_bytes<Eigen::Vector3f>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseComplex::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        _forceManager.ProcessInput(_camera, pos);
    }

    void CaseComplex::OnProcessMouseControl(std::tuple<glm::vec3,glm::vec3, int> force, float dt) {
        Eigen::Vector3f forceDelta = glm2eigen(std::get<0>(force));
        Eigen::Vector3f forcePoint = glm2eigen(std::get<1>(force));
        int boxIdx = std::get<2>(force);
        if (!_box[boxIdx].Fixed) {
            Eigen::Vector3f torque = (forcePoint - _box[boxIdx].Center).cross(forceDelta);
            _box[boxIdx].AngularVelocity += _box[boxIdx].GetInertiaMatrix().inverse() * torque;
            _box[boxIdx].Velocity += forceDelta;
        }
    }

    void CaseComplex::AdvanceComplexRB(float dt) {
        for (int i = 0; i < _box.size(); ++i) {
            if (!_box[i].Fixed) {
                _box[i].Velocity[1] -= dt * _gravity;

                _box[i].Center += dt * _box[i].Velocity;

                Eigen::Quaternionf _angularVelocityQuaternion(0, _box[i].AngularVelocity[0] * dt * 0.5f, _box[i].AngularVelocity[1] * dt * 0.5f, _box[i].AngularVelocity[2] * dt * 0.5f);

                _box[i].Orientation.coeffs() += (_angularVelocityQuaternion * _box[i].Orientation).coeffs();
                _box[i].Orientation.normalize();
            }
        }
        DetectCollision();
        SolveCollision();
    }

    void CaseComplex::DetectCollision() {
        for (int i = 1; i < _box.size(); ++i) {
            for (int j = 0; j < i; ++j) {
                RigidBody::DetectCollision(_box[i], _box[j], _collisionResult[i * (i - 1) / 2 + j]);
            }
        }
    }

    void CaseComplex::SolveCollision() {
        for (int i = 1; i < _box.size(); ++i) {
            for (int j = 0; j < i; ++j) {
                RigidBody::SolveCollision(_box[i], _box[j], _collisionResult[i * (i - 1) / 2 + j], _restitution);
            }
        }
    }

    void CaseComplex::ResetSystem() {
        if (_boxNum + 1 != _box.size()) {
            //     3-----2
            //    /|    /|
            //   0 --- 1 |
            //   | 7 - | 6
            //   |/    |/
            //   4 --- 5
            const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
            std::vector<std::uint32_t> line_index_(line_index.size() * (_boxNum + 1));
            for (int i = 0; i <= _boxNum; ++i) {
                for (int j = 0; j < line_index.size(); ++j) {
                    line_index_[j + i * line_index.size()] = line_index[j] + 8 * i;
                }
            }
            _lineItem.UpdateElementBuffer(line_index_);

            const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
            std::vector<std::uint32_t> tri_index_(tri_index.size() * (_boxNum + 1));
            for (int i = 0; i <= _boxNum; ++i) {
                for (int j = 0; j < tri_index.size(); ++j) {
                    tri_index_[j + i * tri_index.size()] = tri_index[j] + 8 * i;
                }
            }
            _boxItem.UpdateElementBuffer(tri_index_);

            _box.resize(_boxNum + 1);
            _collisionResult.resize(_boxNum * (_boxNum + 1) / 2);
        }
        // regenerate boxes
        float randomness = 0.2;
        for (int i = 1; i < _box.size(); ++i) {
            _box[i] = Box::Box();
            _box[i].Center = randomVector({-randomness, -1 + 2 * i - randomness, -randomness},{randomness, -1 + 2 * i + randomness, randomness});
            _box[i].Dim = Eigen::Vector3f(1, 1, 1);
            _box[i].Orientation = randomQuaternionf();
        }
    }
} // namespace VCX::Labs::RigidBody
