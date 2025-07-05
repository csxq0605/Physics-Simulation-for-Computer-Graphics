#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseDouble.h"
#include "Labs/1-RigidBody/Collision.h"
#include "Labs/Common/ImGuiHelper.h"

static glm::vec3 eigen2glm(Eigen::Vector3f const & eigen_v) {
    return glm::vec3(eigen_v[0], eigen_v[1], eigen_v[2]);
}

static Eigen::Vector3f glm2eigen(glm::vec3 const & glm_v) {
    return Eigen::Vector3f(glm_v[0], glm_v[1], glm_v[2]);
}

namespace VCX::Labs::RigidBody {
    static constexpr auto Scenes = std::array<char const *, 3> {
        "Edge-Edge Collision",
        "Vertex-Face Collision",
        "Face-Face Collision"
    };

    static const std::array<std::pair<Eigen::Vector3f, Eigen::Vector3f>, 3> initialBoxCenters = {
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0, 0, 0}, Eigen::Vector3f{3, -0.5, 0}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0, 0, 0}, Eigen::Vector3f{3, 0, 0}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0, 0, 0}, Eigen::Vector3f{3, 0.4, 0.4}}
    };

    static const std::array<std::pair<Eigen::Vector3f, Eigen::Vector3f>, 3> initialBoxDims = {
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{0.5, 0.5, 1.5}, Eigen::Vector3f{0.5, 1.5, 0.5}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{1, 1, 1}, Eigen::Vector3f{1, 1, 1}},
        std::pair<Eigen::Vector3f, Eigen::Vector3f>{Eigen::Vector3f{1, 1, 1}, Eigen::Vector3f{1, 1, 1}}
    };

    static const std::array<std::pair<Eigen::Quaternionf, Eigen::Quaternionf>, 3> initialBoxOrientations = {
        std::pair<Eigen::Quaternionf, Eigen::Quaternionf>{Eigen::Quaternionf{1, 0, 0, 0}, Eigen::Quaternionf{0.3, 0.3, 0.9, 0.1}},
        std::pair<Eigen::Quaternionf, Eigen::Quaternionf>{Eigen::Quaternionf{1, 0, 0, 0}, Eigen::Quaternionf{0.3, 0.9, 0.3, 0.1}},
        std::pair<Eigen::Quaternionf, Eigen::Quaternionf>{Eigen::Quaternionf{1, 0, 0, 0}, Eigen::Quaternionf{1, 0, 0, 0}}
    };

    CaseDouble::CaseDouble():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        std::vector<std::uint32_t> line_index_(line_index.size() * 2);
        for (int i = 0; i < line_index.size(); ++i) {
            line_index_[i] = line_index[i];
            line_index_[i + line_index.size()] = line_index[i] + 8;
        }
        _lineItem.UpdateElementBuffer(line_index_);

        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        std::vector<std::uint32_t> tri_index_(tri_index.size() * 2);
        for (int i = 0; i < tri_index.size(); ++i) {
            tri_index_[i] = tri_index[i];
            tri_index_[i + tri_index.size()] = tri_index[i] + 8;
        }
        _boxItem.UpdateElementBuffer(tri_index_);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        _initialBox[0].Velocity = Eigen::Vector3f(0.5, 0, 0);
        _initialBox[1].Velocity = Eigen::Vector3f(-0.5, 0, 0);

        ResetSystem();
    }

    void CaseDouble::OnSetupPropsUI() {
        if (_sceneChanged = ImGui::Combo("Case", &_sceneId, Scenes.data(), Scenes.size())) ResetSystem();
        if (ImGui::Button("Reset System")) ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
        }
        if (ImGui::CollapsingHeader("Physics Parameter", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Restitution", &_restitution, 0, 1);
            ImGui::SliderFloat("Box 0 mass", &_initialBox[0].Mass, 0.1, 10);
            ImGui::SliderFloat("Box 1 mass", &_initialBox[1].Mass, 0.1, 10);
        }
        if (ImGui::CollapsingHeader("Box Parameter")) {
            ImGui::SliderFloat("Box 0 dim_x", &_initialBox[0].Dim[0], 0.5, 4);
            ImGui::SliderFloat("Box 0 dim_y", &_initialBox[0].Dim[1], 0.5, 4);
            ImGui::SliderFloat("Box 0 dim_z", &_initialBox[0].Dim[2], 0.5, 4);

            ImGui::SliderFloat("Box 1 dim_x", &_initialBox[1].Dim[0], 0.5, 4);
            ImGui::SliderFloat("Box 1 dim_y", &_initialBox[1].Dim[1], 0.5, 4);
            ImGui::SliderFloat("Box 1 dim_z", &_initialBox[1].Dim[2], 0.5, 4);

            ImGui::InputFloat("Box 0 pos_x", &_initialBox[0].Center[0]);
            ImGui::InputFloat("Box 0 pos_y", &_initialBox[0].Center[1]);
            ImGui::InputFloat("Box 0 pos_z", &_initialBox[0].Center[2]);

            ImGui::InputFloat("Box 1 pos_x", &_initialBox[1].Center[0]);
            ImGui::InputFloat("Box 1 pos_y", &_initialBox[1].Center[1]);
            ImGui::InputFloat("Box 1 pos_z", &_initialBox[1].Center[2]);

        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseDouble::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) AdvanceDoubleRB(Engine::GetDeltaTime());

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
        VertsPosition.resize(8 * 2);
        for (int i = 0; i < 2; ++i) {
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

    void CaseDouble::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseDouble::AdvanceDoubleRB(float dt) {
        for (int i = 0; i < 2; ++i) {
            _box[i].Center += dt * _box[i].Velocity;

            Eigen::Quaternionf _angularVelocityQuaternion(0, _box[i].AngularVelocity[0] * dt * 0.5f, _box[i].AngularVelocity[1] * dt * 0.5f, _box[i].AngularVelocity[2] * dt * 0.5f);

            _box[i].Orientation.coeffs() += (_angularVelocityQuaternion * _box[i].Orientation).coeffs();
            _box[i].Orientation.normalize();
        }
        DetectCollision();
        SolveCollision();
    }

    void CaseDouble::DetectCollision() {
        RigidBody::DetectCollision(_box[0], _box[1], _collisionResult);
    }

    void CaseDouble::SolveCollision() {
        RigidBody::SolveCollision(_box[0], _box[1], _collisionResult, _restitution);
    }

    void CaseDouble::ResetSystem() {
        if (_sceneChanged) {
            _initialBox[0].Center = initialBoxCenters[_sceneId].first;
            _initialBox[1].Center = initialBoxCenters[_sceneId].second;

            _initialBox[0].Dim = initialBoxDims[_sceneId].first;
            _initialBox[1].Dim = initialBoxDims[_sceneId].second;

            _initialBox[0].Orientation = initialBoxOrientations[_sceneId].first;
            _initialBox[1].Orientation = initialBoxOrientations[_sceneId].second;
        }

        _box[0] = _initialBox[0];
        _box[1] = _initialBox[1];
    }
} // namespace VCX::Labs::RigidBody
