#include "Engine/app.h"
#include "Labs/3-FEM/CaseCloth.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::FEM {
    static constexpr auto Models = std::array<char const *, 2> {
        "StVK",
        "Neo-Hookean"
    };

    CaseCloth::CaseCloth():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _triItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        _meshItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) { 
        _triSystem.setupScene(_res);

        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 0 };
        std::vector<std::uint32_t> mesh_index_(line_index.size() * _triSystem.Tris.size());
        std::vector<std::uint32_t> line_index_;
        for (int i = 0; i < _triSystem.Tris.size(); i++) {
            glm::ivec3 tri = _triSystem.Tris[i];
            for (int j = 0; j < line_index.size(); j++) {
                mesh_index_[j + i * line_index.size()] = tri[line_index[j]];
            }
            for (int j = 0; j < 3; j++) {
                for (int k = j + 1; k < 4; k++) {
                    glm::ivec2 coord1 = _triSystem.GetCoord(tri[j]);
                    glm::ivec2 coord2 = _triSystem.GetCoord(tri[k]);
                    if (_triSystem.IsOnEdge(coord1, coord2)) {
                        line_index_.push_back(tri[j]);
                        line_index_.push_back(tri[k]);
                    }
                }
            }
        }
        _meshItem.UpdateElementBuffer(mesh_index_);
        _lineItem.UpdateElementBuffer(line_index_);

        const std::vector<std::uint32_t> tri_index = { 0, 1, 2 };
        std::vector<std::uint32_t> tri_index_(tri_index.size() * _triSystem.Tris.size());
        for (int i = 0; i < _triSystem.Tris.size(); i++) {
            glm::ivec3 tri = _triSystem.Tris[i];
            for (int j = 0; j < tri_index.size(); j++) {
                tri_index_[j + i * tri_index.size()] = tri[tri_index[j]];
            }
        }
        _triItem.UpdateElementBuffer(tri_index_);

        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseCloth::OnSetupPropsUI() {
        if (ImGui::Button("Reset System")) ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_triColor));
        }
        ImGui::Combo("Model", &_triSystem.modelId, Models.data(), Models.size());
        ImGui::Checkbox("Show Mesh", &_showMesh);
        ImGui::SliderFloat("Young", &_triSystem.young, 0.0f, 100.0f);
        ImGui::SliderFloat("Poison", &_triSystem.poison, -1.0f, 0.5f);
        ImGui::SliderFloat("Gravity", &_triSystem.gravity, 0.0f, 10.0f);
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseCloth::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        std::pair<Eigen::Vector3f, int> force = _forceManager.getForce(_triSystem.Positions);

        if (! _stopped) {
            OnProcessMouseControl(_forceManager.getForce(_triSystem.Positions), Engine::GetDeltaTime());
            _triSystem.AdvanceTetSystem(0.001);
        }

        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        if (_showMesh) {
            glDisable(GL_DEPTH_TEST);
        } else {
            glEnable(GL_DEPTH_TEST);
        }
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        auto span_bytes = Engine::make_span_bytes<Eigen::Vector3f>(_triSystem.Positions);

        _program.GetUniforms().SetByName("u_Color", _triColor);
        _triItem.UpdateVertexBuffer("position", span_bytes);
        _triItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        if (_showMesh) {
            _meshItem.UpdateVertexBuffer("position", span_bytes);
            _meshItem.Draw({ _program.Use() });
        } else {
            _lineItem.UpdateVertexBuffer("position", span_bytes);
            _lineItem.Draw({ _program.Use() });
        }

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

    void CaseCloth::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        _forceManager.ProcessInput(_camera, pos);
    }

    void CaseCloth::OnProcessMouseControl(std::pair<Eigen::Vector3f, int> force, float dt) {
        Eigen::Vector3f forceDelta = force.first;
        int idx = force.second;
        float forceScale = 200.0f;
        _triSystem.Forces[idx] += forceDelta * forceScale;
    }

    void CaseCloth::ResetSystem() {
        _triSystem.Positions = _triSystem.Positions_;
        _triSystem.Velocities = _triSystem.Velocities_;
    }
} // namespace VCX::Labs::FEM
