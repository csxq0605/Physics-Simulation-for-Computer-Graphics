#include <spdlog/spdlog.h>
#include "Engine/app.h"
#include "Labs/4-PBF/CaseFluid.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>

namespace VCX::Labs::PBF {
    const std::vector<std::uint32_t> line_index = 
        { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7,
        8, 9, 9, 10, 10, 11, 11, 8 }; // line index

    static constexpr auto Scenes = std::array<char const *, 2> {
        "Free Fall",
        "Wave",
    };

    CaseFluid::CaseFluid(std::initializer_list<Assets::ExampleScene> && scenes) :
        _scenes(scenes),
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/fluid.vert"),
                Engine::GL::SharedShader("assets/shaders/fluid.frag") })),
        _lineprogram(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/flat.vert"),
                Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _sceneObject(1),
        _BoundaryItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0), Engine::GL::PrimitiveType::Lines){ 
        _cameraManager.AutoRotate = false;
        _program.BindUniformBlock("PassConstants", 1);
        _program.GetUniforms().SetByName("u_DiffuseMap" , 0);
        _program.GetUniforms().SetByName("u_SpecularMap", 1);
        _program.GetUniforms().SetByName("u_HeightMap"  , 2);
        _lineprogram.GetUniforms().SetByName("u_Color",  glm::vec3(1.0f));
        _BoundaryItem.UpdateElementBuffer(line_index);
        ResetSystem();
        _sphere = Engine::Model{Engine::Sphere(6, _simulation.m_particleRadius), 0};
        obstacle_sphere = Engine::Model{Engine::Sphere(36,_simulation.obstacleRadius), 0};
    }

    void CaseFluid::OnSetupPropsUI() {
        if (_sceneChanged = ImGui::Combo("Case", &_simulation.sceneId, Scenes.data(), Scenes.size()))
            ResetSystem();
        if (ImGui::Button("Reset System"))
            ResetSystem();
        ImGui::SameLine();
        if (ImGui::Button(_stopped ? "Start Simulation":"Stop Simulation"))
            _stopped = ! _stopped;
        ImGui::Spacing();
        ImGui::SliderFloat("relWaterX", &_simulation.relWater[0], 0.0f, 1.0f);
        ImGui::SliderFloat("relWaterY", &_simulation.relWater[1], 0.0f, 1.0f);
        ImGui::SliderFloat("relWaterZ", &_simulation.relWater[2], 0.0f, 1.0f);
        ImGui::SliderFloat("offsetX", &_simulation.offset[0], 0.0f, 1.0f);
        ImGui::SliderFloat("offsetY", &_simulation.offset[1], 0.0f, 1.0f);
        ImGui::SliderFloat("offsetZ", &_simulation.offset[2], 0.0f, 1.0f);
        if (_simulation.sceneId == 1) {
            ImGui::SliderFloat("slideVel", &_simulation.slideVel, 0.0f, 2.0f);
        }
    }


    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _sceneObject.ReplaceScene(GetScene(_sceneIdx));
            _cameraManager.Save(_sceneObject.Camera);
        }
        OnProcessMouseControl(_cameraManager.getMouseMove(), Engine::GetDeltaTime());
        if (! _stopped) _simulation.SimulateTimestep(1.0 / 200);

        vertex_pos = {
            0.5f * glm::vec3(-_simulation.tank[0], -_simulation.tank[1], -_simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0], -_simulation.tank[1], -_simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0], _simulation.tank[1], -_simulation.tank[2]),
            0.5f * glm::vec3(-_simulation.tank[0], _simulation.tank[1], -_simulation.tank[2]),
            0.5f * glm::vec3(-_simulation.tank[0], -_simulation.tank[1], _simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0], -_simulation.tank[1], _simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0], _simulation.tank[1], _simulation.tank[2]),
            0.5f * glm::vec3(-_simulation.tank[0], _simulation.tank[1], _simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0] * _simulation.slidePos, -_simulation.tank[1], _simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0] * _simulation.slidePos, -_simulation.tank[1], -_simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0] * _simulation.slidePos, _simulation.tank[1], -_simulation.tank[2]),
            0.5f * glm::vec3(_simulation.tank[0] * _simulation.slidePos, _simulation.tank[1], _simulation.tank[2]),
        };
        _BoundaryItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(vertex_pos));
        _frame.Resize(desiredSize);

        _cameraManager.Update(_sceneObject.Camera);
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::Projection, _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::View, _sceneObject.Camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(&VCX::Labs::Rendering::SceneObject::PassConstants::ViewPosition, _sceneObject.Camera.Eye);
        _lineprogram.GetUniforms().SetByName("u_Projection", _sceneObject.Camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _lineprogram.GetUniforms().SetByName("u_View"      , _sceneObject.Camera.GetViewMatrix());
        
        if (_uniformDirty) {
            _uniformDirty = false;
            _program.GetUniforms().SetByName("u_AmbientScale"      , _ambientScale);
            _program.GetUniforms().SetByName("u_UseBlinn"          , _useBlinn);
            _program.GetUniforms().SetByName("u_Shininess"         , _shininess);
            _program.GetUniforms().SetByName("u_UseGammaCorrection", int(_useGammaCorrection));
            _program.GetUniforms().SetByName("u_AttenuationOrder"  , _attenuationOrder);            
            _program.GetUniforms().SetByName("u_BumpMappingBlend"  , _bumpMappingPercent * .01f);            
        }
        
        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);
        glLineWidth(_BndWidth);
        _BoundaryItem.Draw({ _lineprogram.Use() });
        glLineWidth(1.f);

        Rendering::ModelObject m = Rendering::ModelObject(_sphere,_simulation.m_particlePos,_simulation.m_particleColor);
        auto const & material    = _sceneObject.Materials[0];
        m.Mesh.Draw({ material.Albedo.Use(),  material.MetaSpec.Use(), material.Height.Use(),_program.Use() },
            _sphere.Mesh.Indices.size(), 0, _simulation.m_iNumSpheres);

        if (_simulation.sceneId == 0) {
            Rendering::ModelObject obstacle = Rendering::ModelObject(obstacle_sphere, {_simulation.obstaclePos}, {glm::vec3(0.3f)});
            obstacle.Mesh.Draw({ material.Albedo.Use(),  material.MetaSpec.Use(), material.Height.Use(),_program.Use() },
                obstacle_sphere.Mesh.Indices.size(), 0, 1);
        }

        glDepthFunc(GL_LEQUAL);
        glDepthFunc(GL_LESS);
        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFluid::OnProcessInput(ImVec2 const& pos) {
        _cameraManager.ProcessInput(_sceneObject.Camera, pos);
    }

    void CaseFluid::OnProcessMouseControl(glm::vec3 mouseDelta, float deltaTime) {
        float movingScale = 0.05f;
        _simulation.obstaclePos += mouseDelta * movingScale;
    }

    void CaseFluid::ResetSystem(){
        if (_sceneChanged) {
            if (_simulation.sceneId == 0) {
                _simulation.tank = { 1.0f, 2.0f, 1.0f };
                _simulation.relWater = { 0.4f, 0.8f, 0.5f };
                _simulation.offset = { 0.5f, 1.0f, 0.7f };
            } else if (_simulation.sceneId == 1) {
                _simulation.tank = { 2.0f, 1.0f, 0.5f };
                _simulation.relWater = { 0.4f, 0.6f, 1.0f };
                _simulation.offset = { 0.0f, 0.0f, 0.5f };
            }
            _simulation.slidePos = 1.0f;
            _simulation.slideVel = 1.0f;
            _simulation.slideDir = -1;
        }
        _simulation.setupScene();
    }
}