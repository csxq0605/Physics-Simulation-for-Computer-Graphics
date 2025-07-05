#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSingle.h"
#include "Labs/Common/ImGuiHelper.h"

static glm::vec3 eigen2glm(Eigen::Vector3f const & eigen_v) {
    return glm::vec3(eigen_v[0], eigen_v[1], eigen_v[2]);
}

static Eigen::Vector3f glm2eigen(glm::vec3 const & glm_v) {
    return Eigen::Vector3f(glm_v[0], glm_v[1], glm_v[2]);
}

namespace VCX::Labs::RigidBody {

    CaseSingle::CaseSingle():
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
        _lineItem.UpdateElementBuffer(line_index);

        // const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseSingle::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
            ImGui::SliderFloat("x", &_box.Dim[0], 0.5, 4);
            ImGui::SliderFloat("y", &_box.Dim[1], 0.5, 4);
            ImGui::SliderFloat("z", &_box.Dim[2], 0.5, 4);

            ImGui::InputFloat("pos_x", &_box.Center[0]);
            ImGui::InputFloat("pos_y", &_box.Center[1]);
            ImGui::InputFloat("pos_z", &_box.Center[2]);

            ImGui::SliderFloat("vel_x", &_box.Velocity[0], -4, 4);
            ImGui::SliderFloat("vel_y", &_box.Velocity[1], -4, 4);
            ImGui::SliderFloat("vel_z", &_box.Velocity[2], -4, 4);

            ImGui::SliderFloat("ang_vel_x", &_box.AngularVelocity[0], -4, 4);
            ImGui::SliderFloat("ang_vel_y", &_box.AngularVelocity[1], -4, 4);
            ImGui::SliderFloat("ang_vel_z", &_box.AngularVelocity[2], -4, 4);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseSingle::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_forceManager.getForce(_box), Engine::GetDeltaTime());

        AdvanceSingleRB(Engine::GetDeltaTime());

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
        Eigen::Vector3f        new_x = _box.Orientation * Eigen::Vector3f(_box.Dim[0] / 2, 0.f, 0.f);
        Eigen::Vector3f        new_y = _box.Orientation * Eigen::Vector3f(0.f, _box.Dim[1] / 2, 0.f);
        Eigen::Vector3f        new_z = _box.Orientation * Eigen::Vector3f(0.f, 0.f, _box.Dim[2] / 2);
        VertsPosition.resize(8);
        VertsPosition[0] = _box.Center - new_x + new_y + new_z;
        VertsPosition[1] = _box.Center + new_x + new_y + new_z;
        VertsPosition[2] = _box.Center + new_x + new_y - new_z;
        VertsPosition[3] = _box.Center - new_x + new_y - new_z;
        VertsPosition[4] = _box.Center - new_x - new_y + new_z;
        VertsPosition[5] = _box.Center + new_x - new_y + new_z;
        VertsPosition[6] = _box.Center + new_x - new_y - new_z;
        VertsPosition[7] = _box.Center - new_x - new_y - new_z;

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

    void CaseSingle::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        _forceManager.ProcessInput(_camera, pos);
    }

    void CaseSingle::OnProcessMouseControl(std::pair<glm::vec3,glm::vec3> force, float dt) {
        Eigen::Vector3f forceDelta = glm2eigen(force.first);
        Eigen::Vector3f forcePoint = glm2eigen(force.second);
        Eigen::Vector3f torque = (forcePoint - _box.Center).cross(forceDelta);
        _box.AngularVelocity += _box.GetInertiaMatrix().inverse() * torque;
        _box.Velocity += forceDelta;
    }

    void CaseSingle::AdvanceSingleRB(float dt) {
        _box.Center += dt * _box.Velocity;

        Eigen::Quaternionf _angularVelocityQuaternion(0, _box.AngularVelocity[0] * dt * 0.5f, _box.AngularVelocity[1] * dt * 0.5f, _box.AngularVelocity[2] * dt * 0.5f);

        _box.Orientation.coeffs() += (_angularVelocityQuaternion * _box.Orientation).coeffs();
        _box.Orientation.normalize();
    }

} // namespace VCX::Labs::RigidBody
