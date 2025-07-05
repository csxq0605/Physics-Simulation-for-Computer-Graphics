#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/3-FEM/TriSystem.h"
#include "Labs/Common/ExternalForceManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::FEM {

    class CaseCloth : public Common::ICase {
    public:
        CaseCloth();

        virtual std::string_view const GetName() override { return "Cloth"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(std::pair<Eigen::Vector3f, int> force, float dt);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        Common::ExternalForceManager        _forceManager;
        Engine::GL::UniqueIndexedRenderItem _triItem;  // render the tri
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line
        Engine::GL::UniqueIndexedRenderItem _meshItem; // render mesh
        glm::vec3                           _triColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        bool                                _stopped { true };
        int                                 _res { 16 };
        bool                                _showMesh { false };

        TriSystem _triSystem;

        void ResetSystem();
    };
} // namespace VCX::Labs::FEM
