#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/3-FEM/TetSystem.h"
#include "Labs/Common/ExternalForceManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::FEM {

    class CaseDeform : public Common::ICase {
    public:
        CaseDeform();

        virtual std::string_view const GetName() override { return "Deformable Solid"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(std::pair<Eigen::Vector3f, int> force, float dt);

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-8, 8, 8) };
        Common::OrbitCameraManager          _cameraManager;
        Common::ExternalForceManager        _forceManager;
        Engine::GL::UniqueIndexedRenderItem _tetItem;  // render the tet
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line
        Engine::GL::UniqueIndexedRenderItem _meshItem; // render mesh
        glm::vec3                           _tetColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        bool                                _stopped { true };
        int                                 _res { 2 };
        bool                                _showMesh { false };

        TetSystem _tetSystem;

        void ResetSystem();
    };
} // namespace VCX::Labs::FEM
