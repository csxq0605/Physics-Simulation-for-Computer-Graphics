#pragma once

#include <fcl/narrowphase/collision.h>

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/1-RigidBody/Box.h"
#include "Labs/Common/ExternalForceManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::RigidBody {
    class CaseComplex : public Common::ICase {
    public:
        CaseComplex();

        virtual std::string_view const GetName() override { return "Complex Scene with Multiple Rigid Bodies"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void OnProcessMouseControl(std::tuple<glm::vec3,glm::vec3, int> force, float dt);

        void AdvanceComplexRB(float dt);

    private:
        Engine::GL::UniqueProgram                _program;
        Engine::GL::UniqueRenderFrame            _frame;
        Engine::Camera                           _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager               _cameraManager;
        Common::ExternalForceManager             _forceManager;
        Engine::GL::UniqueIndexedRenderItem      _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem      _lineItem; // render line on box
        glm::vec3                                _boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        int                                      _boxNum { 5 };
        bool                                     _stopped { false };
        std::vector<fcl::CollisionResult<float>> _collisionResult;
        float                                    _restitution { 0.9f };
        float                                    _gravity { 0.5f };

        std::vector<Box> _box;// _box[0] is ground box

        void DetectCollision();
        void SolveCollision();
        void ResetSystem();
    };
} // namespace VCX::Labs::RigidBody
