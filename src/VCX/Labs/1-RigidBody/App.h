#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSingle.h"
#include "Labs/1-RigidBody/CaseDouble.h"
#include "Labs/1-RigidBody/CaseComplex.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseSingle  _CaseSingle;
        CaseDouble  _CaseDouble;
        CaseComplex _CaseComplex;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _CaseSingle, _CaseDouble, _CaseComplex };

    public:
        App();

        void OnFrame() override;
    };
}
