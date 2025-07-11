#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/4-PBF/CaseFluid.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::PBF {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFluid  _casefluid;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _casefluid };

    public:
        App();

        void OnFrame() override;
    };
}
