#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/3-FEM/CaseDeform.h"
#include "Labs/3-FEM/CaseCloth.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseDeform _casedeform;
        CaseCloth  _casecloth;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _casedeform, _casecloth };

    public:
        App();

        void OnFrame() override;
    };
}
