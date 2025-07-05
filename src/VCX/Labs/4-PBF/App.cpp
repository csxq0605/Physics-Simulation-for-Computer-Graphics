#include "Assets/bundled.h"
#include "Labs/4-PBF/App.h"

namespace VCX::Labs::PBF {
    App::App():
        _ui(Labs::Common::UIOptions {}),
        _casefluid({ Assets::ExampleScene::Fluid }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
