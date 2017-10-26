#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "Cinder-VNM/include/OscHelper.h"
#include "Cinder-VNM/include/TuioHelper.h"
#include "Cinder-OpenCV3/include/CinderOpenCV.h"
#include "rplidar.h"

#include "AssetManager.h"
#include "MiniConfig.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace rp::standalone::rplidar;

struct RPlidarHelper
{
    bool setup(const string& serialPort)
    {
        return false;
    }

    RPlidarDriver* driver = nullptr;
};

class MiniAreaScanApp : public App
{
  public:
    void setup() override
    {
        log::makeLogger<log::LoggerFile>();
        createConfigUI({200, 200});
    
        getWindow()->getSignalKeyUp().connect([&](KeyEvent& event) {
            if (event.getCode() == KeyEvent::KEY_ESCAPE) quit();
        });
        
        getWindow()->getSignalDraw().connect([&] {
            gl::clear();
        });
    }
    
private:
};

CINDER_APP( MiniAreaScanApp, RendererGl, [](App::Settings* settings) {
    readConfig();
    settings->setWindowSize(APP_WIDTH, APP_HEIGHT);
    settings->setMultiTouchEnabled(false);
} )
