#include "uiManager.h"
#include "screen_home.h"
#include "screen_loading.h"
// #include other screens...

static Screen_t * screens[SCREEN_MAX] = {
    &Screen_Home,
    // &Screen_Calibration,
    &Screen_Loading,
    // &Screen_Loading,
};

static Screen_t * current_screen = nullptr;

void ui_manager_set_screen(ScreenID_t id)
{
    if (current_screen && current_screen->destroy) {
        current_screen->destroy();
    }

    current_screen = screens[id];
    if (current_screen && current_screen->create) {
        current_screen->create();
    }
}

void ui_manager_update(void)
{
    if (current_screen && current_screen->update) {
        current_screen->update();
    }
}
