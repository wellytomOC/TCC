#include "uiManager.h"
#include "screen_home.h"
#include "screen_loading.h"
#include "screen_results.h"
#include "screen_parameters.h"

static Screen_t * screens[SCREEN_MAX] = {
    &Screen_Home,
    &Screen_Loading,
    &Screen_Results,
    &Screen_Parameters,
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


// --------------------------------------------------
// Helper functions
// --------------------------------------------------
bool ui_manager_is_screen(ScreenID_t id)
{
    return (current_screen == screens[id]);
}

ScreenID_t ui_manager_get_current_id(void)
{
    for (int i = 0; i < SCREEN_MAX; i++) {
        if (current_screen == screens[i]) {
            return (ScreenID_t)i;
        }
    }
    return SCREEN_MAX;  // Return a “none” or invalid value
}