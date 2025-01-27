#include "lvgl.h"

enum State
{
    LISTENING = 0,
    SPEAKING = 1,
    OFF = 2,
    ON = 3
};

void ui_init(void);
void ui_speaking(void);
void ui_listening(void);
void ui_off(void);
void ui_on(void);