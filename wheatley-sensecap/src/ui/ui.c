#include "ui.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"

#define DISPLAY_SIZE 412

// LVGL objects
static lv_group_t *g;
static lv_obj_t *screen;

// LVGL styles
static lv_style_t eye_style;

// Assume that the images have been converted to C arrays and included
extern const lv_img_dsc_t wheatley;
extern const lv_img_dsc_t honeycomb;
extern const lv_img_dsc_t eyelid;

// Image objects
static lv_obj_t *wheatley_img;
static lv_obj_t *honeycomb_img;
static lv_obj_t *upper_eyelid_img;
static lv_obj_t *lower_eyelid_img;

// Animation variables
static lv_anim_t speaking_anim;
static lv_anim_t listening_anim;
static lv_anim_t off_anim;
static lv_anim_t on_anim;

// Time variables
static int current_distance = 0;
static int current_opacity = 0;
static enum State current_state = ON;

int map_int(int value, int in_min, int in_max, int out_min, int out_max)
{
    if (value < in_min)
        value = in_min;
    if (value > in_max)
        value = in_max;
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

void set_eyelid_distance(int distance)
{
    if (distance < 0)
    {
        distance = 0;
    }
    else if (distance > DISPLAY_SIZE)
    {
        distance = DISPLAY_SIZE;
    }

    current_distance = distance;

    int distance_from_center = distance / 2;

    lv_obj_set_y(upper_eyelid_img, (-DISPLAY_SIZE / 4) - distance_from_center);
    lv_obj_set_y(lower_eyelid_img, (DISPLAY_SIZE / 4) + distance_from_center);
}

void set_eye_opacity(int opacity)
{
    if (opacity < 0 || opacity > 255)
    {
        opacity = 255;
    }

    lv_obj_set_style_bg_img_opa(wheatley_img, opacity, 0);
    lv_obj_set_style_img_opa(wheatley_img, opacity, 0);
}

void set_eye_position(int x, int y)
{
    lv_obj_align(wheatley_img, LV_ALIGN_CENTER, x, y);
}

void wheatly_eye_init()
{
    // Create the first image object (wheatley)
    wheatley_img = lv_img_create(lv_scr_act());
    lv_img_set_src(wheatley_img, &wheatley);
    lv_obj_align(wheatley_img, LV_ALIGN_CENTER, 0, 0);

    // Eye style init
    lv_style_init(&eye_style);
    lv_obj_add_style(wheatley_img, &eye_style, 0);

    // Create the second image object (honeycomb)
    honeycomb_img = lv_img_create(lv_scr_act());
    lv_img_set_src(honeycomb_img, &honeycomb);
    lv_obj_align(honeycomb_img, LV_ALIGN_CENTER, 0, 0);

    // Create the third image object (upper eyelid)
    upper_eyelid_img = lv_img_create(lv_scr_act());
    lv_img_set_src(upper_eyelid_img, &eyelid);

    // Create the third image object (upper eyelid)
    lower_eyelid_img = lv_img_create(lv_scr_act());
    lv_img_set_src(lower_eyelid_img, &eyelid);
}

void eyelid_dist_animation_func(void *var, int value)
{
    // Set the eye position
    set_eyelid_distance(value);
}

void eye_opa_animation_func(void *var, int value)
{
    // Set the eye position
    set_eye_opacity(value);
}

void dist_and_opa_animation_func(void *var, int value)
{
    // Value goes from 0 to 412
    int opacity = map_int(value, 0, 412, 0, 255);
    // Set the eye position
    set_eye_opacity(value);
    set_eyelid_distance(value);
}

void speaking_animation_init()
{
    // Listening animation
    lv_anim_set_exec_cb(&speaking_anim, (lv_anim_exec_xcb_t)eye_opa_animation_func);
    lv_anim_set_var(&speaking_anim, wheatley_img);
    lv_anim_set_time(&speaking_anim, 1000);
    lv_anim_set_values(&speaking_anim, 0, 255);
    lv_anim_set_repeat_count(&speaking_anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_playback_time(&speaking_anim, 1000);
}

void wheatly_animation_init()
{
    // Initialize animations
    lv_anim_init(&speaking_anim);
    lv_anim_init(&listening_anim);
    lv_anim_init(&off_anim);
    lv_anim_init(&on_anim);

    speaking_animation_init();

    // Speaking animation
    lv_anim_set_exec_cb(&listening_anim, (lv_anim_exec_xcb_t)eyelid_dist_animation_func);
    lv_anim_set_var(&listening_anim, NULL);
    lv_anim_set_time(&listening_anim, 2000);
    lv_anim_set_values(&listening_anim, current_distance, 206); // go from wherever to 206
    lv_anim_set_path_cb(&listening_anim, lv_anim_path_linear);

    // Off animation
    lv_anim_set_exec_cb(&off_anim, (lv_anim_exec_xcb_t)dist_and_opa_animation_func);
    lv_anim_set_var(&off_anim, NULL);
    lv_anim_set_time(&off_anim, 500);
    lv_anim_set_values(&off_anim, current_distance, 0);
    lv_anim_set_path_cb(&off_anim, lv_anim_path_ease_in);

    // On animation
    lv_anim_set_exec_cb(&on_anim, (lv_anim_exec_xcb_t)dist_and_opa_animation_func);
    lv_anim_set_var(&on_anim, NULL);
    lv_anim_set_time(&on_anim, 500);
    lv_anim_set_values(&on_anim, 0, 412);
    lv_anim_set_path_cb(&on_anim, lv_anim_path_ease_out);
}

void state_handler(enum State next_state)
{
    ESP_LOGI("Animation", "State: [%d] change to [%d]", current_state, next_state);

    if (current_state == SPEAKING && next_state != SPEAKING)
    {
        // pause the listening animation
        lv_anim_del(wheatley_img, (lv_anim_exec_xcb_t)eye_opa_animation_func);
    }

    current_state = next_state;

    if (next_state == OFF)
    {
        lv_anim_set_values(&off_anim, current_distance, 0); // go from wherever to 0
        lv_anim_start(&off_anim);
        return;
    }

    if (next_state == ON)
    {
        lv_anim_set_values(&on_anim, current_distance, 412); // go from wherever to 412
        lv_anim_start(&on_anim);
        return;
    }

    if (next_state == SPEAKING)
    {
        lv_anim_start(&speaking_anim);
        return;
    }

    if (next_state == LISTENING)
    {
        lv_anim_set_values(&listening_anim, current_distance, 206); // go from wherever to 206
        lv_anim_start(&listening_anim);
        return;
    }
}

void ui_speaking(void)
{
    state_handler(SPEAKING);
}

void ui_listening(void)
{
    state_handler(LISTENING);
}

void ui_off(void)
{
    state_handler(OFF);
}

void ui_on(void)
{
    state_handler(ON);
}

// UI elements
void ui_init(void)
{
    lvgl_port_lock(0);

    // Create a screen
    screen = lv_obj_create(NULL);
    lv_obj_set_size(screen, 412, 412);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);

    // Fill the screen Wheatley's blue
    lv_obj_t *scr = lv_scr_act(); // Get current screen
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0032a8), 0);

    wheatly_eye_init();
    // wheatly_animation_init();

    set_eye_opacity(256);
    set_eyelid_distance(206);
    set_eye_position(0, 0);
    lvgl_port_unlock();
}
