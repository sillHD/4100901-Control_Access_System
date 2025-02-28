#include "button.h"
#include "main.h"

uint32_t b1_tick = 0; // para detectar antirebote y doble presion basado en el ultimo evento

int detect_button_press(void) {
    uint8_t button_pressed = 0;
    if (HAL_GetTick() - b1_tick < 50) {
        // Ignore bounces less than 50ms
    } else if (HAL_GetTick() - b1_tick > 500) {
        button_pressed = 1; // single press
    } else {
        button_pressed = 2; // double press
    }
    b1_tick = HAL_GetTick();
    return button_pressed;
}