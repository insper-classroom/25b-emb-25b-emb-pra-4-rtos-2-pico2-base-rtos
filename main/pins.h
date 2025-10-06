#ifndef PINS_H
#define PINS_H

const int BTN_PIN_R = 4;
const int BTN_PIN_G = 5;
const int BTN_PIN_B = 6;
const int LED_PIN_R = 7;
const int LED_PIN_G = 8;
const int LED_PIN_B = 9;

const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

// OLED (Pico Dock): I2C0 em GPIO2 (SDA) e GPIO3 (SCL)
const int OLED_SDA_PIN = 2;
const int OLED_SCL_PIN = 3;

#endif // PINS_H