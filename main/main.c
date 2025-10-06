// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// C stdio
#include <stdio.h>

// Pico SDK
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// Projeto
#include "pins.h"

// -----------------------------------------------------------------------------
// Contrato (resumo):
// - pin_callback: mede largura do pulso no ECHO e envia duração (us) para xQueueTime
// - trigger_task: gera pulso de 10us no TRIG e libera xSemaphoreTrigger
// - echo_task: converte duração (us) em distância (cm) e envia para xQueueDistance
// - oled_task: ao receber xSemaphoreTrigger, aguarda a distância com timeout; exibe
//              valores e seta LED: verde (<=1m), amarelo (>1m), vermelho (falha)
// -----------------------------------------------------------------------------

// Recursos do sistema
static QueueHandle_t xQueueTime;      // fila de durações do pulso (us)
static QueueHandle_t xQueueDistance;  // fila de distâncias (cm)
static SemaphoreHandle_t xSemaphoreTrigger; // sinaliza leitura disparada

// Armazenamento do tempo de subida do echo (acessado na IRQ)
static volatile uint64_t s_echo_rise_time_us = 0;

// Utilidades LED RGB (ativo alto)
static inline void led_rgb_init(void) {
    gpio_init(LED_PIN_R); gpio_set_dir(LED_PIN_R, GPIO_OUT); gpio_put(LED_PIN_R, 0);
    gpio_init(LED_PIN_G); gpio_set_dir(LED_PIN_G, GPIO_OUT); gpio_put(LED_PIN_G, 0);
    gpio_init(LED_PIN_B); gpio_set_dir(LED_PIN_B, GPIO_OUT); gpio_put(LED_PIN_B, 0);
}

static inline void led_set(bool r, bool g, bool b) {
    gpio_put(LED_PIN_R, r);
    gpio_put(LED_PIN_G, g);
    gpio_put(LED_PIN_B, b);
}

static inline void led_green(void)  { led_set(false, true,  false); }
static inline void led_yellow(void) { led_set(true,  true,  false); }
static inline void led_red(void)    { led_set(true,  false, false); }
static inline void led_off(void)    { led_set(false, false, false); }

// Callback de IRQ do pino do ECHO
static void pin_callback(uint gpio, uint32_t events) {
    if (gpio != (uint)ECHO_PIN) return;

    if (events & GPIO_IRQ_EDGE_RISE) {
        // Guarda o tempo de subida
        s_echo_rise_time_us = time_us_64();
    }
    if (events & GPIO_IRQ_EDGE_FALL) {
        const uint64_t now_us = time_us_64();
        if (s_echo_rise_time_us != 0 && now_us >= s_echo_rise_time_us) {
            uint32_t duration_us = (uint32_t)(now_us - s_echo_rise_time_us);

            BaseType_t xHigherPrioWoken = pdFALSE;
            if (xQueueTime) {
                (void)xQueueSendFromISR(xQueueTime, &duration_us, &xHigherPrioWoken);
            }
            // Libera mudança de contexto se necessário
            portYIELD_FROM_ISR(xHigherPrioWoken);
        }
        // Reseta para evitar usar valor velho
        s_echo_rise_time_us = 0;
    }
}

// Task: gera pulso no TRIG a cada período
static void trigger_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(150); // 150ms entre leituras
    (void)arg;
    for(;;) {
        // Pulso de 10us (HS-SR04)
        gpio_put(TRIG_PIN, 0);
        sleep_us(2);
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);

        // Notifica o OLED que nova leitura foi disparada
        xSemaphoreGive(xSemaphoreTrigger);

        vTaskDelay(period);
    }
}

// Converte duração do pulso do ECHO (us) para distância (cm)
static float us_to_cm(uint32_t duration_us) {
    // Aprox. padrão para HC-SR04: distancia (cm) = duracao_us / 58.0f
    return (float)duration_us / 58.0f;
}

// Task: recebe duração do pulso e envia a distância
static void echo_task(void *arg) {
    (void)arg;
    for(;;) {
        uint32_t duration_us;
        if (xQueueReceive(xQueueTime, &duration_us, portMAX_DELAY) == pdTRUE) {
            float dist_cm = us_to_cm(duration_us);
            // Envia a distância (cm) para a fila do display
            (void)xQueueOverwrite(xQueueDistance, &dist_cm);
        }
    }
}

// Ganchos simples para atualizar o display (fallback: stdout)
// -----------------------------------------------------------------------------
// SSD1306 (128x32) via I2C - implementação mínima para texto e barra
// -----------------------------------------------------------------------------
typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t width;
    uint8_t height;
} ssd1306_t;

static ssd1306_t g_oled;

static void ssd1306_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(g_oled.i2c, g_oled.addr, buf, 2, false);
}

static void ssd1306_init(ssd1306_t *dev, i2c_inst_t *i2c, uint8_t addr, uint8_t w, uint8_t h) {
    g_oled.i2c = i2c; g_oled.addr = addr; g_oled.width = w; g_oled.height = h;
    sleep_ms(50);
    ssd1306_cmd(0xAE); // display off
    ssd1306_cmd(0x20); ssd1306_cmd(0x00); // horizontal addressing
    ssd1306_cmd(0xB0); // page 0
    ssd1306_cmd(0xC8); // COM scan dec
    ssd1306_cmd(0x00); // low col
    ssd1306_cmd(0x10); // high col
    ssd1306_cmd(0x40); // start line
    ssd1306_cmd(0x81); ssd1306_cmd(0x7F); // contrast
    ssd1306_cmd(0xA1); // segment remap
    ssd1306_cmd(0xA6); // normal display
    ssd1306_cmd(0xA8); ssd1306_cmd(h - 1); // mux ratio
    ssd1306_cmd(0xA4); // display follows RAM
    ssd1306_cmd(0xD3); ssd1306_cmd(0x00); // display offset
    ssd1306_cmd(0xD5); ssd1306_cmd(0x80); // clock div
    ssd1306_cmd(0xD9); ssd1306_cmd(0xF1); // pre-charge
    ssd1306_cmd(0xDA); ssd1306_cmd(0x02); // compins (para 128x32)
    ssd1306_cmd(0xDB); ssd1306_cmd(0x40); // vcom detect
    ssd1306_cmd(0x8D); ssd1306_cmd(0x14); // charge pump
    ssd1306_cmd(0xAF); // display ON
}

static void ssd1306_set_cursor(uint8_t col, uint8_t page) {
    ssd1306_cmd(0x21); ssd1306_cmd(col); ssd1306_cmd(g_oled.width - 1);
    ssd1306_cmd(0x22); ssd1306_cmd(page); ssd1306_cmd((g_oled.height/8) - 1);
}

static void ssd1306_clear(void) {
    ssd1306_set_cursor(0, 0);
    uint8_t zero = 0x00;
    uint8_t prefix = 0x40;
    for (uint16_t i = 0; i < (uint16_t)g_oled.width * (g_oled.height/8); ++i) {
        uint8_t buf[2] = {prefix, zero};
        i2c_write_blocking(g_oled.i2c, g_oled.addr, buf, 2, false);
    }
}

// Fonte 5x7 compacta (subset ASCII 32..90 só o necessário)
static const uint8_t font5x7[][5] = {
    // ' ' .. '/' (espaço até '/')
    {0,0,0,0,0},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    // '0' .. '9'
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    // ':' .. '@'
    {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    // 'A' .. 'Z'
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
};

static void ssd1306_draw_char(uint8_t x, uint8_t page, char c) {
    if (c < 32) c = 32;
    if (c > 'Z') c = 'Z';
    ssd1306_set_cursor(x, page);
    uint8_t data[6];
    data[0] = 0x40; // data prefix
    const uint8_t *glyph = font5x7[c - 32];
    for (int i = 0; i < 5; ++i) data[1+i] = glyph[i];
    i2c_write_blocking(g_oled.i2c, g_oled.addr, data, 6, false);
}

static void ssd1306_print(uint8_t col, uint8_t row, const char *s) {
    // row é em páginas (8px), col em pixels
    uint8_t x = col;
    while (*s && x < g_oled.width - 6) {
        ssd1306_draw_char(x, row, *s++);
        x += 6; // 5px font + 1px espaçamento
    }
}

static void ssd1306_draw_bar(uint8_t y_page, float value_cm) {
    // Barra horizontal de 0..200cm mapeada para 0..128px
    float max_cm = 200.0f;
    if (value_cm < 0) value_cm = 0;
    if (value_cm > max_cm) value_cm = max_cm;
    uint8_t w = (uint8_t)((value_cm / max_cm) * (float)g_oled.width);

    // Desenha duas páginas (16px de altura) como barra cheia
    for (uint8_t page = y_page; page < y_page + 2 && page < (g_oled.height/8); ++page) {
        ssd1306_set_cursor(0, page);
        for (uint8_t x = 0; x < g_oled.width; ++x) {
            uint8_t pix = (x < w) ? 0xFF : 0x00;
            uint8_t buf[2] = {0x40, pix};
            i2c_write_blocking(g_oled.i2c, g_oled.addr, buf, 2, false);
        }
    }
}

static void display_init(void) {
    // I2C0 nos pinos definidos no pins.h
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    ssd1306_init(&g_oled, i2c0, 0x3C, 128, 32);
    ssd1306_clear();
}

static void display_clear(void) {
    ssd1306_clear();
}

static void display_show_reading(float dist_cm) {
    char line[24];
    snprintf(line, sizeof(line), "Dist: %.1f cm", dist_cm);
    ssd1306_clear();
    ssd1306_print(0, 0, line);
}

static void display_show_error(void) {
    ssd1306_clear();
    ssd1306_print(0, 0, "Sensor: FALHA");
}

static void display_show_bar(float dist_cm) {
    ssd1306_draw_bar(2, dist_cm); // ocupa páginas 2 e 3 (metade inferior)
}

// Task: espera trigger e exibe resultado com timeout
static void oled_task(void *arg) {
    (void)arg;
    const TickType_t rx_timeout = pdMS_TO_TICKS(100); // pensar no melhor valor!
    for(;;) {
        // Aguarda um trigger de leitura
        if (xSemaphoreTake(xSemaphoreTrigger, portMAX_DELAY) == pdTRUE) {
            float dist_cm;
            if (xQueueReceive(xQueueDistance, &dist_cm, rx_timeout) == pdTRUE) {
                // Sucesso
                display_show_reading(dist_cm);
                display_show_bar(dist_cm);
                if (dist_cm <= 100.0f) {
                    led_green();
                } else {
                    led_yellow();
                }
            } else {
                // Falha: dado não chegou no tempo
                display_show_error();
                led_red();
            }
        }
    }
}

int main(void) {
    stdio_init_all();

    // OLED
    display_init();

    // Inicializa pinos do sensor
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN); // ajuste conforme hardware (algumas placas preferem pull-down)

    // LED RGB
    led_rgb_init();
    led_off();

    // IRQ no pino de ECHO (subida e descida)
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &pin_callback);

    // Cria recursos FreeRTOS
    xQueueTime = xQueueCreate(4, sizeof(uint32_t));
    // xQueueDistance é melhor como fila de 1 elemento (overwrite) para manter apenas o mais recente
    xQueueDistance = xQueueCreate(1, sizeof(float));
    xSemaphoreTrigger = xSemaphoreCreateBinary();

    // Cria tasks
    xTaskCreate(trigger_task, "trigger_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(echo_task,    "echo_task",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(oled_task,    "oled_task",    configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Inicia escalonador
    vTaskStartScheduler();

    while (true) {}
}
