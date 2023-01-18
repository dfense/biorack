#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_task.h"
// #include "driver/rmt_tx.h"
#include "led_strip.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM 27

#define LED_NUMBERS 2
static led_strip_handle_t led_strip;

void init_led_strip(){

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .max_leds = LED_NUMBERS, // at least one LED on board
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);

}

void set_led(int location, int r, int b, int g){
    led_strip_set_pixel(led_strip, location, r , g, b);
    led_strip_refresh(led_strip);
}