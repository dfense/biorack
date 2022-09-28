#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"

// buttons
#define B1 GPIO_NUM_14
#define B2 GPIO_NUM_15

// contactors
#define C1 GPIO_NUM_35
#define C2 GPIO_NUM_39

// uint32_t timerTicks = 43200000;
// uint32_t timerTicks = 1800000; // 30 minutes
uint32_t timerTicks = 14400000; // 4 hrs 

QueueHandle_t interruptHandlerQueue;
int cnt = 0;

typedef struct
{
    int gpio_l1_pin;
    int line1_state;
    int gpio_l2_pin;
    int line2_state;
    int gpio_enable_pin;
    int enable_state;
    int direction; // 0=clockwise 1=counterclockwise
} motor_t;

motor_t motor1;

void init_buttons(int buttons[])
{

    gpio_config_t gpio_config1;
    gpio_config1.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config1.mode = GPIO_MODE_INPUT;
    gpio_config1.pull_up_en = 1;
    gpio_config1.pull_down_en = 0;
    gpio_config1.pin_bit_mask = ((1ULL << B1) | (1ULL << B2) | (1ULL << C1) | (1ULL << C2));
    gpio_config(&gpio_config1);

    motor1.enable_state = 255;
    motor1.gpio_l1_pin = GPIO_NUM_23;
    motor1.gpio_l2_pin = GPIO_NUM_19;
    motor1.gpio_enable_pin = GPIO_NUM_4;

    // motor2_enable_state = 255;
    // motor2.gpio_l1_pin = GPIO_NUM_32;
    // motor2.gpio_l2_pin = GPIO_NUM_33;
    // motor2.gpio_enable_pin = GPIO_NUM_18;

    gpio_config_t gpio_config2;
    gpio_config2.mode = GPIO_MODE_OUTPUT;
    gpio_config2.intr_type = GPIO_INTR_DISABLE;
    gpio_config2.pull_up_en = 0;
    gpio_config2.pull_down_en = 0;
    gpio_config2.pin_bit_mask = ((1ULL << motor1.gpio_l1_pin) | (1ULL << motor1.gpio_l2_pin) | (1ULL << motor1.gpio_enable_pin));
    gpio_config(&gpio_config2);

    // default motor enable. can be used to PWM speed
    gpio_set_level(motor1.gpio_enable_pin, 1);
}

bool verify_direction(motor_t *motor)
{

    if (motor->direction)
    {
        return true;
    }
    return true;
    // set L0 and L2 lines to begin appropriate direction
}

void motor_start(motor_t *motor, bool start)
{
    if (start)
    {
        motor->line1_state = 1;
        motor->line2_state = 0;

        printf("inside starting motor - pin %d = %d\n", motor->gpio_l1_pin, motor->line1_state);
        gpio_set_level(motor->gpio_l1_pin, motor->line1_state);
        gpio_set_level(motor->gpio_l2_pin, motor->line2_state);
    }
    else // reverse
    {
        motor->line1_state = 0;
        motor->line2_state = 1;
        gpio_set_level(motor->gpio_l1_pin, motor->line1_state);
        gpio_set_level(motor->gpio_l2_pin, motor->line2_state);
    }
}

void motor_stop(motor_t *motor)
{
    gpio_set_level(motor->gpio_l1_pin, 0);
    gpio_set_level(motor->gpio_l2_pin, 0);
}

// get in , get out fast !
static void IRAM_ATTR button_handler(void *args)
{

    int int_pin = (int)args;
    xQueueSendFromISR(interruptHandlerQueue, &int_pin, NULL);
}

// listen for button events forever
void buttonListener(void *params)
{
    int pinNumber = 0, count = 0;
    // uint64_t last39detection = 0, last36detection = 0;
    // uint64_t tmpTime = 0;
    // static int lastTime = 0;
    while (1)
    {
        if (xQueueReceive(interruptHandlerQueue, &pinNumber, portMAX_DELAY))
        {
            // software debounce for non-schmitt buttons
            switch (pinNumber)
            {
            case C1:
                if (motor1.line1_state)
                {
                    motor_stop(&motor1);
                }
                // tmpTime = esp_timer_get_time();
                // if ((tmpTime - last39detection) / 1000.f > 20.0)
                // {
                    printf("-> GPIO %d was pressed. The state is %d\n", pinNumber, gpio_get_level(pinNumber));
                    //printf("%f -> GPIO %d was pressed %d times. The state is %d\n", (float)esp_timer_get_time() / 1000.f, pinNumber, count++, gpio_get_level(pinNumber));
                // }
                // else
                // {
                //     printf("%d -> shake detected %f\n", pinNumber, (tmpTime - last39detection) / 1000.f);
                // }
                // last39detection = tmpTime;
                break;
            case C2:
                if (motor1.line2_state)
                {
                    motor_stop(&motor1);
                }
                // tmpTime = esp_timer_get_time();
                // if ((tmpTime - last36detection) / 1000.f < 20.0)
                // {
                    printf("-> GPIO %d was pressed. The state is %d\n", pinNumber, gpio_get_level(pinNumber));
                    // printf("%d -> shake detected %f\n", pinNumber, (tmpTime - last36detection) / 1000.f);
                // }
                // else
                // {
                    //printf("%f -> GPIO %d was pressed %d times. The state is %d\n", (float)esp_timer_get_time() / 1000.f, pinNumber, count++, gpio_get_level(pinNumber));
                // }
                // last36detection = tmpTime;
                break;
            case B1:
                printf("%f -> GPIO %d was pressed %d times. The state is %d\n", (float)esp_timer_get_time() / 1000.f, pinNumber, count++, gpio_get_level(pinNumber));
                if (!gpio_get_level(pinNumber))
                {
                    printf("starting motor\n");
                    if (gpio_get_level(C2))
                    {
                        printf("starting motor forward\n");
                        motor_start(&motor1, true);
                    }
                    else
                    {
                        printf("starting motor reverse\n");
                        motor_start(&motor1, false);
                    }
                    // motor_start(&motor1);
                }
                break;
            case B2:
                if (gpio_get_level(pinNumber))
                {
                    printf("stopping motor\n");
                    motor_stop(&motor1);
                }
                break;
            }
        }
    }
}

void on_reverse(TimerHandle_t xTimer)
{
    motor_start(&motor1, false);
}

void on_timer(TimerHandle_t xTimer)
{
    // fire on every (n) ticks thorugh
    ESP_LOGI("LOG", "motor started");
    motor_start(&motor1, true);
    TimerHandle_t xTimer2 = xTimerCreate("rotation_timer", pdMS_TO_TICKS(60000), false, NULL, on_reverse);
    xTimerStart(xTimer2, 0);
}

void app_main(void)
{
    esp_log_level_set("LOG", ESP_LOG_INFO);
    ESP_LOGI("LOG", "this is a informative");
    ESP_LOGE("LOG", "this is an error");
    ESP_LOGW("LOG", "this is a warning");
    ESP_LOGV("LOG", "this is verbose");
    printf("hello world\n");

    init_buttons(NULL);
    gpio_install_isr_service(0);

    interruptHandlerQueue = xQueueCreate(10, sizeof(B1));
    xTaskCreate(buttonListener, "buttonListener", 2048, NULL, 1, NULL);

    gpio_set_intr_type(B1, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(B1, button_handler, (void *)B1);

    gpio_set_intr_type(B2, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(B2, button_handler, (void *)B2);





    gpio_isr_handler_add(C2, button_handler, (void *)C2);

    TimerHandle_t xTimer = xTimerCreate("rotation_timer", pdMS_TO_TICKS(timerTicks), true, NULL, on_timer);
    xTimerStart(xTimer, 0);

    // printf("app started %lld\n", esp_timer_get_time()/ 1000) ;
    // TimerHandle_t xTimer = xTimerCreate("mytimer", pdMS_TO_TICKS(1000), true, NULL, on_timer);
    // xTimerStart(xTimer, 0);
}
