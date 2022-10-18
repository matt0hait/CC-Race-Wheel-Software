#include <sys/select.h>
#include <sys/cdefs.h>
#include <sys/select.h>
#include <sys/cdefs.h>
#include <sys/select.h>
#include <sys/cdefs.h>
#include <sys/select.h>
#include <sys/cdefs.h>
#include <sys/select.h>
#include <sys/cdefs.h>
// FreeRTOS
#include <FreeRTOS.h>
#ifdef __GNUC__
    #define USED __attribute__((used))
#else
    #define USED
#endif
//const int USED uxTopUsedPriority = configMAX_PRIORITIES - 1;
#include <task.h>
#include <queue.h>
#include "semphr.h"
#include "timers.h"
// C
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
// Pico SDK
#include "pico/stdlib.h"            // Includes `hardware_gpio.h`
#include "pico/bootrom.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
//#include "hardware/dma.h"
// TinyUSB
#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "custom_gamepad.h"

// @brief Static task for USBD
#if CFG_TUSB_DEBUG
    #define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE)
#else
    #define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE/2)
#endif

// @brief Static task for hid
#define HID_STACK_SIZE      configMINIMAL_STACK_SIZE

#define FIRST_BUTTON_GPIO   0   // @brief First button GPIO number
#define BUTTON_CNT          14  // @brief Number of buttons
#define ENCODER_CNT         5   // @brief Number of Encoders
#define ENCODER_MAX         15  // @brief Encoder max count
#define ENCODER_PIN_CNT     10  // @brief Number of encoder pins
#define PACKET_HOLD_CNT     10  // @brief Number of packets to hold encoder input
#define MASK(x) (1UL << (x))

uint16_t btnBuff = 0;           // @brief Button state buffer

static uint32_t RTOS_RunTimeCounter;    // @brief runtime counter, used for configGENERATE_RUNTIME_STATS

// Inter-task queues
QueueHandle_t enc_irq_queue = NULL;
// Task handles
TaskHandle_t handle_usb_device_task = NULL;
TaskHandle_t handle_hid_task = NULL;
TaskHandle_t handle_encoder_task = NULL;

// @brief Quadrature Encoder values for calculating steps
struct		encoder {
    uint32_t	Encoder_A;      // @brief Encoder Input Pin A
    uint32_t	Encoder_B;      // @brief Encoder Input Pin B
    bool			old_b;      // @brief Encoder B previous state
    bool			old_a;      // @brief Encoder A previous state
    bool			new_a;      // @brief Encoder A current state
    bool			new_b;      // @brief Encoder B current state
    uint8_t         new_count;  // @brief Current count from home
    uint8_t         hold_count; // @brief Reset position by rotating counterclockwise at least 2 full rotations to the lowest position.
} encoder;
struct encoder encoders[ENCODER_CNT];   // @brief array of encoder values
const int enc_LUT[16] = {0, 1, 2, 0, 2, 0, 0, 1, 1, 0, 0, 2, 0, 2, 1, 0}; // @brief Lookup table for encoder step from interrupts; 1: dec, 2: inc
const int enc_LU2T[16]= {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // @brief Lookup table for encoder step from interrupts
const uint8_t encoder_pins[10] = {14, 15,
                                  16, 17,
                                  18, 19,
                                  20, 21,
                                  22, 28};

void encoder_callback(uint gpio, uint32_t events) {
    gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false, &encoder_callback);
    uint temp_gpio = gpio; // May not need this. Might be able to use "gpio"
    xQueueSendToBackFromISR(enc_irq_queue, &temp_gpio, 0);
    events &= ~0xF;
}


_Noreturn static void encoder_task() {
    uint que_event = 0;
    while (true) {
        if (xQueueReceive(enc_irq_queue, &que_event, portMAX_DELAY) == pdPASS) {
            // Loop through axes and set new input
            size_t i = 0;
            for (; i < ENCODER_CNT; i++) {
                // Check A of encoder
                if ((uint) que_event == (uint) encoders[i].Encoder_A) {
                    encoders[i].new_a = gpio_get(encoders[i].Encoder_A);
                    break;    // Exit, no need to iterate through rest of axes.
                }
                // Check B of encoder
                if ((uint) que_event == (uint) encoders[i].Encoder_B) {
                    encoders[i].new_b = gpio_get(encoders[i].Encoder_B);
                    break;
                }
            }
            // Calculate count of interrupt axis
            int old_sum = (encoders[i].old_a << 1) + encoders[i].old_b;
            int new_sum = (encoders[i].new_a << 1) + encoders[i].new_b;
            int inc = old_sum * 4 + new_sum;
            if (encoders[i].hold_count == 0) {
                encoders[i].new_count = enc_LUT[inc];
                encoders[i].hold_count = PACKET_HOLD_CNT;
            }
            // Update old values for next interrupt
            encoders[i].old_a = encoders[i].new_a;
            encoders[i].old_b = encoders[i].new_b;
            // clear status flags
            gpio_set_irq_enabled_with_callback(que_event, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
        }
    }
}


void init_gpio(void) {
    // Setup Buttons, Using "hello_7segment.c as example
    for (int gpio = FIRST_BUTTON_GPIO; gpio < FIRST_BUTTON_GPIO + BUTTON_CNT; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_up(gpio); // All buttons pull to ground when pressed
        gpio_set_input_hysteresis_enabled(gpio,true); // Enable Schmitt triggers to debounce. Set slew rate if further issues.
        gpio_set_slew_rate(gpio, GPIO_SLEW_RATE_SLOW );
    }
    // Setup Encoders
    for (int gpio = 0, gpio_pins = 0; gpio < ENCODER_CNT; gpio++, gpio_pins += 2) {
        encoders[gpio].Encoder_A = encoder_pins[gpio_pins];
        encoders[gpio].Encoder_B = encoder_pins[gpio_pins+1];
        // Init Encoders params
        // TODO: Load from NVM to save encoder values
        encoders[gpio].old_a = gpio_get(encoders[gpio].Encoder_A);
        encoders[gpio].old_b = gpio_get(encoders[gpio].Encoder_B);
        encoders[gpio].new_count = 0;
        encoders[gpio].hold_count = 0;
    }
    for (int gpio = 0; gpio < ENCODER_PIN_CNT; gpio++) {
        gpio_init(encoder_pins[gpio]);
        gpio_set_dir(encoder_pins[gpio], GPIO_IN);
        gpio_pull_up(encoder_pins[gpio]); // All buttons pull to ground when pressed
        // Fast trigger *happens* to work better with tested HW setup
        gpio_set_input_hysteresis_enabled(encoder_pins[gpio],true); // Enable Schmitt triggers to debounce. Set slew rate if further issues.
        gpio_set_slew_rate(gpio, GPIO_SLEW_RATE_SLOW);
        // Start Interrupts
        gpio_set_irq_enabled_with_callback(encoder_pins[gpio], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    }

    /*
    // Setup DMA on buttons. Note, 12 DMA channels.
    int chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16); // Try to do all the buttons in one channel
    // 2.19.6.1 of RP2040 Datasheet For GPIO register locations
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(
            chan,          // Channel to be configured
            &c,            // The configuration we just created
            dst,           // The initial write address
            src,           // The initial read address
            count_of(src), // Number of transfers; in this case each is 1 byte.
            true           // Start immediately.
    );
    dma_channel_wait_for_finish_blocking(chan);
     */

}

_Noreturn static void usb_device_task(void* param);
_Noreturn static void hid_task();

int main(void) {
    board_init();
    init_gpio();
    xTaskCreate( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-3, &handle_usb_device_task);  // Create a task for tinyusb device stack
    xTaskCreate(hid_task, "hid", HID_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, &handle_hid_task);  // Create HID task
    xTaskCreate(encoder_task, "enc", HID_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &handle_encoder_task);  // Create encoder task
    enc_irq_queue = xQueueCreate(ENCODER_CNT, sizeof(uint8_t));

    vTaskStartScheduler();
    return 0;
}

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
_Noreturn static void usb_device_task(void* param) {
    (void) param;
    tusb_init();
    while (1) {
        tud_task(); // tinyusb device task
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
    //xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    //xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void) remote_wakeup_en;
    //xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    // xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn) {
    if ( !tud_hid_ready() ) return; // skip if hid is not ready yet
    switch(report_id) {
        case REPORT_ID_KEYBOARD: {
            // No use at this time
        }
        break;
        case REPORT_ID_GAMEPAD: {
            hid_custom_gamepad_report_t report = {
                .clutch = 0,
                .hat = 0,
                .buttons = 0,
                .dial_0 = 0,
                .dial_1 = 0,
                .dial_2 = 0,
                .dial_3 = 0,
                .dial_4 = 0
            };
            report.dial_0 = encoders[0].new_count;
            report.dial_1 = encoders[1].new_count;
            report.dial_2 = encoders[2].new_count;
            report.dial_3 = encoders[3].new_count;
            report.dial_4 = encoders[4].new_count;
            // Grab button data
            for (int gpio = FIRST_BUTTON_GPIO, offset = 0; gpio < FIRST_BUTTON_GPIO + BUTTON_CNT; gpio++, offset++) {
                if (gpio_get(gpio) == 0) report.buttons |= TU_BIT(offset);
            }
            // Reboot if GPIO 3, 5, 7, & 9 are pressed simultaneously.
            if ((report.buttons & 0x154) == 0x154) {
                reset_usb_boot(0,0);
            }
            tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
            for (uint8_t i = 0; i < ENCODER_CNT; i++) {
                if(encoders[i].hold_count == 0) {
                    encoders[i].new_count = 0;
                } else {
                    encoders[i].hold_count -= 1;
                }
            }
        }
        break;
        default: break;
    }
}

_Noreturn static void hid_task() {
    while(1) {
        // Poll every 10ms
        vTaskDelay(pdMS_TO_TICKS(10));
        uint32_t const btn = board_button_read();
        // Remote wakeup
        if ( tud_suspended() && btn ) {
            // Wake up host if we are in suspend mode and REMOTE_WAKEUP feature is enabled by host
            tud_remote_wakeup();
        } else {
            // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
            send_hid_report(REPORT_ID_GAMEPAD, btn);
        }
    }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint8_t len) {
    (void) instance;
    (void) len;
    uint8_t next_report_id = report[0] + 1;
    if (next_report_id < REPORT_ID_COUNT) {
        send_hid_report(next_report_id, board_button_read());
    }
}


// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    // TODO not Implemented
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    (void) instance;
    if (report_type == HID_REPORT_TYPE_OUTPUT) {
        // Set keyboard LED e.g Capslock, Numlock etc...
        if (report_id == REPORT_ID_KEYBOARD) {
            // bufsize should be (at least) 1
            if ( bufsize < 1 ) return;
            uint8_t const kbd_leds = buffer[0];
            if (kbd_leds & KEYBOARD_LED_CAPSLOCK) {
                // Capslock On: disable blink, turn led on
                //board_led_write(true);
            } else {
                // Caplocks Off: back to normal blink
                //board_led_write(false);
            }
        }
    }
}

// Setup function for portCONFIGURE_TIMER_FOR_RUN_TIME_STATS
// Enables run time tracking
void RTOS_AppConfigureTimerForRuntimeStats(void) {
    RTOS_RunTimeCounter = 0;
    //EnableIRQ(FTM0_IRQn);
}

// Callback function for portCONFIGURE_TIMER_FOR_RUN_TIME_STATS
// Enables run time tracking
uint32_t RTOS_AppGetRuntimeCounterValueFromISR(void) {
    return time_us_32();
}