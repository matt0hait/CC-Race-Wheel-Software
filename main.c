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
#define BUTTON_CNT          13  // @brief Number of buttons that short to ground
#define BUTTON_CNT_POS      9   // @brief Number of buttons that short to +5v
#define ENCODER_CNT         0   // @brief Number of Encoders
#define ENCODER_MAX         0   // @brief Encoder max count
#define ENCODER_PIN_CNT     0
#define MASK(x) (1UL << (x))

uint16_t btnBuff = 0;           // @brief Button state buffer

static uint32_t RTOS_RunTimeCounter;    // @brief runtime counter, used for configGENERATE_RUNTIME_STATS

// @brief Quadrature Encoder values for calculating steps
struct		encoder {
    uint32_t	Encoder_A;      // @brief Encoder Input Pin A
    uint32_t	Encoder_B;      // @brief Encoder Input Pin B
    bool			old_b;      // @brief Encoder B previous state
    bool			old_a;      // @brief Encoder A previous state
    bool			new_a;      // @brief Encoder A current state
    bool			new_b;      // @brief Encoder B current state
    uint8_t         new_count;  // @brief Current count from home
    uint8_t         reset_cnt;  // @brief Reset position by rotating counterclockwise at least 2 full rotations to the lowest position.
} encoder;
struct encoder encoders[ENCODER_CNT];   // @brief array of encoder values
const int enc_LUT[16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // @brief Lookup table for encoder step from interrupts
const uint8_t encoder_pins[10] = {14, 15,
                                  16, 17,
                                  18, 19,
                                  20, 21,
                                  22, 28};

void encoder_callback(uint gpio, uint32_t events) {
    // Loop through axes and set new input
    size_t i = 0;
    for ( ; i < ENCODER_CNT; i++) {
        // Check A of encoder
        if ((uint)gpio == (uint)encoders[i].Encoder_A) {
            encoders[i].new_a = gpio_get(encoders[i].Encoder_A);
            break;	// Exit, no need to iterate through rest of axes.
        }
        // Check B of encoder
        if ((uint)gpio == (uint)encoders[i].Encoder_B) {
            encoders[i].new_b = gpio_get(encoders[i].Encoder_B);
            break;
        }
    }
    // Calculate count of interrupt axis
    int old_sum = (encoders[i].old_a << 1) + encoders[i].old_b;
    int new_sum = (encoders[i].new_a << 1) + encoders[i].new_b;
    int inc = old_sum*4+new_sum;
    encoders[i].new_count +=  enc_LUT[inc];
    // If rotating counterclockwise, add to reset sequence.
    if (enc_LUT[inc] == 1) {
        encoders[i].reset_cnt++;
    }
    if (enc_LUT[inc] == -1) {
        // Normal behaviour, reset the "Reset" counter.
        encoders[i].reset_cnt = 0;
    }
    if (encoders[i].reset_cnt > 30) {
        // Encoder has been rotated counterclockwise consistently for two rotations, reset to the lowest position.
        encoders[i].new_count = 0;
    }
    // TODO: Check if LUT = 2 (encoder skipped a step) then reboot, rehome, or count errors.
    // Check for count overflows
    if(encoders[i].new_count == ((uint8_t)-1)) {
        // Axis has turned below 0, clamp count
        encoders[i].new_count = ENCODER_MAX;
    }
    if(encoders[i].new_count > ENCODER_MAX) {
        encoders[i].new_count = 0;  // Clamp count
    }
    // Update old values for next interrupt
    encoders[i].old_a = encoders[i].new_a;
    encoders[i].old_b = encoders[i].new_b;
    // clear status flags
    events &= ~0xF;
}


void init_gpio(void) {
    // Setup Buttons, Using "hello_7segment.c as example
    for (int gpio = FIRST_BUTTON_GPIO; gpio < FIRST_BUTTON_GPIO + BUTTON_CNT; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_up(gpio); // All buttons pull to ground when pressed
        gpio_set_input_hysteresis_enabled(gpio,true); // Enable Schmitt triggers to debounce. Set slew rate if further issues.
    }
    // Setup Buttons that short to +5V
    for (int gpio = 14; gpio < 14 + BUTTON_CNT_POS; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_down(gpio); // Buttons pull to +5V when pressed
        gpio_set_input_hysteresis_enabled(gpio,true); // Enable Schmitt triggers to debounce. Set slew rate if further issues.
    }
    // Setup Encoders
    for (int gpio = 0, gpio_pins = 0; gpio < ENCODER_CNT; gpio++, gpio_pins += 2) {
        encoders[gpio].Encoder_A = encoder_pins[gpio_pins];
        encoders[gpio].Encoder_B = encoder_pins[gpio_pins+1];
    }
    for (int gpio = 0; gpio < ENCODER_PIN_CNT; gpio++) {
        gpio_init(encoder_pins[gpio]);
        gpio_set_dir(encoder_pins[gpio], GPIO_IN);
        gpio_pull_up(encoder_pins[gpio]); // All buttons pull to ground when pressed
        gpio_set_input_hysteresis_enabled(encoder_pins[gpio],true); // Enable Schmitt triggers to debounce. Set slew rate if further issues.
        // Init Encoders params
        // TODO: Load from NVM to save encoder values
        encoders[gpio/2].old_a = gpio_get(encoders[gpio/2].Encoder_A);
        encoders[gpio/2].old_b = gpio_get(encoders[gpio/2].Encoder_B);
        encoders[gpio/2].new_count = 0;
        encoders[gpio/2].reset_cnt = 0;
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
    xTaskCreate( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-3, NULL);  // Create a task for tinyusb device stack
    xTaskCreate(hid_task, "hid", HID_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, NULL);  // Create HID task

    // skip starting scheduler (and return) for ESP32-S2 or ESP32-S3
    #if !( TU_CHECK_MCU(ESP32S2) || TU_CHECK_MCU(ESP32S3) )
        vTaskStartScheduler();
    #endif
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
                .dial_1 = 0,
                .dial_2 = 0,
                .dial_3 = 0,
                .dial_4 = 0
            };
            report.dial_0 = TU_BIT(encoders[0].new_count);
            report.dial_1 = TU_BIT(encoders[1].new_count);
            report.dial_2 = TU_BIT(encoders[2].new_count);
            report.dial_3 = TU_BIT(encoders[3].new_count);
            report.dial_4 = TU_BIT(encoders[4].new_count);
            // Grab button data
            for (int gpio = FIRST_BUTTON_GPIO, offset = 0; gpio < FIRST_BUTTON_GPIO + BUTTON_CNT; gpio++, offset++) {
                if (gpio_get(gpio) == 0) report.buttons |= TU_BIT(offset);
            }
            for (int gpio = 14, offset = 14; gpio < 14 + BUTTON_CNT_POS; gpio++, offset++) {
                if (gpio_get(gpio) == 1) report.buttons |= TU_BIT(offset);
            }
            // Reboot if GPIO 3, 5, 7, 9, & 11 are pressed simultaneously.
            if ((report.buttons & 0x554) == 0x554) {
                reset_usb_boot(0,0);
            }
            tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
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