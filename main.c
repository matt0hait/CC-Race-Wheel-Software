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
#include "pico/binary_info.h"
#include "hardware/gpio.h"
//#include "hardware/dma.h"
// TinyUSB
#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"

// @brief Static task for USBD
#if CFG_TUSB_DEBUG
    #define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE)
#else
    #define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE/2)
#endif

// @brief Static task for hid
#define HID_STACK_SIZE      configMINIMAL_STACK_SIZE

#define FIRST_BUTTON_GPIO   2   // @brief First button GPIO number
#define BUTTON_CNT          12  // @brief Number of buttons

uint16_t btnBuff = 0;           // @brief Button state buffer


void init_gpio(void) {
    // Setup Buttons, Using "hello_7segment.c as example
    for (int gpio = FIRST_BUTTON_GPIO; gpio < FIRST_BUTTON_GPIO + BUTTON_CNT; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_up(gpio); // All buttons pull to ground when pressed
        gpio_set_input_hysteresis_enabled(gpio,true); // Enable Schmitt triggers to debounce. Set slew rate if further issues.
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
    // skip if hid is not ready yet
    if ( !tud_hid_ready() ) return;
    switch(report_id) {
        case REPORT_ID_KEYBOARD: {
            // Used to avoid send multiple consecutive zero report for keyboard
            static bool has_keyboard_key = false;
            if ( btn ) {
                uint8_t keycode[6] = { 0 };
                keycode[0] = HID_KEY_F24;

                tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
                has_keyboard_key = true;
            } else {
                // send empty key report if previously has key pressed
                if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
                has_keyboard_key = false;
            }
        }
        break;
        case REPORT_ID_GAMEPAD: {
            // Used to avoid send multiple consecutive zero report for keyboard
            static bool has_gamepad_key = false;
            hid_gamepad_report_t report = {
                //.x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0, // TODO: Add temps sensor or IMU
                .hat = 0,
                .buttons = 0
            };
            //gpio_get
            if ( btn ) {
                report.hat = GAMEPAD_HAT_UP;
                report.buttons = GAMEPAD_BUTTON_A;
                tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
                has_gamepad_key = true;
            } else {
                report.hat = GAMEPAD_HAT_CENTERED;
                report.buttons = 0;
                if (has_gamepad_key) tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
                has_gamepad_key = false;
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
            send_hid_report(REPORT_ID_KEYBOARD, btn);
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
