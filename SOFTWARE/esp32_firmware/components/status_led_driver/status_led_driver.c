#include "status_led_driver.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "neopixel.h"

#define STATUS_LED_DRIVER_TASK_STACK_SIZE 2048

#define BRIGHTNESS 10

static void status_led_driver_task(void *arg) {}

tNeopixelContext neopixels;

void set_status(enum eStatus status)
{
    switch (status) {
        case eWifiDisconnected:
            neopixel_SetPixel(
              neopixels, (tNeopixel[1]){ { 2, NP_RGB(BRIGHTNESS, 0, 0) } }, 1);
            break;
        case eWifiProvisioning:
            neopixel_SetPixel(
              neopixels,
              (tNeopixel[1]){ { 2, NP_RGB(BRIGHTNESS, BRIGHTNESS, 0) } },
              1);
            break;
        case eWifiConnected:
            neopixel_SetPixel(
              neopixels, (tNeopixel[1]){ { 2, NP_RGB(0, BRIGHTNESS, 0) } }, 1);
            break;
        case eAgentDisconnected:
            neopixel_SetPixel(
              neopixels, (tNeopixel[1]){ { 1, NP_RGB(BRIGHTNESS, 0, 0) } }, 1);
            break;
        case eAgentConnected:
            neopixel_SetPixel(
              neopixels, (tNeopixel[1]){ { 1, NP_RGB(0, BRIGHTNESS, 0) } }, 1);
            break;
        case eSystemError:
            neopixel_SetPixel(
              neopixels, (tNeopixel[1]){ { 0, NP_RGB(BRIGHTNESS, 0, 0) } }, 1);
            break;
        case eImuInitFailed:
            neopixel_SetPixel(
              neopixels,
              (tNeopixel[1]){ { 0, NP_RGB(BRIGHTNESS, 0, BRIGHTNESS) } },
              1);
            break;
        case eLidarInitFailed:
            neopixel_SetPixel(
              neopixels,
              (tNeopixel[1]){ { 0, NP_RGB(0, BRIGHTNESS, BRIGHTNESS) } },
              1);
            break;
        case eDriveBaseInitFailed:
            neopixel_SetPixel(
              neopixels,
              (tNeopixel[1]){ { 0, NP_RGB(BRIGHTNESS, BRIGHTNESS, 0) } },
              1);
            break;
        case eSystemGood:
            neopixel_SetPixel(
              neopixels, (tNeopixel[1]){ { 0, NP_RGB(0, BRIGHTNESS, 0) } }, 1);
            break;
    }
}

void status_led_driver_init()
{
    neopixels = neopixel_Init(3, 10);

    neopixel_SetPixel(neopixels,
                      (tNeopixel[3]){
                        { 0, NP_RGB(0, 0, 0) }, /* red */
                        { 1, NP_RGB(0, 0, 0) }, /* green */
                        { 2, NP_RGB(0, 0, 0) }, /* blue */
                      },
                      3);
}
