#include <stdint.h>

#include "deck.h"
#include "FreeRTOS.h"
#include "log.h"
#include "task.h"

static uint8_t enablePressed;
static uint8_t disablePressed;
static uint8_t takeoffPressed;
static uint8_t landPressed;
static uint8_t stopPressed;

#define SAMPLE_MS 20
#define DEBOUNCE_SAMPLES 3
#define EVENT_PULSE_SAMPLES 10

typedef struct {
  uint8_t stable;
  uint8_t candidate;
  uint8_t candidateCount;
} DebouncedButton;

static void updateButton(DebouncedButton *button, uint8_t rawPressed)
{
  if (rawPressed == button->candidate) {
    if (button->candidateCount < DEBOUNCE_SAMPLES) {
      button->candidateCount++;
    }
  } else {
    button->candidate = rawPressed;
    button->candidateCount = 1;
  }

  if (button->candidateCount >= DEBOUNCE_SAMPLES) {
    button->stable = button->candidate;
  }
}

void appMain(void)
{
  DebouncedButton button1 = {0};
  DebouncedButton button2 = {0};
  uint8_t button1WasPressed = 0;
  uint8_t button2WasPressed = 0;
  uint8_t stopHandled = 0;
  uint8_t enablePulse = 0;
  uint8_t disablePulse = 0;
  uint8_t stopPulse = 0;

  pinMode(DECK_GPIO_IO1, INPUT_PULLUP);
  pinMode(DECK_GPIO_IO2, INPUT_PULLUP);

  while (1) {
    updateButton(&button1, !digitalRead(DECK_GPIO_IO1));
    updateButton(&button2, !digitalRead(DECK_GPIO_IO2));

    if (enablePulse > 0) enablePulse--;
    if (disablePulse > 0) disablePulse--;
    if (stopPulse > 0) stopPulse--;

    // Stop on the debounced press, without waiting for release. Suppress
    // other actions until both buttons are released after an emergency stop.
    if (button2.stable && !button2WasPressed) {
      stopPulse = EVENT_PULSE_SAMPLES;
      stopHandled = 1;
      enablePulse = 0;
      disablePulse = 0;
    }

    if (button1.stable && !button1WasPressed && !stopHandled) {
      enablePulse = EVENT_PULSE_SAMPLES;
      disablePulse = 0;
    }

    if (!button1.stable && button1WasPressed) {
      enablePulse = 0;
      if (!stopHandled) {
        disablePulse = EVENT_PULSE_SAMPLES;
      }
    }

    if (!button1.stable && !button2.stable) {
      stopHandled = 0;
    }

    enablePressed = enablePulse > 0;
    disablePressed = disablePulse > 0;
    // Preserve the log interface; neither takeoff nor landing is a button action.
    takeoffPressed = 0;
    landPressed = 0;
    stopPressed = stopPulse > 0;

    button1WasPressed = button1.stable;
    button2WasPressed = button2.stable;
    vTaskDelay(M2T(SAMPLE_MS));
  }
}

LOG_GROUP_START(wand)
LOG_ADD(LOG_UINT8, enable, &enablePressed)
LOG_ADD(LOG_UINT8, disable, &disablePressed)
LOG_ADD(LOG_UINT8, takeoff, &takeoffPressed)
LOG_ADD(LOG_UINT8, land, &landPressed)
LOG_ADD(LOG_UINT8, stop, &stopPressed)
LOG_GROUP_STOP(wand)
