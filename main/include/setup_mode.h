#ifndef SETUP_MODE_H
#define SETUP_MODE_H

/**
 * Entered instead of normal flight init when the BOOT button is pressed
 * within the boot window checked in app_main() (see setup_mode_requested()
 * in main.c). Brings up the RC capture/PWM output hardware (for live bench
 * pass-through), the setup-mode WiFi AP, and the local HTTP/JSON setup UI.
 *
 * Never returns -- getting back to flight mode is a plain physical reset
 * with BOOT not held, same as any other boot decision made this early.
 */
void setup_mode_run(void);

#endif // SETUP_MODE_H
