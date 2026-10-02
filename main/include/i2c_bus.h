#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c_master.h"

/**
 * Shared I2C bus for I2C_PORT (globals.h) -- imu.c (ICM-20948 + AK09916, via
 * bypass mode) and airspeed.c (MS4525DO) are three separate addressable
 * devices on the one physical bus, each running from its own task.
 *
 * Replaces the legacy driver/i2c.h API this project used until a confirmed
 * bench crash: sustained I2C traffic from two concurrent tasks eventually
 * drove the legacy driver into its own internal bus-recovery path
 * (i2c_hw_fsm_reset) badly enough to trip the interrupt watchdog and hard-
 * crash the chip -- a known weakness of that driver, which is why ESP-IDF
 * itself warns "please migrate your application code to adapt
 * driver/i2c_master.h" at every boot.
 */

// Creates the shared bus. NOT thread-safe, and deliberately not made so:
// i2c_new_master_bus() itself allocates memory and installs an interrupt
// handler, neither of which is safe to do inside a spinlock -- and a
// FreeRTOS mutex just moves the same "who creates the mutex" race one level
// down. The actual fix is to not need locking at all: call this once, early,
// from app_main()/setup_mode_run()'s single-threaded setup phase, before any
// sensor task is created -- the same convention config_store_init() and
// nav_init() already follow. Safe to call again after that (a no-op).
void i2c_bus_init(void);

// Registers a 7-bit device address on the shared bus and returns its device
// handle for i2c_master_transmit()/i2c_master_receive()/
// i2c_master_transmit_receive(). i2c_master_bus_add_device() itself is
// documented thread-safe (multiple drivers each adding their own device to
// one shared bus concurrently is the normal use case), so -- unlike bus
// creation above -- this one doesn't need the same single-threaded-caller
// discipline; imu_task and airspeed_task can each call this from their own
// task without coordinating.
esp_err_t i2c_bus_add_device(uint8_t addr7, uint32_t clk_speed_hz, i2c_master_dev_handle_t *out_handle);

// Bring-up/debugging utility: probes every valid 7-bit address (0x08-0x77,
// the same range standard I2C scanner tools use -- addresses outside it are
// reserved) and logs each one that acknowledges. An empty scan (nothing
// responds anywhere) points at a bus-level problem -- power, pull-ups, SDA/
// SCL continuity -- rather than a single sensor's own wiring or a driver
// config issue, since it means the electrical bus itself isn't working, not
// just one device's protocol handling on top of it. Call after
// i2c_bus_init(). Safe to call any time after that; not on any hot path.
void i2c_bus_scan(void);

#endif // I2C_BUS_H
