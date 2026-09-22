#ifndef FEATURES_H
#define FEATURES_H

// Compile-time feature flags for hardware not present on the current PCB
// revision. Nothing includes this yet -- it's a placeholder for a future
// board rev that adds an SD card slot for flight-data logging (diagnostics
// screen, deferred to a follow-up iteration -- see setup mode's Outputs/
// Mission/PID screens for what v1 actually covers). Uncomment once that
// logging driver actually exists; leave undefined until then.
// #define SDCARD_SUPPORT

#endif // FEATURES_H
