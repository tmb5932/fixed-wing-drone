CC = gcc
CFLAGS = -Wall -Wextra -O2 -I ../main/include

# pid.c/gps_math.c are pulled directly from main/ (not copied) -- the whole
# point of the SITL is that it's testing the exact code that flies, not a
# reimplementation.
ATTITUDE_SRCS = ../main/pid.c plant.c actuator.c airframe_params.c noise.c sitl_main.c
ATTITUDE_BIN = sitl_sim.exe

NAV_SRCS = ../main/pid.c ../main/gps_math.c plant.c actuator.c airframe_params.c noise.c nav_world.c nav_sitl_main.c
NAV_BIN = nav_sim.exe

all: $(ATTITUDE_BIN) $(NAV_BIN)

$(ATTITUDE_BIN): $(ATTITUDE_SRCS) plant.h actuator.h airframe_params.h noise.h ../main/include/pid.h
	$(CC) $(CFLAGS) -o $(ATTITUDE_BIN) $(ATTITUDE_SRCS) -lm

$(NAV_BIN): $(NAV_SRCS) plant.h actuator.h airframe_params.h noise.h nav_world.h ../main/include/pid.h ../main/include/gps_math.h
	$(CC) $(CFLAGS) -o $(NAV_BIN) $(NAV_SRCS) -lm

test: all
	python3 tests/run_tests.py

clean:
	rm -f $(ATTITUDE_BIN) $(NAV_BIN) output.csv nav_output.csv

.PHONY: all test clean
