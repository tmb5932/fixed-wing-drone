CC = gcc
CFLAGS = -Wall -Wextra -O2 -I ../main/include

# pid.c is pulled directly from main/ (not copied) -- the whole point of the
# SITL is that it's testing the exact code that flies, not a reimplementation.
SRCS = ../main/pid.c plant.c actuator.c sitl_main.c
BIN = sitl_sim.exe

all: $(BIN)

$(BIN): $(SRCS) plant.h actuator.h ../main/include/pid.h
	$(CC) $(CFLAGS) -o $(BIN) $(SRCS) -lm

clean:
	rm -f $(BIN) output.csv

.PHONY: all clean
