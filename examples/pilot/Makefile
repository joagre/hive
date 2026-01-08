# Makefile for pilot example (Webots controller)
#
# This builds a Webots controller that uses the hive actor runtime.
#
# Usage:
#   make              - Build the controller
#   make clean        - Remove build artifacts
#   make install      - Copy controller to Webots project
#
# Requirements:
#   - WEBOTS_HOME environment variable set
#   - hive runtime library built (../../build/libhive.a)

# Webots paths
WEBOTS_HOME ?= /usr/local/webots
WEBOTS_INCLUDE = $(WEBOTS_HOME)/include/controller/c
WEBOTS_LIB = $(WEBOTS_HOME)/lib/controller

# Hive runtime paths (relative to this directory)
HIVE_ROOT = ../..
HIVE_INCLUDE = $(HIVE_ROOT)/include
HIVE_LIB = $(HIVE_ROOT)/build

# Compiler settings
CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -g
CFLAGS += -I$(WEBOTS_INCLUDE) -I$(HIVE_INCLUDE) -I.
CFLAGS += -MMD -MP
LDFLAGS = -L$(WEBOTS_LIB) -L$(HIVE_LIB)
LDLIBS = -lController -lhive -lm

# Target
TARGET = pilot
SRCS = pilot.c pid.c altitude_actor.c attitude_actor.c motor_actor.c
OBJS = $(SRCS:.c=.o)
DEPS = $(SRCS:.c=.d)

# Webots controller directory (for install)
CONTROLLER_DIR = controllers/pilot

.PHONY: all clean install check-webots check-hive

all: check-webots check-hive $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS)

install: $(TARGET)
	@mkdir -p $(CONTROLLER_DIR)
	cp $(TARGET) $(CONTROLLER_DIR)/

check-webots:
	@if [ ! -d "$(WEBOTS_HOME)" ]; then \
		echo "Error: WEBOTS_HOME not set or invalid: $(WEBOTS_HOME)"; \
		echo "Set WEBOTS_HOME to your Webots installation directory"; \
		exit 1; \
	fi

check-hive:
	@if [ ! -f "$(HIVE_LIB)/libhive.a" ]; then \
		echo "Error: hive library not found at $(HIVE_LIB)/libhive.a"; \
		echo "Run 'make' in $(HIVE_ROOT) first"; \
		exit 1; \
	fi

# Auto-generated dependencies
-include $(DEPS)
