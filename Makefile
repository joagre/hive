# Compiler and flags
CC := gcc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
CPPFLAGS := -Iinclude -D_POSIX_C_SOURCE=200809L
DEPFLAGS = -MMD -MP
LDFLAGS :=
LDLIBS := -pthread

# Directories
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
EXAMPLES_DIR := examples

# Source files
SRCS := $(wildcard $(SRC_DIR)/*.c)
ASM_SRCS := $(wildcard $(SRC_DIR)/*.S)
OBJS := $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o) $(ASM_SRCS:$(SRC_DIR)/%.S=$(BUILD_DIR)/%.o)
DEPS := $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.d)

# Library
LIB := $(BUILD_DIR)/librt.a

# Examples
EXAMPLE_SRCS := $(wildcard $(EXAMPLES_DIR)/*.c)
EXAMPLES := $(EXAMPLE_SRCS:$(EXAMPLES_DIR)/%.c=$(BUILD_DIR)/%)

# Benchmarks
BENCHMARKS_DIR := benchmarks
BENCHMARK_SRCS := $(wildcard $(BENCHMARKS_DIR)/*.c)
BENCHMARKS := $(BENCHMARK_SRCS:$(BENCHMARKS_DIR)/%.c=$(BUILD_DIR)/%)

# Default target
.PHONY: all
all: $(LIB) $(EXAMPLES) $(BENCHMARKS)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile source files with dependency generation
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) -c $< -o $@

# Compile assembly files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.S | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) -c $< -o $@

# Create static library
$(LIB): $(OBJS)
	ar rcs $@ $^

# Build examples
$(BUILD_DIR)/%: $(EXAMPLES_DIR)/%.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -L$(BUILD_DIR) -lrt $(LDLIBS)

# Build benchmarks
$(BUILD_DIR)/%: $(BENCHMARKS_DIR)/%.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -L$(BUILD_DIR) -lrt $(LDLIBS)

# Run benchmarks
.PHONY: bench
bench: $(BUILD_DIR)/bench
	./$(BUILD_DIR)/bench

# Clean
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

# Run ping-pong example
.PHONY: run-pingpong
run-pingpong: $(BUILD_DIR)/pingpong
	./$(BUILD_DIR)/pingpong

# Run file I/O example
.PHONY: run-fileio
run-fileio: $(BUILD_DIR)/fileio
	./$(BUILD_DIR)/fileio

# Run echo server/client example
.PHONY: run-echo
run-echo: $(BUILD_DIR)/echo
	./$(BUILD_DIR)/echo

# Help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all               - Build library, examples, and benchmarks (default)"
	@echo "  clean             - Remove build artifacts"
	@echo "  bench             - Build and run benchmark suite"
	@echo "  run-pingpong      - Build and run ping-pong example"
	@echo "  run-fileio        - Build and run file I/O example"
	@echo "  run-echo          - Build and run echo server/client example"
	@echo "  help              - Show this help message"

# Dependencies
.PHONY: deps
deps:
	@echo "No external dependencies required"

# Print variables for debugging
.PHONY: print-%
print-%:
	@echo '$*=$($*)'

# Include automatically generated dependencies
-include $(DEPS)
