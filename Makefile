CC = gcc
CFLAGS = -Wall -Wextra -O3 -march=native -std=c11 -D_DEFAULT_SOURCE -Isrc
LDFLAGS = -lm

TARGET = simorgh_pro
SRC = src/simorgh_pro.c
CONFIG_DIR = config

.PHONY: all clean run

all: $(CONFIG_DIR) $(TARGET)

$(CONFIG_DIR):
	mkdir -p $(CONFIG_DIR)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

distclean: clean
	rm -f $(CONFIG_DIR)/simorgh.conf