CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -g -I../Lib -Iunity

UNITY_SRC = unity/unity.c
LIB_SRCS = $(wildcard ../Lib/*.c)

TEST_SRCS = $(wildcard test_*.c)
TEST_BINS = $(TEST_SRCS:.c=)

all: $(TEST_BINS)
	@for t in $(TEST_BINS); do echo "\n=== Running $$t ===" && ./$$t; done

test_%: test_%.c $(UNITY_SRC) $(LIB_SRCS)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TEST_BINS)

.PHONY: all clean
