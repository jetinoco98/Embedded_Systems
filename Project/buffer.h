#ifndef BUFFER_H
#define	BUFFER_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE_A 64
#define BUFFER_SIZE_B 10
#define COMMAND_MAX_LEN 16

typedef struct {
    char buffer[BUFFER_SIZE_A];
    int head;
    int tail;
    int count;
} TextCircularBuffer;

typedef struct {
    int buffer[BUFFER_SIZE_B];
    int head;
    int tail;
    int count;
} NumberCircularBuffer;

void init_text_buffer(TextCircularBuffer *cb);
int is_text_buffer_full(TextCircularBuffer *cb);
int is_text_buffer_empty(TextCircularBuffer *cb);
int write_text_buffer(TextCircularBuffer *cb, char c);

/**
 * A function to read a text circular buffer and update its tail parameter
 * 
 * Returns:
 * - `\0` if the buffer was full
 * - The character in the tail position of the buffer.
 */
char read_text_buffer(TextCircularBuffer *cb);

void init_number_buffer(NumberCircularBuffer *cb);
int is_number_buffer_full(NumberCircularBuffer *cb);
int is_number_buffer_empty(NumberCircularBuffer *cb);
int queue_number_buffer(NumberCircularBuffer *cb, int n);

/**
 * A function to read a numerical circular buffer and update its tail parameter
 * 
 * Returns:
 * - `-1` if the buffer was full
 * - The number in the tail position of the buffer.
 */
int dequeue_number_buffer(NumberCircularBuffer *cb);

/**
 * This parser reads a `TextCircularBuffer` for a match in the format "%PCCMD,x,t*". When found 
 * or after reaching a limit, it updates the parameters of the buffer. The numerical values are 
 * stored in each `NumberCircularBuffer`.
 * 
 * Returns:
 * - `0` The parser did not find any match.
 * - `1` The parser found a match and updated the `NumberCircularBuffer`.
 * - `2` The parser found a match but both the `NumberCircularBuffer` were full.
*/
int parse_uart_rx(TextCircularBuffer *cb, NumberCircularBuffer *commands, 
                    NumberCircularBuffer *times);

#endif