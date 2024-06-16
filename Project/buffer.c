#include "buffer.h"

//////////////////////////////////////
// Functions for TextCircularBuffer
/////////////////////////////////////

void init_text_buffer(TextCircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

int is_text_buffer_full(TextCircularBuffer *cb) {
    return cb->count == BUFFER_SIZE_A;
}

int is_text_buffer_empty(TextCircularBuffer *cb) {
    return cb->count == 0;
}

int write_text_buffer(TextCircularBuffer *cb, char c) {
    if (!is_text_buffer_full(cb)) {
        cb->buffer[cb->head] = c;
        cb->head = (cb->head + 1) % BUFFER_SIZE_A;
        cb->count++;
        return 0;
    }
    return 1;
}

char read_text_buffer(TextCircularBuffer *cb) {
    char c = '\0';
    if (!is_text_buffer_empty(cb)) {
        c = cb->buffer[cb->tail];
        cb->tail = (cb->tail + 1) % BUFFER_SIZE_A;
        cb->count--;
    }
    return c;
}


//////////////////////////////////////
// Functions for NumberCircularBuffer
/////////////////////////////////////

void init_number_buffer(NumberCircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

int is_number_buffer_full(NumberCircularBuffer *cb) {
    return cb->count == BUFFER_SIZE_B;
}

int is_number_buffer_empty(NumberCircularBuffer *cb) {
    return cb->count == 0;
}

int queue_number_buffer(NumberCircularBuffer *cb, int n) {
    if (!is_number_buffer_full(cb)) {
        cb->buffer[cb->head] = n;
        cb->head = (cb->head + 1) % BUFFER_SIZE_B;
        cb->count++;
        return 0;
    }
    return 1;
}

int dequeue_number_buffer(NumberCircularBuffer *cb){
    int n = -1;
    if (!is_number_buffer_empty(cb)){
        n = cb->buffer[cb->tail];
        cb->tail = (cb->tail + 1) % BUFFER_SIZE_B;
        cb->count--;
    }
    return n;
}


//////////////////////////////////////
// Additional
/////////////////////////////////////

int is_between(int value, int min, int max) {
    if (value > min && value < max){return 1;}
    else {return 0;}
}


int parse_uart_rx(TextCircularBuffer *cb, NumberCircularBuffer *commands, 
                    NumberCircularBuffer *times) {

    if (is_text_buffer_empty(cb)) {
        return 0;
    }  

    // First character 
    char start = cb->buffer[cb->tail];
    if (start != '$'){
        cb->tail = (cb->tail + 1) % BUFFER_SIZE_A;
        cb->count--;
        return 0;
    }

    // Reset if '*' is not found after limit
    int limit = 16;
    if (cb->count >= limit){
        cb->tail = (cb->tail + cb->count) % BUFFER_SIZE_A;
        cb->count = 0;
        return 0;
    }

    // Get the index of the character '*'
    int found_match = 0;
    int start_index = cb->tail;
    int end_index = cb->tail;   // This will change on following loop

    while (end_index != cb->head) { // loop until the end of the circular buffer
        if (cb->buffer[end_index] == '*') {
            found_match = 1;
            break;
        }
        end_index = (end_index + 1) % BUFFER_SIZE_A;
    }

    if (!found_match){return 0;}

    // Update COUNT value
    int length;
    if (cb->head > start_index){
        length = cb->head - start_index;
    }
    else{
        length = (BUFFER_SIZE_A + cb->head - start_index) % BUFFER_SIZE_A;
    }
    cb->count = cb->count - length + 1;

    // Update TAIL value: Becomes the end_index where * was found
    cb->tail = end_index;

    // Obtain the substring from the tail (start_index) to the new tail location (end_index)
    char raw_command[COMMAND_MAX_LEN];
    int rc_index = 0;

    int i = start_index;
    while (1) {
        raw_command[rc_index] = cb->buffer[i];
        rc_index++;
        if (i == end_index) {break;}
        i = (i + 1) % BUFFER_SIZE_A; // Move to the next index, wrapping around
    }

    // Extract the command and its corresponding time: Send it to 2 buffers (commands, times)
    int x;
    int t;
    if (sscanf(raw_command, "$PCCMD,%d,%d*", &x, &t) == 2) {
        // Check if the integers are valid
        if (is_between(x, 0, 5) && is_between(t, 0, 9999)){
            // Add them to the FIFO queue
            int result_queue_x = queue_number_buffer(commands, x);
            int result_queue_t = queue_number_buffer(times, t);
            if (result_queue_x || result_queue_t){return 2;}
            else {return 1;}
        }
    } 
    return 0;
}