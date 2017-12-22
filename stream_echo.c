#include <unabto/unabto_stream.h>
#include <unabto/unabto_memory.h>
#include <stream_echo.h>

//
#define STREAM_MAX_SIZE     (172*STREAM_BUF_SIZE)
#define STREAM_BUF_SIZE     (12*1024)

/**
 * Stream echo server
 *
 * The echo server first receives an ``echo'' command, then it sends a
 * ``+'' if it accepts the request. When the command is accepted the
 * state is switched to an echo state.
 *
 * recv: echo\n
 * send: +\n
 * echo data until close
 */

typedef enum {
    ECHO_STATE_IDLE,
    ECHO_STATE_READ_COMMAND,
    ECHO_STATE_COMMAND_FAIL,
    ECHO_STATE_COMMAND_OK,
    ECHO_STATE_FORWARDING,
    ECHO_STATE_CLOSING
} echo_state;

typedef struct {
    uint16_t commandLength;
    bool commandOk;
    echo_state state;
} echo_stream;

echo_stream echo_streams[NABTO_MEMORY_STREAM_MAX_STREAMS];

//
static uint32_t requestedSize = 0;
static uint32_t totalSize = 0;
static uint8_t testBuffer[STREAM_BUF_SIZE] = {0};

void stream_echo_init() {
    memset(echo_streams, 0, sizeof(echo_streams));
}

void unabto_stream_accept(unabto_stream* stream) {
    echo_stream* echo = &echo_streams[unabto_stream_index(stream)];
    UNABTO_ASSERT(echo->state == ECHO_STATE_IDLE);
    memset(echo, 0, sizeof(echo_stream));
    echo->state = ECHO_STATE_READ_COMMAND;
}

void unabto_stream_event(unabto_stream* stream, unabto_stream_event_type type) {
    echo_stream* echo = &echo_streams[unabto_stream_index(stream)];

    if (echo->state == ECHO_STATE_IDLE) {
        return;
    }

    if (echo->state == ECHO_STATE_READ_COMMAND) {
        const uint8_t* buf;
        unabto_stream_hint hint;

        totalSize = 0;

        size_t readLength = unabto_stream_read(stream, &buf, &hint);
        if (readLength > 0)
        {
            size_t ackLength = readLength;

            requestedSize = atoi(buf);
            NABTO_LOG_INFO(("--> Client requested size (%d)", requestedSize));

            if (requestedSize <= STREAM_MAX_SIZE)
            {
                echo->state = ECHO_STATE_COMMAND_OK;
            }
            else
            {
                echo->state = ECHO_STATE_CLOSING;
            }


            if (!unabto_stream_ack(stream, buf, ackLength, &hint)) {
                echo->state = ECHO_STATE_CLOSING;
            }

        }
        else
        {
            if (hint != UNABTO_STREAM_HINT_OK) {
                echo->state = ECHO_STATE_CLOSING;
            }
        }
    }

    if (echo->state == ECHO_STATE_COMMAND_FAIL) {
        const char* failString = "-\n";
        unabto_stream_hint hint;
        unabto_stream_write(stream, (uint8_t*)failString, strlen(failString),
                            &hint);
        echo->state = ECHO_STATE_CLOSING;
    }

    if (echo->state == ECHO_STATE_COMMAND_OK) {
        const char* okString = "+\n";
        unabto_stream_hint hint;
        size_t wrote = unabto_stream_write(stream, (uint8_t*)okString,
                                           strlen(okString), &hint);
        if (wrote != strlen(okString)) {
            echo->state = ECHO_STATE_CLOSING;
        } else {
            echo->state = ECHO_STATE_FORWARDING;
        }
    }

    if (echo->state == ECHO_STATE_FORWARDING)
    {
        const uint8_t* buf = testBuffer;
        unabto_stream_hint hint;
        if (UNABTO_STREAM_EVENT_TYPE_DATA_WRITTEN == type)
        {
            size_t writeLength = unabto_stream_write(stream, buf, sizeof(testBuffer), &hint);
            if (writeLength > 0)
            {
                // NABTO_LOG_INFO(("-->Wrote [%" PRIu16 "] bytes to stream!", writeLength));
                totalSize += writeLength;
                if (requestedSize <= totalSize)
                {
                    echo->state = ECHO_STATE_CLOSING;
                }
            }
            else
            {
                if (hint != UNABTO_STREAM_HINT_OK)
                {
                    echo->state = ECHO_STATE_CLOSING;
                }
            }
        }
    }

    if (echo->state == ECHO_STATE_CLOSING)
    {
        if (unabto_stream_close(stream)) {
            unabto_stream_release(stream);
            echo->state = ECHO_STATE_IDLE;
        }
    }
}
