// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>


// Frame delimiters and constants
#define FLAG 0x7E
#define A_SE 0x03
#define A_RE 0x01

// Control field values
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B

// Maximum frame size
#define MAX_FRAME_SIZE 1024

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    STOP_R
} State;

volatile int STOP = FALSE;
int alarmOn = FALSE;
int alarmCount = 0;
int timeout = 0; 



// Create a Supervision (S) or Unnumbered (U) Frame
int createSupervisionFrame(unsigned char* frame, unsigned char address, unsigned char control) {
    frame[0] = FLAG;         // Start flag
    frame[1] = address;      // Address field
    frame[2] = control;      // Control field
    frame[3] = address ^ control; // BCC1 (A XOR C)
    frame[4] = FLAG;         // End flag
    return 5; // Length of the frame
}

// Create an Information (I) Frame
int createInformationFrame(unsigned char* frame, unsigned char address, unsigned char control, const unsigned char* data, int dataLength) {
    int i;
    frame[0] = FLAG;         // Start flag
    frame[1] = address;      // Address field
    frame[2] = control;      // Control field
    frame[3] = address ^ control; // BCC1 (A XOR C)
    
    // Copy the data field
    for (i = 0; i < dataLength; i++) {
        frame[4 + i] = data[i];
    }
    
    // Calculate BCC2 (XOR of all data bytes)
    unsigned char bcc2 = 0;
    for (i = 0; i < dataLength; i++) {
        bcc2 ^= data[i];
    }
    frame[4 + dataLength] = bcc2; // BCC2

    frame[5 + dataLength] = FLAG; // End flag
    return 6 + dataLength; // Length of the frame
}

// Validate a received frame
int validateFrame(const unsigned char* frame, int length) {
    if (frame[0] != FLAG || frame[length - 1] != FLAG) {
        return -1; // Invalid start or end flag
    }
    
    // Validate BCC1
    if (frame[3] != (frame[1] ^ frame[2])) {
        return -1; // BCC1 error
    }
    
    // If it's an Information frame, validate BCC2
    if (length > 5) {
        unsigned char bcc2 = 0;
        for (int i = 4; i < length - 2; i++) {
            bcc2 ^= frame[i];
        }
        if (bcc2 != frame[length - 2]) {
            return -1; // BCC2 error
        }
    }
    return 0; // Frame is valid
}

void alarmHandler(int signal) {
    alarmOn = TRUE;
    alarmCount++;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    // Define frame arrays
    unsigned char setFrame[5];
    unsigned char uaFrame[5];
    int retries = 0;
    int timeout = connectionParameters.timeout;

    // Create SET frame
    createSupervisionFrame(setFrame, A_SE, C_SET);

    if (connectionParameters.role == LlTx)
    {
        // Transmitter side
        (void) signal(SIGALRM, alarmHandler);

        while (retries < connectionParameters.nRetransmissions)
        {
            // Send SET frame
            if (writeBytesSerialPort(setFrame, 5) < 0)
            {
                printf("Failed to send SET frame\n");
                return -1;
            }

            printf("SET frame sent, waiting for UA...\n");

            // Set alarm for the timeout duration
            alarmOn = 0; // Reset alarm flag
            alarm(timeout); // Set the alarm to trigger after the specified timeout

            // State machine for receiving UA frame
            State state = START;
            int bytesRead = 0;
            unsigned char byte;

            while (state != STOP_R && !alarmOn)
            {
                int res = readByteSerialPort(&byte);
                if (res > 0)
                {
                    printf("Received byte: 0x%02X\n", byte);
                    uaFrame[bytesRead++] = byte;

                    switch (state)
                    {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == A_RE) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == C_UA) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (A_RE ^ C_UA)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG) state = STOP_R;
                            else state = START;
                            break;
                        default:
                            break;
                    }
                }
                else
                {
                    // If no byte is received, we just wait until the alarm goes off
                }
            }

            // Disable the alarm
            alarm(0);

            if (state == STOP_R && validateFrame(uaFrame, 5) == 0)
            {
                // Received valid UA
                printf("UA frame received, connection established.\n");
                return 1;
            }
            else
            {
                printf("Invalid UA frame received or timeout occurred.\n");
            }

            // Retry if the UA was not received or was invalid
            printf("Retry %d: Resending SET frame...\n", retries + 1);
            retries++;
        }

        // Failed to establish a connection
        printf("Connection failed after %d attempts\n", retries);
        return -1;
    }
    else if (connectionParameters.role == LlRx)
    {
        // Receiver side
        printf("Waiting for SET frame...\n");

        // State machine for receiving SET frame
        State state = START;
        int bytesRead = 0;
        unsigned char byte;

        while (alarmCount < connectionParameters.nRetransmissions)
        {
            int res = readByteSerialPort(&byte);
            if (res > 0)
            {
                printf("Received byte: 0x%02X\n", byte);
                switch (state)
                {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_SE) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == C_SET) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_SE ^ C_SET)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) state = STOP_R;
                        else state = START;
                        break;
                    default:
                        break;
                }
                setFrame[bytesRead++] = byte;
            }
            else
            {
                // If no byte is received, the loop continues to wait for the next byte
                // and the alarm mechanism ensures it doesn't wait indefinitely
            }

            if (state == STOP_R && validateFrame(setFrame, 5) == 0)
            {
                // Received valid SET
                printf("SET frame received, sending UA...\n");

                // Create UA frame
                createSupervisionFrame(uaFrame, A_RE, C_UA);

                // Send UA frame
                if (writeBytesSerialPort(uaFrame, 5) < 0)
                {
                    printf("Failed to send UA frame\n");
                    return -1;
                }

                printf("UA frame sent, connection established.\n");
                return 1;
            }
        }

        printf("Timeout occurred or invalid SET frame received\n");
        return -1;
    }

    return -1;
}

    


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 0;
}

