// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>


// Frame delimiters and constants
#define FLAG 0x7E
#define A_SENDER 0x03
#define A_RECEIVER 0x01

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

LinkLayer currentLinkLayer;



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


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    currentLinkLayer = connectionParameters;

    // Define frame arrays
    unsigned char setFrame[5];
    unsigned char uaFrame[5];
    int retries = 0;

    // Create SET frame
    createSupervisionFrame(setFrame, A_SENDER, C_SET);

    if (connectionParameters.role == LlTx)
    {
        // Transmitter side
        while (retries < connectionParameters.nRetransmissions)
        {
            // Send SET frame
            if (writeBytesSerialPort(setFrame, 5) < 0)
            {
                printf("Failed to send SET frame\n");
                return -1;
            }

            printf("SET frame sent, waiting for UA...\n");

            // Wait for UA frame
            int res = readByteSerialPort(uaFrame);
            if (res > 0 && uaFrame[2] == C_UA && validateFrame(uaFrame, 5) == 0)
            {
                // Received valid UA
                printf("UA frame received, connection established.\n");
                return 1;
            }

            // Retry if UA was not received
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

        while (1)
        {
            // Wait for SET frame
            int res = readByteSerialPort(setFrame);
            if (res > 0 && setFrame[2] == C_SET && validateFrame(setFrame, 5) == 0)
            {
                // Received valid SET
                printf("SET frame received, sending UA...\n");

                // Create UA frame
                createSupervisionFrame(uaFrame, A_RECEIVER, C_UA);

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
    unsigned char discFrame[5];
    unsigned char uaFrame[5];
    int retries = 0;

    if (currentLinkLayer.role == LlTx)
    {
        // Transmitter side
        createSupervisionFrame(discFrame, A_SENDER, C_DISC);

        // Send DISC frame
        while (retries < currentLinkLayer.nRetransmissions)
        {
            if (writeBytesSerialPort(discFrame, 5) < 0)
            {
                printf("Failed to send DISC frame\n");
                return -1;
            }

            printf("DISC frame sent, waiting for DISC response...\n");

            // Wait for DISC response
            int res = readByteSerialPort(discFrame);
            if (res > 0 && discFrame[2] == C_DISC && validateFrame(discFrame, 5) == 0)
            {
                // Received DISC, send UA
                createSupervisionFrame(uaFrame, A_RECEIVER, C_UA);
                if (writeBytesSerialPort(uaFrame, 5) < 0)
                {
                    printf("Failed to send UA frame\n");
                    return -1;
                }

                // Connection successfully closed
                printf("UA frame sent, connection closed.\n");
                break;
            }
            retries++;
        }
    }
    else if (currentLinkLayer.role == LlRx)
    {
        // Receiver side
        printf("Waiting for DISC frame...\n");

        while (1)
        {
            // Wait for DISC frame
            int res = readByteSerialPort(discFrame);
            if (res > 0 && discFrame[2] == C_DISC && validateFrame(discFrame, 5) == 0)
            {
                // Received DISC, send DISC response
                createSupervisionFrame(discFrame, A_RECEIVER, C_DISC);
                if (writeBytesSerialPort(discFrame, 5) < 0)
                {
                    printf("Failed to send DISC response\n");
                    return -1;
                }

                printf("DISC response sent, waiting for UA...\n");

                // Wait for UA frame
                int uaRes = readByteSerialPort(uaFrame);
                if (uaRes > 0 && uaFrame[2] == C_UA && validateFrame(uaFrame, 5) == 0)
                {
                    // Connection successfully closed
                    printf("UA frame received, connection closed.\n");
                    break;
                }
            }
        }
    }

    // Close the serial port
    return closeSerialPort();
}

