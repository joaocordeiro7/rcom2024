// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>


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
#define ESC 0x7D

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
    DATA_RCV,
    STOP_R
} State;

volatile int STOP = FALSE;
int alarmOn = FALSE;
int alarmCount = 0;
int timeout = 0; 
int retransmissions = 0;
char* serialPort = 0;
LinkLayer currentLinkLayer;
int serialFd = -1;
time_t start_time, end_time;
int frames_sent = 0;
int frames_received = 0;

// Create a Supervision (S) or Unnumbered (U) Frame
int createSupervisionFrame(unsigned char* frame, unsigned char address, unsigned char control) {
    frame[0] = FLAG;         // Start flag
    frame[1] = address;      // Address field
    frame[2] = control;      // Control field
    frame[3] = address ^ control; // BCC1 (A XOR C)
    frame[4] = FLAG;         // End flag
    return 5; // Length of the frame
}

// Create an Information (I) Frame with Byte Stuffing
int createInformationFrame(unsigned char* frame, unsigned char address, unsigned char control, const unsigned char* data, int dataLength) {
        if (dataLength > MAX_FRAME_SIZE - 6) { // Frame size check
        printf("Error: Data too large for frame\n");
        return -1;
    }
    int i;
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control; // BCC1 (A XOR C)

    // Copy data with byte stuffing
    int stuffedLength = 4;
    for (i = 0; i < dataLength; i++) {
        if (data[i] == FLAG || data[i] == ESC) {
            frame[stuffedLength++] = ESC;
            frame[stuffedLength++] = data[i] ^ 0x20; // Byte stuffing
        } else {
            frame[stuffedLength++] = data[i];
        }
    }

    // Calculate BCC2 (XOR of all data bytes before stuffing)
    unsigned char bcc2 = 0;
    for (i = 0; i < dataLength; i++) {
        bcc2 ^= data[i];
    }

    // Add BCC2 with byte stuffing
    if (bcc2 == FLAG || bcc2 == ESC) {
        frame[stuffedLength++] = ESC;
        frame[stuffedLength++] = bcc2 ^ 0x20;
    } else {
        frame[stuffedLength++] = bcc2;
    }

    frame[stuffedLength++] = FLAG; // End flag
    return stuffedLength;
}

// Validate a received frame
int validateFrame(const unsigned char* frame, int length) {
    if (frame[0] != FLAG || frame[length - 1] != FLAG) {
        printf("Frame validation failed: Invalid start or end FLAG\n");
        return -1; // Invalid start or end flag
    }
    
    // Validate BCC1 (A XOR C)
    if (frame[3] != (frame[1] ^ frame[2])) {
        printf("Frame validation failed: BCC1 mismatch\n");
        printf("Expected: 0x%02X, Actual: 0x%02X\n", frame[1] ^ frame[2], frame[3]);
        return -1; // BCC1 error
    }
    
    // If it's an Information frame, validate BCC2
    if (length > 5) {
        unsigned char bcc2 = 0;
        for (int i = 4; i < length - 2; i++) {
            bcc2 ^= frame[i];
        }
        if (bcc2 != frame[length - 2]) {
            printf("Frame validation failed: BCC2 mismatch\n");
            printf("Expected BCC2: 0x%02X, Actual BCC2: 0x%02X\n", bcc2, frame[length - 2]);
            return -1; // BCC2 error
        }
    }
    
    printf("Frame validation successful\n");
    return 0; // Frame is valid
}

void alarmHandler(int signal) {
    alarmOn = TRUE;
    alarmCount++;
    //timemouts
    printf("Alarm triggered %d\n", alarmCount);
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    serialFd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);

    if (serialFd < 0)
    {
        return -1;
    }

    // Define frame arrays
    unsigned char setFrame[5];
    unsigned char uaFrame[5];
    int retries = 0;
    currentLinkLayer = connectionParameters;

    // Create SET frame
    createSupervisionFrame(setFrame, A_SE, C_SET);

    if (connectionParameters.role == LlTx)
    {
        // Transmitter side
        (void) signal(SIGALRM, alarmHandler);

        while (retries < connectionParameters.nRetransmissions)
        {
            // Send SET frame
            frames_sent++;
            if (writeBytesSerialPort(setFrame, 5) < 0)
            {
                printf("Failed to send SET frame\n");
                return -1;
            }

            printf("SET frame sent, waiting for UA...\n");

            // Set alarm for the timeout duration
            alarmOn = 0; // Reset alarm flag
            alarm(timeout); // Set the alarm to trigger after the specified timeout

            alarmOn = 0;
            alarm(connectionParameters.timeout);

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
                frames_received++;
                printf("UA frame received, connection established.\n");
                time(&start_time);
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
                frames_received++;
                printf("SET frame received, sending UA...\n");

                // Create UA frame
                createSupervisionFrame(uaFrame, A_RE, C_UA);

                // Send UA frame
                frames_sent++;
                if (writeBytesSerialPort(uaFrame, 5) < 0)
                {
                    printf("Failed to send UA frame\n");
                    return -1;
                }

                printf("UA frame sent, connection established.\n");
                time(&start_time);
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
int llwrite(const unsigned char *buf, int bufSize) {
    static int Ns = 0; // Sequence number
    unsigned char frame[MAX_FRAME_SIZE];
    unsigned char rrFrame[5]; // For receiving the RR or REJ frame
    int retries = 0;
    int frameSize;
    int maxRetries = currentLinkLayer.nRetransmissions;
    int timeout = currentLinkLayer.timeout;
    State state;

    // Create the I-frame to send
    frameSize = createInformationFrame(frame, A_SE, Ns << 6, buf, bufSize);

    while (retries < maxRetries) {
        // Send the I-frame
        frames_sent++;
        if (writeBytesSerialPort(frame, frameSize) < 0) {
            printf("Failed to send I-frame\n");
            return -1;
        }
        printf("I-frame sent, waiting for RR/REJ (Attempt %d)\n", retries + 1);

        // Reset the alarm and start waiting for RR/REJ response
        alarmOn = FALSE;
        alarm(timeout); // Set the alarm

        // State machine for reading RR or REJ frame
        state = START;
        int bytesRead = 0;
        unsigned char byte;
        //int receivedResponse = 0;

        while (state != STOP_R && !alarmOn) {
            int res = readByteSerialPort(&byte);
            if (res > 0) {
                rrFrame[bytesRead++] = byte;

                // State machine transitions for RR or REJ
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_RE) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) {
                            state = C_RCV;
                        } else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_RE ^ rrFrame[2])) state = BCC1_OK;
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
        }

        alarm(0); // Disable alarm to avoid premature timeout handling

        if (state == STOP_R && validateFrame(rrFrame, 5) == 0) {
            if (rrFrame[2] == C_RR0 || rrFrame[2] == C_RR1) {
                frames_received++;
                printf("Received RR frame, data acknowledged\n");
                Ns = (Ns + 1) % 2; // Toggle sequence number
                return bufSize;
            } else if (rrFrame[2] == C_REJ0 || rrFrame[2] == C_REJ1) {
                frames_received++;
                printf("Received REJ frame, retransmitting...\n");
                maxRetries++;  // Increase retry counter for REJ retransmissions
            }
        } else {
            // If timeout or no valid RR/REJ received
            printf("Timeout or invalid frame received, retrying transmission\n");
            maxRetries++;
        }
    }

    printf("Failed to send data after %d attempts\n", retries);
    return -1;
}




int byteUnstuffing(const unsigned char *input, int length, unsigned char *output) {
    int j = 0;
    for (int i = 0; i < length; i++) {
        if (input[i] == ESC) { // Escape character
            i++; // Skip next byte for unstuffing
            output[j++] = input[i] ^ 0x20; // Unstuff by XORing with 0x20
        } else {
            output[j++] = input[i];
        }
    }
    return j; // Return the new length after unstuffing
}
////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    State state = START;
    unsigned char byte;
    unsigned char frame[MAX_FRAME_SIZE];
    unsigned char unstuffedFrame[MAX_FRAME_SIZE];
    int bytesRead = 0;
    int dataIndex = 4; // Start placing data after the header fields
    static int expectedNs = 0; // Expected sequence number (0 or 1)

    // State machine to read an I-frame
    while (1) {
        int res = readByteSerialPort(&byte);
        if (res > 0) {
            if (bytesRead >= MAX_FRAME_SIZE) {
                printf("Error: Frame buffer overflow. Resetting buffer and awaiting retransmission.\n");
                bytesRead = 0;
                state = START;
                dataIndex = 4;

                // Clear the input buffer to remove any lingering bytes
                tcflush(serialFd, TCIFLUSH); // This will need to reference your serial port descriptor
                continue;
            }

            frame[bytesRead++] = byte;

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == A_SE) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;

                case A_RCV:
                    if ((byte == (expectedNs << 6)) || (byte == ((1 - expectedNs) << 6))) {
                        state = C_RCV;
                    } else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case C_RCV:
                    if (byte == (A_SE ^ frame[2])) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case BCC1_OK:
                    if (byte != FLAG) {
                        state = DATA_RCV;
                        frame[dataIndex++] = byte;
                    } else {
                        state = STOP_R;
                    }
                    break;

                case DATA_RCV:
                    if (byte == FLAG) {
                        state = STOP_R;  // End of frame
                    } else {
                        frame[dataIndex++] = byte;
                    }
                    break;

                default:
                    break;
            }
        }

        // Once STOP_R is reached, unstuff and validate
        if (state == STOP_R) {
            int unstuffedLength = byteUnstuffing(frame, bytesRead, unstuffedFrame);

            // Validate the frame
            if (validateFrame(unstuffedFrame, unstuffedLength) != 0) {
                frames_received++;
                printf("Invalid I-frame received, sending REJ\n");
                unsigned char rejFrame[5];
                createSupervisionFrame(rejFrame, A_RE, (expectedNs == 0) ? C_REJ0 : C_REJ1);
                frames_sent++;
                writeBytesSerialPort(rejFrame, 5);

                // Clear input buffer to ensure fresh data on retry
                tcflush(serialFd, TCIFLUSH); // Adjust to match your serial port descriptor

                // Reset buffer and state for retransmission
                bytesRead = 0;
                state = START;
                dataIndex = 4;
                continue; // Wait for the retransmitted frame
            }
            frames_received++;
            // Send RR if validation is successful
            unsigned char rrFrame[5];
            createSupervisionFrame(rrFrame, A_RE, (expectedNs == 0) ? C_RR0 : C_RR1);
            frames_sent++;
            if (writeBytesSerialPort(rrFrame, 5) < 0) {
                printf("Failed to send RR frame\n");
                return -1;
            }
            printf("RR frame sent, acknowledgment complete.\n");

            // Toggle expected sequence number
            expectedNs = 1 - expectedNs;

            if (unstuffedLength - 6 <= 0) {
                printf("Error: No valid data in the frame.\n");
                return -1;
            }

            // Extract data and store it in packet (excluding header and BCC)
            memcpy(packet, &unstuffedFrame[4], unstuffedLength - 6);

            return unstuffedLength - 6;  // Return the number of bytes read
        }
    }
    return -1;
}








////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // Initialize variables
    State state = START;
    unsigned char byte;
    unsigned char frame[5];
    int retries = 0;
    int maxRetries = currentLinkLayer.nRetransmissions;
    int timeout = currentLinkLayer.timeout;
    int role = currentLinkLayer.role; // LlTx or LlRx

    // Set the alarm handler
    (void) signal(SIGALRM, alarmHandler);

    printf("Starting llclose, role: %s\n", (role == LlTx) ? "LlTx" : "LlRx");

    while (retries < maxRetries && state != STOP_R)
    {
        if (role == LlTx)
        {
            // Transmitter: send DISC if in START state
            if (state == START)
            {
                // Create and send DISC frame
                createSupervisionFrame(frame, A_SE, C_DISC);
                frames_sent++;
                if (writeBytesSerialPort(frame, 5) < 0)
                {
                    printf("Failed to send DISC frame\n");
                    return -1;
                }
                printf("Transmitter: DISC frame sent, waiting for DISC response...\n");
            }
        }
        else if (role == LlRx && state == START)
        {
            printf("Receiver: Waiting for DISC frame from transmitter...\n");
        }

        // Set the alarm for timeout duration
        alarmOn = FALSE;
        alarm(timeout);

        // State machine for handling communication
        while (!alarmOn && state != STOP_R)
        {
            if (readByteSerialPort(&byte) > 0)
            {
                // State machine transitions
                switch (state)
                {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if ((role == LlTx && byte == A_RE) || (role == LlRx && byte == A_SE))
                            state = A_RCV;
                        else if (byte != FLAG)
                            state = START;
                        break;
                    case A_RCV:
                        if (byte == C_DISC)
                            state = C_RCV;
                        else if (byte == FLAG)
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;
                    case C_RCV:
                        if (byte == ((role == LlTx ? A_RE : A_SE) ^ C_DISC))
                            state = BCC1_OK;
                        else if (byte == FLAG)
                            state = FLAG_RCV;
                        else
                            state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG)
                            state = STOP_R;
                        else
                            state = START;
                        break;
                    default:
                        break;
                }
            }
        }

        // Disable the alarm
        alarm(0);

        // Retry if state is not STOP_R
        if (state != STOP_R)
        {
            printf("Timeout occurred, retrying (%d/%d)\n", retries + 1, maxRetries);
            retries++;
        }
    }

    // Check if we reached STOP_R
    if (state != STOP_R)
    {
        printf("Failed to complete DISC exchange, connection not closed properly.\n");
        return -1;
    }
    frames_received++;

    if (role == LlTx)
    {
        // Transmitter: Send UA frame to acknowledge disconnection
        createSupervisionFrame(frame, A_SE, C_UA);
        frames_sent++;
        if (writeBytesSerialPort(frame, 5) < 0)
        {
            printf("Failed to send UA frame\n");
            return -1;
        }
        printf("Transmitter: UA frame sent, connection closed.\n");
    }
    else if (role == LlRx)
    {
        // Receiver: Send DISC and wait for UA
        createSupervisionFrame(frame, A_RE, C_DISC);
        frames_sent++;
        if (writeBytesSerialPort(frame, 5) < 0)
        {
            printf("Failed to send DISC frame\n");
            return -1;
        }
        printf("Receiver: DISC frame sent back, waiting for UA...\n");

        // Wait for UA from the transmitter
        state = START;
        retries = 0;
        while (retries < maxRetries && state != STOP_R)
        {
            alarmOn = FALSE;
            alarm(timeout);

            while (!alarmOn && state != STOP_R)
            {
                if (readByteSerialPort(&byte) > 0)
                {
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
                            if (byte == C_UA) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (A_SE ^ C_UA)) state = BCC1_OK;
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
            }

            // Disable the alarm
            alarm(0);

            // Retry if state is not STOP_R
            if (state != STOP_R)
            {
                printf("Timeout occurred, retrying to wait for UA (%d/%d)\n", retries + 1, maxRetries);
                retries++;
            }
        }

        if (state != STOP_R)
        {
            printf("Failed to receive final UA, connection not closed properly.\n");
            return -1;
        }
        frames_received++;
        printf("Receiver: UA frame received, connection closed.\n");
    }

    // Optionally, show statistics if enabled
    if (showStatistics)
    {
        time(&end_time);
        double total_time = end_time - start_time;
        printf("Connection statistics:\n");
        printf("Total transference time elapsed: %.2f seconds\n", total_time);
        printf("Number of retransmissions: %d\n", retries);
        printf("Number of timeouts: %d\n", alarmCount);
        printf("Frames sent: %d\n", frames_sent);
        printf("Frames received: %d\n", frames_received);
    }

    // Close the serial port
    return closeSerialPort();
}
