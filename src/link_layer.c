// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>


// frame delimiters and constants
#define FLAG 0x7E
#define A_SE 0x03
#define A_RE 0x01

// control field values
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define ESC 0x7D

// maximum frame size
#define MAX_FRAME_SIZE 1024

// MISC
#define _POSIX_SOURCE 1

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

// create a supervision (S) or unnumbered (U) frame
int createSupervisionFrame(unsigned char* frame, unsigned char address, unsigned char control) {
    frame[0] = FLAG;         
    frame[1] = address;      
    frame[2] = control;      
    frame[3] = address ^ control; 
    frame[4] = FLAG;         
    return 5; 
}

// create an information (I) frame with byte stuffing
int createInformationFrame(unsigned char* frame, unsigned char address, unsigned char control, const unsigned char* data, int dataLength) {
        if (dataLength > MAX_FRAME_SIZE - 6) { 
        printf("Error: Data too large for frame\n");
        return -1;
    }
    int i;
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;         //BCC1

    int stuffedLength = 4;
    for (i = 0; i < dataLength; i++) {
        if (data[i] == FLAG || data[i] == ESC) {             //byte stuffing
            frame[stuffedLength++] = ESC;
            frame[stuffedLength++] = data[i] ^ 0x20;    
        } else {
            frame[stuffedLength++] = data[i];
        }
    }

    unsigned char bcc2 = 0;
    for (i = 0; i < dataLength; i++) {         //calculate BCC2 before stuffing
        bcc2 ^= data[i];
    }

    if (bcc2 == FLAG || bcc2 == ESC) {
        frame[stuffedLength++] = ESC;
        frame[stuffedLength++] = bcc2 ^ 0x20;            //BCC2 with byte stuffing
    } else {
        frame[stuffedLength++] = bcc2;
    }

    frame[stuffedLength++] = FLAG; 
    return stuffedLength;
}

// validate a received frame
int validateFrame(const unsigned char* frame, int length) {
    if (frame[0] != FLAG || frame[length - 1] != FLAG) {
        printf("Frame validation failed: Invalid start or end FLAG\n");
        return -1; 
    }
    
    if (frame[3] != (frame[1] ^ frame[2])) {                    //BCC1 validation
        printf("Frame validation failed: BCC1 mismatch\n");
        printf("Expected: 0x%02X, Actual: 0x%02X\n", frame[1] ^ frame[2], frame[3]);
        return -1; 
    }
    
    if (length > 5) {
        unsigned char bcc2 = 0;
        for (int i = 4; i < length - 2; i++) {      //if I-Frame, validate BCC2
            bcc2 ^= frame[i];
        }
        if (bcc2 != frame[length - 2]) {
            printf("Frame validation failed: BCC2 mismatch\n");
            printf("Expected BCC2: 0x%02X, Actual BCC2: 0x%02X\n", bcc2, frame[length - 2]);
            return -1; 
        }
    }
    
    printf("Frame validation successful\n");
    return 0; 
}

void alarmHandler(int signal) {
    alarmOn = TRUE;
    alarmCount++;
    //timeouts
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

    unsigned char setFrame[5];
    unsigned char uaFrame[5];
    int retries = 0;
    currentLinkLayer = connectionParameters;

    createSupervisionFrame(setFrame, A_SE, C_SET);

    if (connectionParameters.role == LlTx)  // Transmitter side
    {
        (void) signal(SIGALRM, alarmHandler);

        while (retries < connectionParameters.nRetransmissions)
        {
            frames_sent++;
            if (writeBytesSerialPort(setFrame, 5) < 0)
            {
                printf("Failed to send SET frame\n");
                return -1;
            }

            printf("SET frame sent, waiting for UA...\n");

            alarmOn = 0;
            alarm(connectionParameters.timeout);

            //receiving UA frame
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
                    // no byte received -> wait until the alarm goes off
                }
            }

            alarm(0);

            if (state == STOP_R && validateFrame(uaFrame, 5) == 0)
            {
                // valid UA
                frames_received++;
                printf("UA frame received, connection established.\n");
                time(&start_time);
                return 1;
            }
            else
            {
                printf("Invalid UA frame received or timeout occurred.\n");
            }

            // no UA received or invalid UA -> retry
            printf("Retry %d: Resending SET frame...\n", retries + 1);
            retries++;
        }

        printf("Connection failed after %d attempts\n", retries);
        return -1;
    }
    else if (connectionParameters.role == LlRx) //receiver side
    {
        printf("Waiting for SET frame...\n");

        // receiving SET frame
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
                // no byte received -> wait 
            }

            if (state == STOP_R && validateFrame(setFrame, 5) == 0)
            {
                // valid SET
                frames_received++;
                printf("SET frame received, sending UA...\n");

                // UA frame
                createSupervisionFrame(uaFrame, A_RE, C_UA);

                // Send UA 
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
    static int Ns = 0; 
    unsigned char frame[MAX_FRAME_SIZE];
    unsigned char rrFrame[5]; 
    int retries = 0;
    int frameSize;
    int maxRetries = currentLinkLayer.nRetransmissions;
    int timeout = currentLinkLayer.timeout;
    State state;

    // create I-frame 
    frameSize = createInformationFrame(frame, A_SE, Ns << 6, buf, bufSize);

    while (retries < maxRetries) {
        // send I-frame
        frames_sent++;
        if (writeBytesSerialPort(frame, frameSize) < 0) {
            printf("Failed to send I-frame\n");
            return -1;
        }
        printf("I-frame sent, waiting for RR/REJ (Attempt %d)\n", retries + 1);

        alarmOn = FALSE;
        alarm(timeout);

        // reading RR or REJ frame
        state = START;
        int bytesRead = 0;
        unsigned char byte;

        while (state != STOP_R && !alarmOn) {
            int res = readByteSerialPort(&byte);
            if (res > 0) {
                rrFrame[bytesRead++] = byte;

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

        alarm(0); 

        if (state == STOP_R && validateFrame(rrFrame, 5) == 0) {
            if (rrFrame[2] == C_RR0 || rrFrame[2] == C_RR1) {
                frames_received++;
                printf("Received RR frame, data acknowledged\n");
                Ns = (Ns + 1) % 2; 
                return bufSize;
            } else if (rrFrame[2] == C_REJ0 || rrFrame[2] == C_REJ1) {
                frames_received++;
                printf("Received REJ frame, retransmitting...\n");
                retries = 0;  
            }
        } else {
            printf("Timeout or invalid frame received, retrying transmission\n");
            retries++;
        }
    }

    printf("Failed to send data after %d attempts\n", retries);
    return -1;
}




int byteUnstuffing(const unsigned char *input, int length, unsigned char *output) {
    int j = 0;
    for (int i = 0; i < length; i++) {
        if (input[i] == ESC) { 
            i++; 
            output[j++] = input[i] ^ 0x20; 
        } else {
            output[j++] = input[i];
        }
    }
    return j; 
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
    int dataIndex = 4; 
    static int expectedNs = 0; 

    //read an I-frame
    while (1) {
        int res = readByteSerialPort(&byte);
        if (res > 0) {
            if (bytesRead >= MAX_FRAME_SIZE) {
                printf("Error: Frame buffer overflow. Resetting buffer and awaiting retransmission.\n");
                bytesRead = 0;
                state = START;
                dataIndex = 4;
                continue;
            }

            frame[bytesRead++] = byte;

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    else bytesRead = 0;
                    break;
                case FLAG_RCV:
                    if (byte == A_SE) state = A_RCV;
                    else if (byte != FLAG) {
                        state = START;
                        bytesRead = 0;
                    }
                    else bytesRead = 1;
                    break;
                case A_RCV:
                    if ((byte == (expectedNs << 6)) || (byte == ((1 - expectedNs) << 6))) {
                        state = C_RCV;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV;
                        bytesRead = 1;
                    }
                    else {
                        state = START;
                        bytesRead = 0;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_SE ^ frame[2])) state = BCC1_OK;
                    else if (byte == FLAG) {
                        state = FLAG_RCV;
                        bytesRead = 1;
                    }
                    else {
                        state = START;
                        bytesRead = 0;
                    }
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
                        state = STOP_R; 
                    } else {
                        frame[dataIndex++] = byte;
                    }
                    break;
                default:
                    break;
            }
        }

        // unstuff and validate
        if (state == STOP_R) {
            int unstuffedLength = byteUnstuffing(frame, bytesRead, unstuffedFrame);

            if (validateFrame(unstuffedFrame, unstuffedLength) != 0) {
                frames_received++;
                printf("Invalid I-frame received, sending REJ\n");
                unsigned char rejFrame[5];
                createSupervisionFrame(rejFrame, A_RE, (expectedNs == 0) ? C_REJ0 : C_REJ1);
                frames_sent++;
                writeBytesSerialPort(rejFrame, 5);

                bytesRead = 0;
                state = START;
                dataIndex = 4;
                continue; 
            }
            frames_received++;
            // send RR 
            unsigned char rrFrame[5];
            createSupervisionFrame(rrFrame, A_RE, (expectedNs == 0) ? C_RR0 : C_RR1);
            frames_sent++;
            if (writeBytesSerialPort(rrFrame, 5) < 0) {
                printf("Failed to send RR frame\n");
                return -1;
            }
            printf("RR frame sent, acknowledgment complete.\n");

            expectedNs = 1 - expectedNs;

            if (unstuffedLength - 6 <= 0) {
                printf("Error: No valid data in the frame.\n");
                return -1;
            }

            // extract data and store in packet 
            memcpy(packet, &unstuffedFrame[4], unstuffedLength - 6);

            return unstuffedLength - 6; 
        }
    }
    return -1;
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    State state = START;
    unsigned char byte;
    unsigned char frame[5];
    int retries = 0;
    int maxRetries = currentLinkLayer.nRetransmissions;
    int timeout = currentLinkLayer.timeout;
    int role = currentLinkLayer.role;

    (void) signal(SIGALRM, alarmHandler);

    printf("Starting llclose, role: %s\n", (role == LlTx) ? "LlTx" : "LlRx");

    while (retries < maxRetries && state != STOP_R)
    {
        if (role == LlTx)   //Transmitter
        {
            //send DISC
            if (state == START)
            {
                //create and send DISC 
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

        alarm(0);

        if (state != STOP_R)
        {
            printf("Timeout occurred, retrying (%d/%d)\n", retries + 1, maxRetries);
            retries++;
        }
    }

    if (state != STOP_R)
    {
        printf("Failed to complete DISC exchange, connection not closed properly.\n");
        return -1;
    }
    frames_received++;

    if (role == LlTx)   //Transmitter
    {
        // send UA
        createSupervisionFrame(frame, A_SE, C_UA);
        frames_sent++;
        if (writeBytesSerialPort(frame, 5) < 0)
        {
            printf("Failed to send UA frame\n");
            return -1;
        }
        printf("Transmitter: UA frame sent, connection closed.\n");

        sleep(1);
    }
    else if (role == LlRx)   //Receiver
    {
        // send DISC and wait for UA
        createSupervisionFrame(frame, A_RE, C_DISC);
        frames_sent++;
        if (writeBytesSerialPort(frame, 5) < 0)
        {
            printf("Failed to send DISC frame\n");
            return -1;
        }
        printf("Receiver: DISC frame sent back, waiting for UA...\n");

        // wait for UA
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

            alarm(0);

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

    return closeSerialPort();
}
