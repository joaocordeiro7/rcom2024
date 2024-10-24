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


// Create a Supervision (S) or Unnumbered (U) Frame
int createSupervisionFrame(unsigned char* frame, unsigned char address, unsigned char control) {
    frame[0] = FLAG;       
    frame[1] = address;     
    frame[2] = control;      
    frame[3] = address ^ control; 
    frame[4] = FLAG;         
    return 5; 
}

// Create an Information (I) Frame with Byte Stuffing
int createInformationFrame(unsigned char* frame, unsigned char address, unsigned char control, const unsigned char* data, int dataLength) {
        if (dataLength > MAX_FRAME_SIZE - 6) {ck
        printf("Error: Data too large for frame\n");
        return -1;
    }
    int i;
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control; 

  
    int stuffedLength = 4;
    for (i = 0; i < dataLength; i++) {
        if (data[i] == FLAG || data[i] == ESC) {
            frame[stuffedLength++] = ESC;
            frame[stuffedLength++] = data[i] ^ 0x20; 
        } else {
            frame[stuffedLength++] = data[i];
        }
    }

)
    unsigned char bcc2 = 0;
    for (i = 0; i < dataLength; i++) {
        bcc2 ^= data[i];
    }


    if (bcc2 == FLAG || bcc2 == ESC) {
        frame[stuffedLength++] = ESC;
        frame[stuffedLength++] = bcc2 ^ 0x20;
    } else {
        frame[stuffedLength++] = bcc2;
    }

    frame[stuffedLength++] = FLAG; 
    return stuffedLength;
}


int validateFrame(const unsigned char* frame, int length) {
    if (frame[0] != FLAG || frame[length - 1] != FLAG) {
        printf("Frame validation failed: Invalid start or end FLAG\n");
        return -1; 
    }
    

    if (frame[3] != (frame[1] ^ frame[2])) {
        printf("Frame validation failed: BCC1 mismatch\n");
        printf("Expected: 0x%02X, Actual: 0x%02X\n", frame[1] ^ frame[2], frame[3]);
        return -1; // BCC1 error
    }
    

    if (length > 5) {
        unsigned char bcc2 = 0;
        for (int i = 4; i < length - 2; i++) {
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
    //timemouts
    printf("Alarm triggered %d\n", alarmCount);
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
    currentLinkLayer = connectionParameters;

    // Create SET frame
    createSupervisionFrame(setFrame, A_SE, C_SET);

    if (connectionParameters.role == LlTx)
    {

        (void) signal(SIGALRM, alarmHandler);

        while (retries < connectionParameters.nRetransmissions)
        {

            if (writeBytesSerialPort(setFrame, 5) < 0)
            {
                printf("Failed to send SET frame\n");
                return -1;
            }

            printf("SET frame sent, waiting for UA...\n");

            alarmOn = 0; 
            alarm(timeout);

            alarmOn = 0;
            alarm(connectionParameters.timeout);

       
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

  
            alarm(0);

            if (state == STOP_R && validateFrame(uaFrame, 5) == 0)
            {
     
                printf("UA frame received, connection established.\n");
                return 1;
            }
            else
            {
                printf("Invalid UA frame received or timeout occurred.\n");
            }

            printf("Retry %d: Resending SET frame...\n", retries + 1);
            retries++;
        }

        printf("Connection failed after %d attempts\n", retries);
        return -1;
    }
    else if (connectionParameters.role == LlRx)
    {
        printf("Waiting for SET frame...\n");

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
                // if no byte is received, the loop continues to wait for the next bytE and the alarm mechanism ensures it doesn't wait indefinitely
            }

            if (state == STOP_R && validateFrame(setFrame, 5) == 0)
            {
            
                printf("SET frame received, sending UA...\n");

                createSupervisionFrame(uaFrame, A_RE, C_UA);

         
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
int llwrite(const unsigned char *buf, int bufSize) {
    static int Ns = 0; 
    unsigned char frame[MAX_FRAME_SIZE];
    unsigned char rrFrame[5]; 
    int retries = 0;
    int frameSize;
    int maxRetries = currentLinkLayer.nRetransmissions;
    int timeout = currentLinkLayer.timeout;
    State state;

    
    frameSize = createInformationFrame(frame, A_SE, Ns << 6, buf, bufSize);

    while (retries < maxRetries) {
        // Send the I-frame
        if (writeBytesSerialPort(frame, frameSize) < 0) {
            printf("Failed to send I-frame\n");
            return -1;
        }
        printf("I-frame sent, waiting for RR/REJ (Attempt %d)\n", retries + 1);

  
        alarmOn = FALSE;
        alarm(timeout);

   
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
                        if (byte == A_RE) state = A_RCV; ss
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1)
                            state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
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
                printf("Received RR frame, data acknowledged\n");
                Ns = (Ns + 1) % 2;  
                return bufSize; 
            } else if (rrFrame[2] == C_REJ0 || rrFrame[2] == C_REJ1) {
                printf("Received REJ frame, retransmitting...\n");
                retries++;
            }
        } else {
            printf("Timeout or invalid frame received, retransmitting (Attempt %d)\n", retries + 1);
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


    while (state != STOP_R) {
        int res = readByteSerialPort(&byte);
        if (res > 0) {
            if (bytesRead >= MAX_FRAME_SIZE) {
                printf("Error: Frame buffer overflow\n");
                return -1;
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
                        state = STOP_R; 
                    } else {
                        frame[dataIndex++] = byte;
                    }
                    break;

                default:
                    break;
            }
        }
    }



    int unstuffedLength = byteUnstuffing(frame, bytesRead, unstuffedFrame);


    if (validateFrame(unstuffedFrame, unstuffedLength) != 0) {
        printf("Invalid I-frame received, sending REJ\n");
        unsigned char rejFrame[5];
        createSupervisionFrame(rejFrame, A_RE, (expectedNs == 0) ? C_REJ0 : C_REJ1);
        writeBytesSerialPort(rejFrame, 5); 
        return -1;
    }


    unsigned char rrFrame[5];
    createSupervisionFrame(rrFrame, A_RE, (expectedNs == 0) ? C_RR0 : C_RR1);
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


    memcpy(packet, &unstuffedFrame[4], unstuffedLength - 6);  

    return unstuffedLength - 6;  
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
    int role = currentLinkLayer.role; 


    (void) signal(SIGALRM, alarmHandler);

    printf("Starting llclose, role: %s\n", (role == LlTx) ? "LlTx" : "LlRx");

    while (retries < maxRetries && state != STOP_R)
    {
        if (role == LlTx)
        {

            if (state == START)
            {
                // Create and send DISC frame
                createSupervisionFrame(frame, A_SE, C_DISC);
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

    if (role == LlTx)
    {
        createSupervisionFrame(frame, A_SE, C_UA);
        if (writeBytesSerialPort(frame, 5) < 0)
        {
            printf("Failed to send UA frame\n");
            return -1;
        }
        printf("Transmitter: UA frame sent, connection closed.\n");
    }
    else if (role == LlRx)
    {
        createSupervisionFrame(frame, A_RE, C_DISC);
        if (writeBytesSerialPort(frame, 5) < 0)
        {
            printf("Failed to send DISC frame\n");
            return -1;
        }
        printf("Receiver: DISC frame sent back, waiting for UA...\n");

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

        printf("Receiver: UA frame received, connection closed.\n");
    }

    if (showStatistics)
    {
        printf("Connection statistics:\n");
        printf("Number of retransmissions: %d\n", retries);
    }

    return closeSerialPort();
}
