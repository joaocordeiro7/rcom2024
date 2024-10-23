// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Set up the LinkLayer configuration
    LinkLayer connectionParameters;
    strncpy(connectionParameters.serialPort, serialPort, sizeof(connectionParameters.serialPort));
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    // Set the role based on the "role" parameter
    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
    } else if (strcmp(role, "rx") == 0) {
        connectionParameters.role = LlRx;
    } else {
        printf("Invalid role: %s\n", role);
        return;
    }

    // Test the llopen function
    printf("Testing llopen...\n");
    if (llopen(connectionParameters) < 0) {
        printf("Failed to open the connection using llopen\n");
        printf("transmissions: %d\n", connectionParameters.nRetransmissions);
        return;
    }
    printf("Connection established using llopen\n");

    // Temporary test string for transmission
    const char *testString = "This is a test string for transmission.";
    int testStringLength = strlen(testString) + 1; // Include null terminator

    // Transmitter (Tx) role: Send test string using llwrite
    if (connectionParameters.role == LlTx) {
        printf("Sending test string using llwrite...\n");

        if (llwrite((unsigned char *)testString, testStringLength) < 0) {
            printf("Failed to send test string using llwrite\n");
            llclose(0);
            return;
        }
        printf("Test string sent successfully\n");
    }

    // Receiver (Rx) role: Receive data using llread
    if (connectionParameters.role == LlRx) {
        unsigned char buffer[256]; // Buffer to store the received data
        int bytesRead;

        printf("Receiving data using llread...\n");

        // Read the incoming data (expecting the test string)
        bytesRead = llread(buffer);
        if (bytesRead > 0) {
            printf("Received %d bytes: %s\n", bytesRead, buffer);
        } else {
            printf("Failed to receive data using llread\n");
        }
    }

    // Now close the connection
    printf("Testing llclose...\n");
    if (llclose(0) < 0) {
        printf("Failed to close the connection using llclose\n");
        return;
    }
    printf("Connection closed successfully using llclose\n");
}
