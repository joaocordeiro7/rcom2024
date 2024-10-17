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

    // Now close the connection
    printf("Testing llclose...\n");
    if (llclose(0) < 0) {
        printf("Failed to close the connection using llclose\n");
        return;
    }
    printf("Connection closed successfully using llclose\n");
}