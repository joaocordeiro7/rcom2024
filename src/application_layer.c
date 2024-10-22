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
 // Transmitter (Tx) role: Send data using llwrite
    if (connectionParameters.role == LlTx) {
        // Open the file to send
        FILE *file = fopen(filename, "rb");
        if (!file) {
            printf("Failed to open file: %s\n", filename);
            llclose(0);  // Close the connection
            return;
        }

        // Read the file and send its content in chunks
        unsigned char buffer[256];  // Buffer to hold data
        int bytesRead;

        printf("Sending data using llwrite...\n");

        while ((bytesRead = fread(buffer, 1, sizeof(buffer), file)) > 0) {
            if (llwrite(buffer, bytesRead) < 0) {
                printf("Failed to send data using llwrite\n");
                fclose(file);
                llclose(0);
                return;
            }
            printf("Sent %d bytes\n", bytesRead);
        }

        // Close the file
        fclose(file);
        printf("File transmission complete\n");
    }

    // Receiver (Rx) role: Receive data using llread
    if (connectionParameters.role == LlRx) {
        // Open a file to store the received data
        FILE *file = fopen(filename, "wb");
        if (!file) {
            printf("Failed to open file for writing: %s\n", filename);
            llclose(0);  // Close the connection
            return;
        }

        // Buffer to store the received data
        unsigned char buffer[256];
        int bytesRead;

        printf("Receiving data using llread...\n");

        // Continuously read data until the transmission is complete
        while ((bytesRead = llread(buffer)) > 0) {
            fwrite(buffer, 1, bytesRead, file);
            printf("Received %d bytes\n", bytesRead);
        }

        // Close the file after all data is received
        fclose(file);
        printf("File reception complete\n");
    }

    // Now close the connection
    printf("Testing llclose...\n");
    if (llclose(0) < 0) {
        printf("Failed to close the connection using llclose\n");
        return;
    }
    printf("Connection closed successfully using llclose\n");
}