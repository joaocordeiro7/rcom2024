// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

#define MAX_FRAME_SIZE 1024


int createControlPacket(unsigned char *packet, const char *filename, int fileSize, int isStart) {
    int index = 0;
    packet[index++] = isStart ? 0x02 : 0x03; // 0x02 for START, 0x03 for END

    // File size
    packet[index++] = 0x00; // Indicating that this is a file size field
    packet[index++] = sizeof(fileSize); // Length of the file size field
    memcpy(&packet[index], &fileSize, sizeof(fileSize));
    index += sizeof(fileSize);

    // Filename
    int filenameLength = strlen(filename);
    packet[index++] = 0x01; // Indicating that this is a filename field
    packet[index++] = filenameLength; // Length of the filename field
    memcpy(&packet[index], filename, filenameLength);
    index += filenameLength;

    return index; // Total packet length
}


int createDataPacket(unsigned char *packet, int sequenceNumber, const unsigned char *data, int dataLength) {
    int index = 0;
    packet[index++] = 0x01; // DATA packet type

    packet[index++] = sequenceNumber; // Sequence number

    // Data length
    packet[index++] = (dataLength >> 8) & 0xFF; // High byte of length
    packet[index++] = dataLength & 0xFF;        // Low byte of length

    // Copy the data itself
    memcpy(&packet[index], data, dataLength);
    index += dataLength;

    return index; // Total packet length
}



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
        return;
    }
    printf("Connection established using llopen\n");

    if (connectionParameters.role == LlTx) {
        // Transmitter (Tx) role: Send file data
        FILE *file = fopen(filename, "rb");
        if (!file) {
            printf("Failed to open file: %s\n", filename);
            llclose(0);
            return;
        }

        // Prepare the START control packet
        unsigned char controlPacket[MAX_FRAME_SIZE];
        int fileSize;
        fseek(file, 0, SEEK_END);
        fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);
        int controlPacketLength = createControlPacket(controlPacket, filename, fileSize, 1);

        // Send the START control packet
        if (llwrite(controlPacket, controlPacketLength) < 0) {
            printf("Failed to send START control packet\n");
            fclose(file);
            llclose(0);
            return;
        }
        printf("START control packet sent\n");

        // Send the file data in DATA packets
        unsigned char dataBuffer[256];
        int bytesRead;
        int sequenceNumber = 0;

        printf("Sending file data using DATA packets...\n");
        while ((bytesRead = fread(dataBuffer, 1, sizeof(dataBuffer), file)) > 0) {
            // Create the DATA packet
            unsigned char dataPacket[MAX_FRAME_SIZE];
            int dataPacketLength = createDataPacket(dataPacket, sequenceNumber, dataBuffer, bytesRead);

            // Send the DATA packet
            if (llwrite(dataPacket, dataPacketLength) < 0) {
                printf("Failed to send DATA packet\n");
                fclose(file);
                llclose(0);
                return;
            }
            printf("Sent DATA packet with sequence number: %d, size: %d bytes\n", sequenceNumber, bytesRead);

            // Toggle sequence number between 0 and 1
            sequenceNumber = (sequenceNumber + 1) % 2;
        }

        // Send the END control packet
        controlPacketLength = createControlPacket(controlPacket, filename, fileSize, 0);
        if (llwrite(controlPacket, controlPacketLength) < 0) {
            printf("Failed to send END control packet\n");
            fclose(file);
            llclose(0);
            return;
        }
        printf("END control packet sent\n");

        // Close the file
        fclose(file);
        printf("File transmission complete\n");
    }

    if (connectionParameters.role == LlRx) {
        // Receiver (Rx) role: Receive file data
        FILE *file = fopen(filename, "wb");
        if (!file) {
            printf("Failed to open file for writing: %s\n", filename);
            llclose(0);
            return;
        }

        unsigned char buffer[MAX_FRAME_SIZE];
        int bytesRead;
        int expectedSequenceNumber = 0;
        int receivingFile = 0;

        printf("Receiving data...\n");

        while ((bytesRead = llread(buffer)) > 0) {
            unsigned char packetType = buffer[0];

            if (packetType == 0x02) {
                printf("Received START control packet\n");
                receivingFile = 1;
            } else if (packetType == 0x03) {
                printf("Received END control packet\n");
                receivingFile = 0;
                break;
            } else if (packetType == 0x01 && receivingFile) {
                unsigned char sequenceNumber = buffer[1];
                int dataLength = (buffer[2] << 8) | buffer[3];

                if (sequenceNumber == expectedSequenceNumber) {
                    fwrite(&buffer[4], 1, dataLength, file);
                    printf("Received DATA packet with sequence number: %d, size: %d bytes\n", sequenceNumber, dataLength);
                    expectedSequenceNumber = (expectedSequenceNumber + 1) % 2;
                } else {
                    printf("Unexpected sequence number: %d (expected %d)\n", sequenceNumber, expectedSequenceNumber);
                }
            }
        }

        fclose(file);
        printf("File reception complete\n");
    }

    // Close the connection
    printf("Testing llclose...\n");
    if (llclose(0) < 0) {
        printf("Failed to close the connection using llclose\n");
        return;
    }
    printf("Connection closed successfully using llclose\n");
}




