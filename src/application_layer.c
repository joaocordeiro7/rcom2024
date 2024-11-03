// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

#define MAX_FRAME_SIZE 1024

#define START 0x02
#define END 0x03

int statisticsEnabled = 1;

int createControlPacket(unsigned char *packet, const char *filename, int fileSize, int isStart) {
    int index = 0;
    packet[index++] = isStart ? START : END; // START or END control packet

    //file size
    packet[index++] = 0x00; //file size field
    packet[index++] = sizeof(fileSize); 
    memcpy(&packet[index], &fileSize, sizeof(fileSize));
    index += sizeof(fileSize);

    //filename
    int filenameLength = strlen(filename);
    packet[index++] = 0x01; // filename field
    packet[index++] = filenameLength; 
    memcpy(&packet[index], filename, filenameLength);
    index += filenameLength;

    return index;
}


int createDataPacket(unsigned char *packet, int sequenceNumber, const unsigned char *data, int dataLength) {
    int index = 0;
    packet[index++] = 0x01; // DATA packet type

    packet[index++] = sequenceNumber; // sequence number

    // data length
    packet[index++] = (dataLength >> 8) & 0xFF; // high byte 
    packet[index++] = dataLength & 0xFF;        // low byte 

    // copy the data 
    memcpy(&packet[index], data, dataLength);
    index += dataLength;

    return index; 
}



void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // setup linklayer
    LinkLayer connectionParameters;
    strncpy(connectionParameters.serialPort, serialPort, sizeof(connectionParameters.serialPort));
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
    } else if (strcmp(role, "rx") == 0) {
        connectionParameters.role = LlRx;
    } else {
        printf("Invalid role: %s\n", role);
        return;
    } 

    printf("Testing llopen...\n");
    if (llopen(connectionParameters) < 0) {
        printf("Failed to open the connection using llopen\n");
        return;
    }
    printf("Connection established using llopen\n");

    if (connectionParameters.role == LlTx) {  // Transmitter
        // send file data
        FILE *file = fopen(filename, "rb");
        if (!file) {
            printf("Failed to open file: %s\n", filename);
            llclose(statisticsEnabled);
            return;
        }

        // START control packet
        unsigned char controlPacket[MAX_FRAME_SIZE];
        int fileSize;
        fseek(file, 0, SEEK_END);
        fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);
        int controlPacketLength = createControlPacket(controlPacket, filename, fileSize, 1);

        // send START control packet
        if (llwrite(controlPacket, controlPacketLength) < 0) {
            printf("Failed to send START control packet\n");
            fclose(file);
            llclose(statisticsEnabled);
            return;
        }
        printf("START control packet sent\n");

        // send file data in DATA packets
        unsigned char dataBuffer[256];
        int bytesRead;
        int sequenceNumber = 0;

        printf("Sending file data using DATA packets...\n");
        while ((bytesRead = fread(dataBuffer, 1, sizeof(dataBuffer), file)) > 0) {
            // create DATA packet
            unsigned char dataPacket[MAX_FRAME_SIZE];
            int dataPacketLength = createDataPacket(dataPacket, sequenceNumber, dataBuffer, bytesRead);

            // send DATA packet
            if (llwrite(dataPacket, dataPacketLength) < 0) {
                printf("Failed to send DATA packet\n");
                fclose(file);
                llclose(statisticsEnabled);
                return;
            }
            printf("Sent DATA packet with sequence number: %d, size: %d bytes\n", sequenceNumber, bytesRead);

            sequenceNumber = (sequenceNumber + 1) % 2;
        }

        // send END control packet
        controlPacketLength = createControlPacket(controlPacket, filename, fileSize, 0);
        if (llwrite(controlPacket, controlPacketLength) < 0) {
            printf("Failed to send END control packet\n");
            fclose(file);
            llclose(statisticsEnabled);
            return;
        }
        printf("END control packet sent\n");

        fclose(file);
        printf("File transmission complete\n");
    }

    if (connectionParameters.role == LlRx) {  // Receiver
        // receive file data
        FILE *file = fopen(filename, "wb");
        if (!file) {
            printf("Failed to open file for writing: %s\n", filename);
            llclose(statisticsEnabled);
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

    printf("Testing llclose...\n");
    if (llclose(statisticsEnabled) < 0) {
        printf("Failed to close the connection using llclose\n");
        return;
    }
    printf("Connection closed successfully using llclose\n");
}




