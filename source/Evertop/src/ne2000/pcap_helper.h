#pragma once
#include "Arduino.h"

void closePcapDevice();
bool setupPcapDevice(unsigned char DeviceIndex, unsigned char addr[6]);
void sendPcapPacket(unsigned char* data, int len);
void PCapReceiveThread();

// Handler when a packet is received
typedef void(*PCAP_RECEIVEHANDLER)(const unsigned char* data, unsigned int count);
void PCapSetReceiveHandler(PCAP_RECEIVEHANDLER ReceiveHandler);