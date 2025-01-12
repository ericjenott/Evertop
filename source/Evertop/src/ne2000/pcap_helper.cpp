#include "pcap_helper.h"
//#include "pcap.h"

//pcap_t *pCapDevHandle = NULL;
PCAP_RECEIVEHANDLER receiveHandler = NULL;

#define MAC_LEN 6
unsigned char pcap_mac_addr[MAC_LEN];

#define PCAP_NETMASK_UNKNOWN 0xffffffff 

//void packet_handler(u_char *param, const struct pcap_pkthdr *header, const u_char *pkt_data)
void packet_handler(u_char *param)
{
	// struct tm *ltime;
	// char timestr[16];
	// time_t local_tv_sec;

	// /*
	// * unused parameters
	// */
	// //(VOID)(param);
  // (void)(param);
	// //(VOID)(pkt_data);
  // (void)(pkt_data);

	// /* convert the timestamp to readable format */
	// local_tv_sec = header->ts.tv_sec;
	// ltime = localtime(&local_tv_sec);
	// strftime(timestr, sizeof timestr, "%H:%M:%S", ltime);

	// // printf("%s,%.6d len:%d\n", timestr, header->ts.tv_usec, header->len);

	// if (receiveHandler != NULL)
	// {
		// receiveHandler(pkt_data, header->len);
	// }
}

bool setupPcapDevice(unsigned char DeviceIndex, unsigned char addr[MAC_LEN])
{	
  return true;
	// pcap_if_t *alldevs;
	// pcap_if_t *d;
	// int inum;
	// int devCount = 0;
	// char errbuf[PCAP_ERRBUF_SIZE];
	// char packet_filter[512];
	// struct bpf_program fcode;

	// memcpy(pcap_mac_addr, addr, MAC_LEN);

	// // only get packet destined for our emulated NE2000 or broadcast message (all FF in mac address)
	// // if broadcast msgs are filtered, DHCP negotiation will fail (some DHCP servers sends broadcast messages as replies!)
	// sprintf(packet_filter, 
		// "ether dst  %02X:%02X:%02X:%02X:%02X:%02X or ether dst ff:ff:ff:ff:ff:ff",
		// // "ether dst  %02X:%02X:%02X:%02X:%02X:%02X",
			// pcap_mac_addr[0], pcap_mac_addr[1], pcap_mac_addr[2], pcap_mac_addr[3], pcap_mac_addr[4], pcap_mac_addr[5]);


	// bool isOk = false;
	// if (pcap_findalldevs(&alldevs, errbuf) == -1)
	// {
		// printf("Error in pcap_findalldevs: %s\n", errbuf);
	// }
	// else {
		// for (d = alldevs; d; d = d->next)
		// {
			// printf("%d. %s", devCount, d->name);
			// if (d->description)
				// printf(" (%s)\n", d->description);
			// else
				// printf(" (No description available)\n");

			// if (devCount == DeviceIndex)
			// {
				// if ((pCapDevHandle = pcap_open_live(d->name,	// name of the device
					// 65536,			// portion of the packet to capture. 
									// // 65536 grants that the whole packet will be captured on all the MACs.
					// 1,				// promiscuous mode (nonzero means promiscuous)
					// 1,				// read timeout. shorter values mean packets arrive at the emulator faster
					// errbuf			// error buffer
					// )) == NULL)
				// {
					// printf("Unable to open the adapter. %s is not supported by WinPcap\n", d->name);
				// }
				// else {
					// printf("Successfully opened network device #%d\n", DeviceIndex);

					// //compile the filter
					// if (pcap_compile(pCapDevHandle, &fcode, packet_filter, 1, PCAP_NETMASK_UNKNOWN) < 0) // 
					// {
						// printf("Unable to compile the MAC Address filter\n");
					// }

					// //set the filter
					// if (pcap_setfilter(pCapDevHandle, &fcode)<0)
					// {
						// printf("Error setting the MAC Address filter.\n");
					// }


					// isOk = true;
				// }
			// }

			// devCount++;
		// }
	// }

	// if (devCount == 0)
	// {
		// printf("No network interfaces found! Make sure WinPcap 4.1.2 is installed.\n");
	// }
	// else {
		// // Free the device list, we don't need it any more
		// pcap_freealldevs(alldevs);
	// }

	// if (pCapDevHandle == NULL)
	// {
		// printf("Cannot find selected network device. Check device index %d\n", DeviceIndex);
	// }
	// else {
		
	// }

	// return isOk;
}

void PCapReceiveThread() 
{
	//pcap_loop(pCapDevHandle, 0, packet_handler, NULL);
}

void closePcapDevice() 
{
	// if (pCapDevHandle != NULL)
	// {
		// pcap_close(pCapDevHandle);
	// }
}

void sendPcapPacket(unsigned char* data, int len) 
{
	// if (pCapDevHandle)
	// {
		// if (pcap_sendpacket(pCapDevHandle, data, len) != 0)
		// {
			// printf("Error sending the packet: %s\n", pcap_geterr(pCapDevHandle));
		// }
		// else {
			// // printf("Packet sent. Size=%d\n", len);
		// }
	// }
	// else {
		// printf("PCAP Device not open!\n");
	// }
}

void PCapSetReceiveHandler(PCAP_RECEIVEHANDLER ReceiveHandler) 
{
	receiveHandler = ReceiveHandler;
}