#pragma once

#include <stdint.h>

// Emulated NE200 IRQ
#define NE2000_ADDR	   0x300

/*
INT 08 - IRQ0 - SYSTEM TIMER; CPU-generated (80286+)
INT 09 - IRQ1 - KEYBOARD DATA READY; CPU-generated (80286,80386)
INT 0A - IRQ2 - LPT2/EGA,VGA/IRQ9; CPU-generated (80286+)
INT 0B - IRQ3 - SERIAL COMMUNICATIONS (COM2); CPU-generated (80286+)
INT 0C - IRQ4 - SERIAL COMMUNICATIONS (COM1); CPU-generated (80286+)
INT 0D - IRQ5 - FIXED DISK/LPT2/reserved; CPU-generated (80286+)
INT 0E - IRQ6 - DISKETTE CONTROLLER; CPU-generated (80386+)
INT 0F - IRQ7 - PARALLEL PRINTER
*/
#define NE2000_INT_NO	   0x0F		

// define to show NE2000 debug output 
//#define DEBUG_NE2000

#define MAX_ETH_FRAME_SIZE 1514

#define MIN_BUF_SIZE 60

#define NE2000_PMEM_SIZE    (32*1024)
#define NE2000_PMEM_START   (16*1024)
#define NE2000_PMEM_END     (NE2000_PMEM_SIZE+NE2000_PMEM_START)
#define NE2000_MEM_SIZE     NE2000_PMEM_END

// Little Endian macro. Not needed on Windows (always little endian)
// http://www.fourwalledcubicle.com/files/LUFA/Doc/120219/html/group___group___endian_conversion.html
#define 	le16_to_cpu(x)   (x)
#define 	cpu_to_le16(x)   (x)

typedef struct NE2000State {
	uint8_t cmd;
	uint32_t start;
	uint32_t stop;
	uint8_t boundary;
	uint8_t tsr;
	uint8_t tpsr;
	uint16_t tcnt;
	uint16_t rcnt;
	uint32_t rsar;
	uint8_t rsr;
	uint8_t rxcr;
	uint8_t isr;
	uint8_t dcfg;
	uint8_t imr;
	uint8_t phys[6]; /* mac address */
	uint8_t curpag;
	uint8_t mult[8]; /* multicast mask array */
	uint8_t macaddr[6];
	//uint8_t mem[NE2000_MEM_SIZE];
	uint8_t * mem;
} NE2000State;

void ne2000_reset(NE2000State *s);
void ne2000_update_irq(NE2000State *s);
int ne2000_buffer_full(NE2000State *s);
int ne2000_can_receive(NE2000State *s);
void ne2000_receive(NE2000State *s, const uint8_t *buf, int size);

// IO Memory is first 16 bytes of the card memory, followed by ASIC memory
// ne->nic_addr = ne->iobase + NE_NOVELL_NIC_OFFSET;
// ne->asic_addr = ne->iobase + NE_NOVELL_ASIC_OFFSET;
// http://www.jbox.dk/sanos/source/sys/dev/ne2000.c.html#:279
#define NE_NOVELL_NIC_OFFSET    0x00
#define NE_NOVELL_ASIC_OFFSET   0x10

/*
IO functions are for read/write access the NE2000 registers
*/
void ne2000_ioport_write(NE2000State *s, uint8_t addr, uint8_t val);
uint8_t ne2000_ioport_read(NE2000State *s, uint8_t addr);

/*
ASIC functions are for read/write access to the 64K private RAM (chip memory) of the NE2000

The NE2000 memory is accessed through the data port of the asic (offset 0) after setting up a remote-DMA transfer.
Both byte and word accesses are allowed. The first 16 bytes contains the MAC address at even locations,
and there is 16K of buffer memory starting at 16K
*/
void ne2000_asic_ioport_write(NE2000State *s, uint32_t addr, uint16_t val);
uint16_t ne2000_asic_ioport_read(NE2000State *s, uint32_t addr);

// Called when incoming bytes are received
void ne2000_receive(NE2000State *s, const uint8_t *buf, int size);

// Triggers when incoming bytes are received.
typedef void(*NE2000_IRQHANDLER)(void *context);
void NE2000SetIRQHandler(void *context, NE2000_IRQHANDLER IRQHandler);

// Triggers when data should be sent out
typedef void(*NE2000_SENDHANDLER)(unsigned char* data, uint16_t count);
void NE2000SetSendHandler(NE2000_SENDHANDLER UpdateHandler);

// Triggers when the operation mode (8-bit and 16-bit) is resey (reset?)
typedef void(*NE2000_MODECHANGEHANDLER)(bool is16bit);
void NE2000SetModeChangeHandler(NE2000_MODECHANGEHANDLER ModeChangeHandler);

// Whether the NE2000 is operating in 16-bit mode (ASIC memory are read/written 2 bytes at a times
// The NE2000 was originally made for 16-bit ISA slot, although it will work in 8-bit slot
// Most drivers for the NE2000 works in 16 bit only, although a patch is available for the Crynwr NE2000 packet drivers to work in 8-bit
// No patches are available for the NDIS2 driver, hence our code must emulate 16-bit properly
// In 16-bit mode, dcfg is set to 1 and "in ax, dx" / "out dx, ax" will translate to 2 successive calls to ReadPort / WritePort of 8086 Tiny Plus code
// In 8-bit mode, dcfg is set to 0 and "in al, dx" / "out dx, al" will just be a single call to ReadPort / WritePort of 8086 Tiny Plus code
bool NE2000_IsIn16BitMode(NE2000State *s);

bool NE2000_IsReadingPROM(NE2000State *s);