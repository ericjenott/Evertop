/*
 * QEMU NE2000 emulation
 *
 * Taken from https://github.com/ntddk/temu/blob/master/hw/ne2000.c
 * Modified by ToughDev for using with Super 8086 Tiny
 * See also: https://wiki.osdev.org/Ne2000
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <cstring>
#include "ne2000.h"

NE2000_MODECHANGEHANDLER  modeChangeHandler;
NE2000_IRQHANDLER  irqHandler;
NE2000_SENDHANDLER sendDataHandler;

void *m_context;

#define E8390_CMD	0x00  /* The command register (for all pages) */
 /* Page 0 register offsets. */
#define EN0_CLDALO	0x01	/* Low byte of current local dma addr  RD */
#define EN0_STARTPG	0x01	/* Starting page of ring bfr WR */
#define EN0_CLDAHI	0x02	/* High byte of current local dma addr  RD */
#define EN0_STOPPG	0x02	/* Ending page +1 of ring bfr WR */
#define EN0_BOUNDARY	0x03	/* Boundary page of ring bfr RD WR */
#define EN0_TSR		0x04	/* Transmit status reg RD */
#define EN0_TPSR	0x04	/* Transmit starting page WR */
#define EN0_NCR		0x05	/* Number of collision reg RD */
#define EN0_TCNTLO	0x05	/* Low  byte of tx byte count WR */
#define EN0_FIFO	0x06	/* FIFO RD */
#define EN0_TCNTHI	0x06	/* High byte of tx byte count WR */
#define EN0_ISR		0x07	/* Interrupt status reg RD WR */
#define EN0_CRDALO	0x08	/* low byte of current remote dma address RD */
#define EN0_RSARLO	0x08	/* Remote start address reg 0 */
#define EN0_CRDAHI	0x09	/* high byte, current remote dma address RD */
#define EN0_RSARHI	0x09	/* Remote start address reg 1 */
#define EN0_RCNTLO	0x0a	/* Remote byte count reg WR */
#define EN0_RTL8029ID0	0x0a	/* Realtek ID byte #1 RD */
#define EN0_RCNTHI	0x0b	/* Remote byte count reg WR */
#define EN0_RTL8029ID1	0x0b	/* Realtek ID byte #2 RD */
#define EN0_RSR		0x0c	/* rx status reg RD */
#define EN0_RXCR	0x0c	/* RX configuration reg WR */
#define EN0_TXCR	0x0d	/* TX configuration reg WR */
#define EN0_COUNTER0	0x0d	/* Rcv alignment error counter RD */
#define EN0_DCFG	0x0e	/* Data configuration reg WR */
#define EN0_COUNTER1	0x0e	/* Rcv CRC error counter RD */
#define EN0_IMR		0x0f	/* Interrupt mask reg WR */
#define EN0_COUNTER2	0x0f	/* Rcv missed frame error counter RD */

#define EN1_PHYS        0x11
#define EN1_CURPAG      0x17
#define EN1_MULT        0x18

#define EN2_STARTPG	0x21	/* Starting page of ring bfr RD */
#define EN2_STOPPG	0x22	/* Ending page +1 of ring bfr RD */

#define EN3_CONFIG0	0x33
#define EN3_CONFIG1	0x34
#define EN3_CONFIG2	0x35
#define EN3_CONFIG3	0x36

 /*  Register accessed at EN_CMD, the 8390 base addr.  */
#define E8390_STOP	0x01	/* Stop and reset the chip */
#define E8390_START	0x02	/* Start the chip, clear reset */
#define E8390_TRANS	0x04	/* Transmit a frame */
#define E8390_RREAD	0x08	/* Remote read */
#define E8390_RWRITE	0x10	/* Remote write  */
#define E8390_NODMA	0x20	/* Remote DMA */
#define E8390_PAGE0	0x00	/* Select page chip registers */
#define E8390_PAGE1	0x40	/* using the two high-order bits */
#define E8390_PAGE2	0x80	/* Page 3 is invalid. */

 /* Bits in EN0_ISR - Interrupt status register */
#define ENISR_RX	0x01	/* Receiver, no error */
#define ENISR_TX	0x02	/* Transmitter, no error */
#define ENISR_RX_ERR	0x04	/* Receiver, with error */
#define ENISR_TX_ERR	0x08	/* Transmitter, with error */
#define ENISR_OVER	0x10	/* Receiver overwrote the ring */
#define ENISR_COUNTERS	0x20	/* Counters need emptying */
#define ENISR_RDC	0x40	/* remote dma complete */
#define ENISR_RESET	0x80	/* Reset completed */
#define ENISR_ALL	0x3f	/* Interrupts we will enable */

 /* Bits in received packet status byte and EN0_RSR*/
#define ENRSR_RXOK	0x01	/* Received a good packet */
#define ENRSR_CRC	0x02	/* CRC error */
#define ENRSR_FAE	0x04	/* frame alignment error */
#define ENRSR_FO	0x08	/* FIFO overrun */
#define ENRSR_MPA	0x10	/* missed pkt */
#define ENRSR_PHY	0x20	/* physical/multicast address */
#define ENRSR_DIS	0x40	/* receiver disable. set in monitor mode */
#define ENRSR_DEF	0x80	/* deferring */

 /* Transmitted packet status, EN0_TSR. */
#define ENTSR_PTX 0x01	/* Packet transmitted without error */
#define ENTSR_ND  0x02	/* The transmit wasn't deferred. */
#define ENTSR_COL 0x04	/* The transmit collided at least once. */
#define ENTSR_ABT 0x08  /* The transmit collided 16 times, and was deferred. */
#define ENTSR_CRS 0x10	/* The carrier sense was lost. */
#define ENTSR_FU  0x20  /* A "FIFO underrun" occurred during transmit. */
#define ENTSR_CDH 0x40	/* The collision detect "heartbeat" signal was lost. */
#define ENTSR_OWC 0x80  /* There was an out-of-window collision. */

#define POLYNOMIAL 0x04c11db6

void NE2000SetIRQHandler(void *context, NE2000_IRQHANDLER IRQHandler) 
{
  m_context = context;
	irqHandler = IRQHandler;
}

void NE2000SetSendHandler(NE2000_SENDHANDLER SENDHandler)
{
	sendDataHandler = SENDHandler;
}

void NE2000SetModeChangeHandler(NE2000_MODECHANGEHANDLER ModeChangeHandler)
{
	modeChangeHandler = ModeChangeHandler;
}

void ne2000_reset(NE2000State *s)
{
    int i;

    s->isr = ENISR_RESET;
    memcpy(s->mem, s->macaddr, 6);
    s->mem[14] = 0x57;
    s->mem[15] = 0x57;

    /* duplicate prom data */
    for(i = 15;i >= 0; i--) 
    {
        s->mem[2 * i] = s->mem[i];
        s->mem[2 * i + 1] = s->mem[i];
    }
}

bool NE2000_IsReadingPROM(NE2000State *s)
{
	return (s->rsar <= 15);
}

void ne2000_update_irq(NE2000State *s)
{
    int isr;
    isr = (s->isr & s->imr) & 0x7f;
#if defined(DEBUG_NE2000)
    printf("NE2000: Set IRQ to %d (%02x %02x)\n", isr ? 1 : 0, s->isr, s->imr);
#endif

	if (isr)
	{
		if (irqHandler)
		{
#if defined(DEBUG_NE2000)
			printf("NE2000 IRQ Raised\n");
#endif
			irqHandler(m_context);
		}
		else 
    {
#if defined(DEBUG_NE2000)
			printf("NE2000 IRQ Handler Not Assigned\n");
#endif
		}
	}
}

int compute_mcast_idx(const uint8_t *ep)
{
    uint32_t crc;
    int carry, i, j;
    uint8_t b;

    crc = 0xffffffff;
    for (i = 0; i < 6; i++) {
        b = *ep++;
        for (j = 0; j < 8; j++) {
            carry = ((crc & 0x80000000L) ? 1 : 0) ^ (b & 0x01);
            crc <<= 1;
            b >>= 1;
            if (carry)
                crc = ((crc ^ POLYNOMIAL) | carry);
        }
    }
    return (crc >> 26);
}

int ne2000_buffer_full(NE2000State *s)
{
    int avail, index, boundary;

    //printf("s->curpag = %d, s->boundary = %d\n", s->curpag, s->boundary);
    index = s->curpag << 8;
    boundary = s->boundary << 8;
    //printf("index = %d, boundary = %d\n", index, boundary);
    if (index < boundary)
    {
      //printf("index < boundary\n");
      avail = boundary - index;
    }
    else
    {
      //printf("else (index is not < boundary)\n");
      //printf("s->stop = %d, s->start = %d\n", s->stop, s->start);
      avail = (s->stop - s->start) - (index - boundary);
    }
    //printf("avail = %d\n", avail);  
      
    if (avail < (MAX_ETH_FRAME_SIZE + 4))
    {
#if defined(DEBUG_NE2000)
      printf("NE2000 Buffer is full. Avail = %d\n", avail);
#endif
      return 1;
    }

    return 0;
}

int ne2000_can_receive(NE2000State *s)
{
    if (s->cmd & E8390_STOP)
        return 1;
    return !ne2000_buffer_full(s);
}

void ne2000_receive(NE2000State *s, const uint8_t *buf, int size)
{
    uint8_t *p;
    unsigned int total_len, next, avail, len, index, mcast_idx;
    uint8_t buf1[60];
    static const uint8_t broadcast_macaddr[6] =
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

#if defined(DEBUG_NE2000)
    printf("NE2000: received len=%d\n", size);
#endif

    
    // if(*(uint8_t*)(buf) == 0x08 && *(uint8_t*)(buf + 1) == 0xf9 && *(uint8_t*)(buf + 2) == 0xe0)
    // {
      // printf("NE2000: received len=%d\n", size);
      // printf("Recv HEX: ");
      // for (int n = 0; n < size; n++)
      // {
        // printf("%02x ", *(char *)(buf + n));
      // }
      // printf("\n");
      // printf("Recv DEC: ");
      // for (int n = 0; n < size; n++)
      // {
        // printf("%d ", *(unsigned char *)(buf + n));
      // }
      // printf("\n");
      // printf("Recv ASCII: ");
      // for (int n = 0; n < size; n++)
      // {
        // if (*(char *)(buf + n) >= 32 && *(char *)(buf + n) < 127)
        // {
          // printf("%c", *(char *)(buf + n));
        // }
        // else
        // {
          // printf(".");
        // }
      // }
      // printf("\n");
    // }

    //printf("s addr = %p\n", s);
    if (s->cmd & E8390_STOP || ne2000_buffer_full(s))
    {
        return;
    }

    /* XXX: check this */
    if (s->rxcr & 0x10) 
    {
        /* promiscuous: receive all */
    }
    else 
    {
        if (!memcmp(buf,  broadcast_macaddr, 6)) 
        {
            /* broadcast address */
            if (!(s->rxcr & 0x04))
                return;
        }
        else if (buf[0] & 0x01) 
        {
            /* multicast */
            if (!(s->rxcr & 0x08))
                return;
            mcast_idx = compute_mcast_idx(buf);
            if (!(s->mult[mcast_idx >> 3] & (1 << (mcast_idx & 7))))
                return;
        } else if (s->mem[0] == buf[0] &&
                   s->mem[2] == buf[1] &&
                   s->mem[4] == buf[2] &&
                   s->mem[6] == buf[3] &&
                   s->mem[8] == buf[4] &&
                   s->mem[10] == buf[5]) 
        {
            /* match */
            //printf("MAC address match!\n");
        }
        else 
        {
            return;
        }
    }


    /* if too small buffer, then expand it */
    if (size < MIN_BUF_SIZE) 
    {
        memcpy(buf1, buf, size);
        memset(buf1 + size, 0, MIN_BUF_SIZE - size);
        buf = buf1;
        size = MIN_BUF_SIZE;
    }

    index = s->curpag << 8;
    /* 4 bytes for header */
    total_len = size + 4;
    /* address for next packet (4 bytes for CRC) */
    next = index + ((total_len + 4 + 255) & ~0xff);
    if (next >= s->stop)
        next -= (s->stop - s->start);
    /* prepare packet header */
    p = s->mem + index;
    s->rsr = ENRSR_RXOK; /* receive status */
    /* XXX: check this */
    if (buf[0] & 0x01)
        s->rsr |= ENRSR_PHY;
    p[0] = s->rsr;
    p[1] = next >> 8;
    p[2] = total_len;
    p[3] = total_len >> 8;
    index += 4;

    /* write packet data */
    while (size > 0) 
    {
        if (index <= s->stop)
            avail = s->stop - index;
        else
            avail = 0;
        len = size;
        if (len > avail)
            len = avail;
        memcpy(s->mem + index, buf, len);
        buf += len;
        index += len;
        if (index == s->stop)
            index = s->start;
        size -= len;
    }
    s->curpag = next >> 8;

    /* now we can signal we have received something */
    s->isr |= ENISR_RX;
    ne2000_update_irq(s);
}

void ne2000_ioport_write(NE2000State *s, uint8_t addr, uint8_t val)
{
    int offset, page, index;

    addr &= 0xf;
#ifdef DEBUG_NE2000
    printf("NE2000: write addr=0x%x val=0x%02x\n", addr, val);
#endif
    if (addr == E8390_CMD) 
    {
        /* control register */
        s->cmd = val;
        if (!(val & E8390_STOP)) 
        { 
            /* START bit makes no sense on RTL8029... */
            s->isr &= ~ENISR_RESET;
            /* test specific case: zero length transfer */
            if ((val & (E8390_RREAD | E8390_RWRITE)) && s->rcnt == 0) 
            {
                s->isr |= ENISR_RDC;
                ne2000_update_irq(s);
            }
            if (val & E8390_TRANS) 
            {
                index = (s->tpsr << 8);
                /* XXX: next 2 lines are a hack to make netware 3.11 work */
                if (index >= NE2000_PMEM_END)
                {
                    index -= NE2000_PMEM_SIZE;
                }
                /* fail safe: check range on the transmitted length  */
                if (index + s->tcnt <= NE2000_PMEM_END) 
                {
                  if (sendDataHandler)
                  {
#if defined(DEBUG_NE2000)
                    printf("Sending %d bytes\n", s->tcnt);
#endif
                    sendDataHandler(s->mem + index, s->tcnt);
                  }
                  else 
                  {
                    printf("Send data handler not assigned!\n");
                  }
                }
                /* signal end of transfer */
                s->tsr = ENTSR_PTX;
                s->isr |= ENISR_TX;
                s->cmd &= ~E8390_TRANS;
                ne2000_update_irq(s);
            }
        }
    }
    else 
    {
        page = s->cmd >> 6;
        offset = addr | (page << 4);
        switch(offset) {
        case EN0_STARTPG:
            s->start = val << 8;
            printf("  ******* SET S->START TO %d *********  \n", s->start);
            break;
        case EN0_STOPPG:
            s->stop = val << 8;
            printf("  ******* SET S->STOP TO %d *********  \n", s->stop);
            break;
        case EN0_BOUNDARY:
            s->boundary = val;
            break;
        case EN0_IMR:
            s->imr = val;
            ne2000_update_irq(s);
            break;
        case EN0_TPSR:
            s->tpsr = val;
            break;
        case EN0_TCNTLO:
            s->tcnt = (s->tcnt & 0xff00) | val;
            break;
        case EN0_TCNTHI:
            s->tcnt = (s->tcnt & 0x00ff) | (val << 8);
            break;
        case EN0_RSARLO:
            s->rsar = (s->rsar & 0xff00) | val;
            break;
        case EN0_RSARHI:
            s->rsar = (s->rsar & 0x00ff) | (val << 8);
            break;
        case EN0_RCNTLO:
            s->rcnt = (s->rcnt & 0xff00) | val;
            break;
        case EN0_RCNTHI:
            s->rcnt = (s->rcnt & 0x00ff) | (val << 8);
            break;
        case EN0_RXCR:
            s->rxcr = val;
            break;
        case EN0_DCFG:
            {
              s->dcfg = val;

              bool is16bit = val & 0x01;
#if defined(DEBUG_NE2000)
              if (is16bit)
              {
                printf("Configured for 16-bit access\n");
              }
              else {
                printf("Configured for 8-bit access\n");
              }
#endif
              if (modeChangeHandler)
              {
                modeChangeHandler(is16bit);
              }
            }
            break;
        case EN0_ISR:
            s->isr &= ~(val & 0x7f);
            ne2000_update_irq(s);
            break;
		case EN1_PHYS:
		case EN1_PHYS + 1:
		case EN1_PHYS + 2:
		case EN1_PHYS + 3:
		case EN1_PHYS + 4:
		case EN1_PHYS + 5:
            s->phys[offset - EN1_PHYS] = val;
            break;
        case EN1_CURPAG:
            s->curpag = val;
            break;
		case EN1_MULT:
		case EN1_MULT + 1:
		case EN1_MULT + 2:
		case EN1_MULT + 3:
		case EN1_MULT + 4:
		case EN1_MULT + 5:
		case EN1_MULT + 6:
		case EN1_MULT + 7:
            s->mult[offset - EN1_MULT] = val;
            break;
        }
    }
}

uint8_t ne2000_ioport_read(NE2000State *s, uint8_t addr)
{
	uint8_t offset, page, ret;

    addr &= 0xf;
    if (addr == E8390_CMD) {
        ret = s->cmd;
    } else {
        page = s->cmd >> 6;
        offset = addr | (page << 4);
        switch(offset) {
        case EN0_TSR:
            ret = s->tsr;
            break;
        case EN0_BOUNDARY:
            ret = s->boundary;
            break;
        case EN0_ISR:
            ret = s->isr;
            break;
	case EN0_RSARLO:
	    ret = s->rsar & 0x00ff;
	    break;
	case EN0_RSARHI:
	    ret = s->rsar >> 8;
	    break;
	case EN1_PHYS: 
	case EN1_PHYS +	1:
	case EN1_PHYS + 2:
	case EN1_PHYS + 3:
	case EN1_PHYS + 4:
	case EN1_PHYS + 5:
            ret = s->phys[offset - EN1_PHYS];
			break;
        case EN1_CURPAG:
            ret = s->curpag;
            break;
		case EN1_MULT:
		case EN1_MULT + 1:
		case EN1_MULT + 2:
		case EN1_MULT + 3:
		case EN1_MULT + 4:
		case EN1_MULT + 5:
		case EN1_MULT + 6:
		case EN1_MULT + 7:
            ret = s->mult[offset - EN1_MULT];
            break;
        case EN0_RSR:
            ret = s->rsr;
            break;
        case EN2_STARTPG:
            ret = s->start >> 8;
            break;
        case EN2_STOPPG:
            ret = s->stop >> 8;
            break;
	case EN0_RTL8029ID0:
	    // ret = 0x50;
		ret = 0x08;
	    break;
	case EN0_RTL8029ID1:
	    // ret = 0x21;
		ret = 0x90;
	    break;
	case EN3_CONFIG0:
	    ret = 0;		/* 10baseT media */
	    break;
	case EN3_CONFIG2:
	    ret = 0x40;		/* 10baseT active */
	    break;
	case EN3_CONFIG3:
	    ret = 0x40;		/* Full duplex */
	    break;
        default:
            ret = 0x00;
            break;
        }
    }
#ifdef DEBUG_NE2000
    printf("NE2000: read addr=0x%x val=0x%02x\n", addr, ret);
#endif
    return ret;
}

void ne2000_mem_writeb(NE2000State *s, uint32_t addr, uint8_t val)
{
    if (addr < 32 ||
        (addr >= NE2000_PMEM_START && addr < NE2000_MEM_SIZE)) {
        s->mem[addr] = val;
    }
	else {
#if defined(DEBUG_NE2000)
		printf("invalid address for ne2000_mem_writeb: %d. Value = %d\n", addr, val);
#endif
	}
}

void ne2000_mem_writew(NE2000State *s, uint32_t addr, uint16_t val)
{
	addr &= ~1; /* XXX: check exact behaviour if not even */
	if (addr < 32 ||
		(addr >= NE2000_PMEM_START && addr < NE2000_MEM_SIZE)) {
		*(uint16_t *)(s->mem + addr) = cpu_to_le16(val);
	}
	else {
#if defined(DEBUG_NE2000)
		printf("invalid address for ne2000_mem_writew: %d. Value = %d\n", addr, val);
#endif
	}
}

uint8_t ne2000_mem_readb(NE2000State *s, uint32_t addr)
{
    if (addr < 32 ||
        (addr >= NE2000_PMEM_START && addr < NE2000_MEM_SIZE)) {
        // TEMU_nic_in(addr-NE2000_PMEM_START, 1);
        return s->mem[addr];
    } else {
#if defined(DEBUG_NE2000)
		printf("invalid address for ne2000_mem_readb: %d\n", addr);
#endif

        return 0xff;
    }
}

uint16_t ne2000_mem_readw(NE2000State *s, uint32_t addr)
{
	addr &= ~1; /* XXX: check exact behaviour if not even */
	if (addr < 32 ||
		(addr >= NE2000_PMEM_START && addr < NE2000_MEM_SIZE)) {
		return le16_to_cpu(*(uint16_t *)(s->mem + addr));
	}
	else {
#if defined(DEBUG_NE2000)
		printf("invalid address for ne2000_mem_readw: %d\n", addr);
#endif

		return 0xffff;
	}

}

void ne2000_dma_update(NE2000State *s, int len)
{
    s->rsar += len;
    /* wrap */
    /* XXX: check what to do if rsar > stop */
    if (s->rsar == s->stop)
        s->rsar = s->start;

    if (s->rcnt <= len) {
        s->rcnt = 0;
        /* signal end of transfer */
        s->isr |= ENISR_RDC;
        ne2000_update_irq(s);
    } else {
        s->rcnt -= len;
    }
}

void ne2000_asic_ioport_write(NE2000State *s, uint32_t addr, uint16_t val)
{
#ifdef DEBUG_NE2000
    printf("NE2000: asic write addr=0x%04x val=0x%04x\n", addr, val);
#endif
    if (s->rcnt == 0)
        return;

    if (s->dcfg & 0x01) {
        // 16 bit access 
        ne2000_mem_writew(s, s->rsar, val);
        ne2000_dma_update(s, 2);
    } 
	else 
	{
        /* 8 bit access */
        ne2000_mem_writeb(s, s->rsar, val);
        ne2000_dma_update(s, 1);
    }
}

uint16_t ne2000_asic_ioport_read(NE2000State *s, uint32_t addr)
{
	uint16_t ret;
    if (s->dcfg & 0x01) {
        /* 16 bit access */
        ret = ne2000_mem_readw(s, s->rsar);
        ne2000_dma_update(s, 2);
    } else {
        /* 8 bit access */
        ret = ne2000_mem_readb(s, s->rsar);
        ne2000_dma_update(s, 1);
    }
#ifdef DEBUG_NE2000
    printf("NE2000: asic read rsar=0x%04x addr=0x%04x val=0x%04x\n", s->rsar, addr, ret);
#endif
    return ret;
}

void ne2000_reset_ioport_write(NE2000State *s, uint32_t addr, uint8_t val)
{
#ifdef DEBUG_NE2000
	printf("ne2000_reset_ioport_write. Nothing to do.\n");
#endif
}

uint8_t ne2000_reset_ioport_read(NE2000State *s, uint32_t addr)
{
    ne2000_reset(s);
    return 0;
}

bool NE2000_IsIn16BitMode(NE2000State *s)
{
	return (s->dcfg & 0x01);
}