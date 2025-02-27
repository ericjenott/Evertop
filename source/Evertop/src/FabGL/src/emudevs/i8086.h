 

#pragma once

#include "../fabgl.h"

#include <stdint.h>


#define I8086_SHOW_OPCODE_STATS 0

#ifndef I80186MODE
  #define I80186MODE 1
#endif



namespace fabgl {


class i8086 {

public:

  // callbacks

  typedef void (*WritePort)(void * context, int address, uint8_t value, bool isWord, bool is2ndByte);
  typedef uint8_t (*ReadPort)(void * context, int address, bool isWord, bool is2ndByte);
  typedef void (*WriteVideoMemory8)(void * context, int address, uint8_t value);
  typedef void (*WriteVideoMemory16)(void * context, int address, uint16_t value);
  typedef uint8_t (*ReadVideoMemory8)(void * context, int address);
  typedef uint16_t (*ReadVideoMemory16)(void * context, int address);
  typedef bool (*Interrupt)(void * context, int num);
  typedef void (*IncHltCount)();
  typedef void (*IncMemsCount)();
  typedef bool (*IntPending)(int &IntNumber);

  static void setCallbacks(void * context, ReadPort readPort, WritePort writePort, WriteVideoMemory8 writeVideoMemory8, WriteVideoMemory16 writeVideoMemory16, ReadVideoMemory8 readVideoMemory8, ReadVideoMemory16 readVideoMemory16, Interrupt interrupt, IncHltCount incHltCount, IncMemsCount incMemsCount, IntPending intPending) 
  {
    s_context            = context;
    s_readPort           = readPort;
    s_writePort          = writePort;
    s_writeVideoMemory8  = writeVideoMemory8;
    s_writeVideoMemory16 = writeVideoMemory16;
    s_readVideoMemory8   = readVideoMemory8;
    s_readVideoMemory16  = readVideoMemory16;
    s_interrupt          = interrupt;
    s_incHltCount		     = incHltCount;
    s_incMemsCount		   = incMemsCount;
    s_intPending         = intPending;
  }

  static void setMemory(uint8_t * memory) { s_memory = memory; }

  static void reset();

  static void setAL(uint8_t value);
  static void setAH(uint8_t value);
  static void setBL(uint8_t value);
  static void setBH(uint8_t value);
  static void setCL(uint8_t value);
  static void setCH(uint8_t value);
  static void setDL(uint8_t value);
  static void setDH(uint8_t value);

  static uint8_t AL();
  static uint8_t AH();
  static uint8_t BL();
  static uint8_t BH();
  static uint8_t CL();
  static uint8_t CH();
  static uint8_t DL();
  static uint8_t DH();

  static void setAX(uint16_t value);
  static void setBX(uint16_t value);
  static void setCX(uint16_t value);
  static void setDX(uint16_t value);
  static void setDI(uint16_t value);
  static void setCS(uint16_t value);
  static void setDS(uint16_t value);
  static void setSS(uint16_t value);
  static void setES(uint16_t value);
  static void setIP(uint16_t value);
  static void setSP(uint16_t value);

  static uint16_t AX();
  static uint16_t BX();
  static uint16_t CX();
  static uint16_t DX();
  static uint16_t BP();
  static uint16_t SI();
  static uint16_t DI();
  static uint16_t SP();

  static uint16_t CS();
  static uint16_t ES();
  static uint16_t DS();
  static uint16_t SS();

  static bool flagIF();
  static bool flagTF();
  static bool flagCF();
  static bool flagZF();
  static bool flagOF();
  static bool flagDF();
  static bool flagSF();
  static bool flagAF();
  static bool flagPF();
  static uint8_t *get_regs();
  static uint8_t *get_flags();
  static uint16_t *get_reg_ip();
  static uint8_t *get_seg_override_en();
  static uint8_t *get_rep_override_en();
  
  static void setFlagZF(bool value);
  static void setFlagCF(bool value);

  static uint16_t IP();

  static bool halted()                                    { return s_halted; }

  static bool IRQ(uint8_t interrupt_num);

  static void step();

  // static uint8_t *cpu_regs;
  // static uint8_t *cpu_flags;

private:

  static uint8_t WMEM8(int addr, uint8_t value);
  static uint16_t WMEM16(int addr, uint16_t value);
  static uint8_t RMEM8(int addr);
  static uint16_t RMEM16(int addr);

  static uint16_t make_flags();
  static void set_flags(int new_flags);

  static void pc_interrupt(uint8_t interrupt_num);
  
  static char pc_interrupt_reset(unsigned char interrupt_num);

  static uint8_t raiseDivideByZeroInterrupt();

  static void stepEx(uint8_t const * opcode_stream);



  static void *             s_context;
  static ReadPort           s_readPort;
  static WritePort          s_writePort;
  static WriteVideoMemory8  s_writeVideoMemory8;
  static WriteVideoMemory16 s_writeVideoMemory16;
  static ReadVideoMemory8   s_readVideoMemory8;
  static ReadVideoMemory16  s_readVideoMemory16;
  static Interrupt          s_interrupt;
  static IncHltCount		    s_incHltCount;
  static IncMemsCount		    s_incMemsCount;
  static IntPending         s_intPending;

  static bool               s_pendingIRQ;
  static uint8_t            s_pendingIRQIndex;
  static uint8_t *          s_memory;
  static bool               s_halted;
  
  // Helper functions
  static void callxms();
  
  // Set carry flag
  static char set_CF(int new_CF);
  
  // Set overflow flag
  static char set_OF(int new_OF);
  
  static void * checkmoveaddress(uint16_t handle, uint16_t segment, uint16_t offset,	uint32_t address, uint32_t count,	uint8_t errorcode_handle, uint8_t errorcode_address);


};


} // namespace fabgl
