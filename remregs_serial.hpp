#ifndef __REMREGS_SERIAL_H
#define __REMREGS_SERIAL_H

#include <stdint.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#endif

#include "remregs.h"
#include "mutex.h"

#ifdef _WIN32
typedef HANDLE FILE_T;
#else
typedef int FILE_T;
#endif

class CSerialRemoteRegs: virtual public CRemoteRegs
{

public:

  CSerialRemoteRegs();
  ~CSerialRemoteRegs();
  bool open(const char* portname, int spd);
  bool sync();
  void close();

  bool get_reg_b(const uint16_t addr, uint8_t& res);
  bool get_reg_w(const uint16_t addr, uint16_t& res);
  bool get_reg_dw(const uint16_t addr, uint32_t& res);
  bool get_reg_mb(const uint16_t addr, uint8_t* data, uint8_t& len);

  uint8_t get_reg_b(const uint16_t addr);
  uint16_t get_reg_w(const uint16_t addr);
  uint32_t get_reg_dw(const uint16_t addr);

  bool set_reg_b(const uint16_t addr, const uint8_t val);
  bool set_reg_w(const uint16_t addr, const uint16_t val);
  bool set_reg_dw(const uint16_t addr, const uint32_t val);
  bool set_reg_mb(const uint16_t addr, const uint8_t* data, const uint8_t len);

private:

  static const uint8_t ACK = 6;
  static const uint8_t NAK = 15;

  bool reg_op(const uint8_t op, const uint16_t addr, const uint8_t* data, const int len, const bool lock = false);

#ifndef _WIN32
  struct termios oldtio;
#endif
  FILE_T file;
  bool connected;
  mutex_t mutex;
};

#endif
