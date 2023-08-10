/**
 * \file   remregs_serial.cpp
 * \brief  Class for accessing the robot radio interface over a UART(RS-232, USB converter or local)
 * \author Alessandro Crespi & Jeremie Knuesel
 * \date   June 2016
 */

#include <cstdio>
#include <sys/types.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "remregs_serial.hpp"
#include "wcompat.h"

#include <iostream>

/// helper function to read a given number of bytes from a port
/// implemented for both Win32 and POSIX
static bool read_n(FILE_T file, void* buf, size_t count)
{
#ifdef _WIN32

  DWORD l;
  if (ReadFile(file, buf, count, &l, NULL)) {
    return true;
  } else {
    return false;
  }

#else

  timeval to;
  to.tv_sec = 0;
  to.tv_usec = 500000;
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(file, &fds);

  int ready, n_read;
  for (unsigned int i(0); i < count; i++) {
    ready = select(file + 1, &fds, NULL, NULL, &to);
    if (ready <= 0) {
      break;
    }
    n_read = read(file, ((char*) buf) + i, 1);
    if (n_read <= 0) {
      break;
    }
  }
  if (ready <= 0 || n_read <= 0) {
    return false;
  } else {
    return true;
  }
#endif
}

/// helper function to write a given number of bytes to a port
/// implemented for both Win32 and POSIX
static bool write_n(FILE_T file, void* buf, size_t count)
{
#ifdef _WIN32
  DWORD l;
  if (WriteFile(file, buf, count, &l, NULL)) {
    return true;
  } else {
    return false;
  }
#else
  return (write(file, buf, count) == (signed) count);
#endif
}

CSerialRemoteRegs::CSerialRemoteRegs()
{
  mutex_init(&mutex);
#ifdef _WIN32
  file = NULL;
#else
  file = -1;
#endif
  connected = false;
}

CSerialRemoteRegs::~CSerialRemoteRegs()
{
  if (connected) {
    close();
  }
  mutex_destroy(&mutex);
}

bool CSerialRemoteRegs::open(const char* portname, int speed)
{
  if (connected) {
    fprintf(stderr, "Serial port is already open.\n");
    return false;
  }

#ifdef _WIN32
  HANDLE h;
  DCB dcb;
  COMMTIMEOUTS ct;

  h = CreateFile(portname, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, 0);
  if (h == INVALID_HANDLE_VALUE) {
    wperror(portname);
    return false;
  }

  GetCommState(h, &dcb);
  dcb.DCBlength = sizeof(dcb);
  dcb.BaudRate = speed;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fOutxCtsFlow = FALSE; //TRUE;
  if (!SetCommState(h, &dcb)) {
    wperror("SetCommState");
    CloseHandle(h);
    return false;
  }

  SetupComm(h, 4096, 16);

  ZeroMemory(&ct, sizeof(ct));
  ct.ReadTotalTimeoutMultiplier = 1;
  ct.ReadTotalTimeoutConstant = 5000;
  if (!SetCommTimeouts(h, &ct)) {
    wperror("SetCommTimeouts");
    CloseHandle(h);
    return false;
  }

  file = h;

#else

  int fd;

  // Open the serial port device
  fd = ::open(portname, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    wperror(portname);
    return false;
  }

  // Save old serial port configuration to keep it on exit
  tcgetattr(fd, &oldtio);

  // Set the new configuration: specified baud rate, no parity, 8 bits
  struct termios iop;
  memset(&iop, 0, sizeof(iop));
  iop.c_cflag &= ~PARENB;                                                       // Disable parity bit(s)
  iop.c_cflag &= ~CSTOPB;
  iop.c_cflag &= ~CSIZE;
  iop.c_cflag |= CS8;                                                           // Set an 8 bit frame
  iop.c_cflag |= CREAD;
  iop.c_cflag |= CLOCAL;
  iop.c_cc[VTIME] = 0;
  iop.c_cc[VMIN] = 1;
  if (cfsetospeed(&iop, speed) < 0) {
    wperror("cfsetospeed");  // typical error: 19200 instead of B19200...
    ::close(fd);
    return false;
  }
  cfsetispeed(&iop, speed);
  if (tcsetattr(fd, TCSANOW, &iop)<0) {
    wperror("tcsetattr");
    ::close(fd);
    return false;
  }
  file = fd;
#endif

  connected = true;
  return true;
}

void CSerialRemoteRegs::close()
{
  mutex_lock(&mutex);
#ifdef _WIN32
  if (file != NULL && file != INVALID_HANDLE_VALUE) {
    CloseHandle(file);
    file = NULL;
  }
#else
  if (connected) {
    tcsetattr(file, TCSAFLUSH, &oldtio);
    ::close(file);
  }
  file = -1;
#endif
  connected = false;
  mutex_unlock(&mutex);
}

bool CSerialRemoteRegs::sync()
{
  uint8_t b(0xff);

  mutex_lock(&mutex);
  for (int i(0); i<24; i++) {
    if (!write_n(file, &b, 1)) {
      // printf("Falsew\n");
      wperror("write_n");
      mutex_unlock(&mutex);
      return false;
    }else{
    	// printf("Truew\n");
    }
  }
  b = 0xAA;
  write_n(file, &b, 1);
  do {
    if (!read_n(file, &b, 1)) {
      // printf("Falser\n");
      wperror("read_n");
      mutex_unlock(&mutex);
      return false;
    }else{
    	// printf("Truer\n");
    }
  } while (b != 0xAA && b != 0x55);      // accepts both 0xAA(radio interface) and 0x55(as implemented by ARM-side of radio protocol)
  mutex_unlock(&mutex);
  return true;
}

bool CSerialRemoteRegs::reg_op(const uint8_t op, const uint16_t addr, const uint8_t* data, const int len, const bool lock)
{
  uint8_t req[32];

  req[0] = (op << 2) | ((addr & 0x300) >> 8);
  req[1] = (addr & 0xFF);
  for (int i(0); i < len; i++) {
    req[i+2] = data[i];
  }

  if (lock) {
    mutex_lock(&mutex);
  }
  if (!write_n(file, req, len + 2)) {
    wperror("write_n");
    if (lock) {
      mutex_unlock(&mutex);
    }
    return false;
  }

  if (!read_n(file, req, 1)) {
    wperror("read_n");
    if (lock) {
      mutex_unlock(&mutex);
    }
    return false;
  }

  if (lock) {
    mutex_unlock(&mutex);
  }

  return (req[0] == ACK);
}

bool CSerialRemoteRegs::get_reg_b(const uint16_t addr, uint8_t& res)
{
  uint8_t buf;

  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_8, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &buf, 1)) {
    wperror("read_n");
    mutex_unlock(&mutex);
    return false;
  }
  res = buf;
  mutex_unlock(&mutex);
  return true;
}

bool CSerialRemoteRegs::get_reg_w(const uint16_t addr, uint16_t& res)
{
  uint16_t buf;

  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_16, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &buf, 2)) {
    wperror("read_n");
    mutex_unlock(&mutex);
    return false;
  }
  res = buf;
  mutex_unlock(&mutex);
  return true;
}

bool CSerialRemoteRegs::get_reg_dw(const uint16_t addr, uint32_t& res)
{
  uint32_t buf;

  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_32, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &buf, 4)) {
    wperror("read_n");
    mutex_unlock(&mutex);
    return false;
  }
  res = buf;
  mutex_unlock(&mutex);
  return true;
}

bool CSerialRemoteRegs::get_reg_mb(const uint16_t addr, uint8_t* data, uint8_t& len)
{
  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_MB, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &len, 1)) {
    wperror("read_n");
    len = 0;
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, data, len)) {
    wperror("read_n");
    len = 0;
    mutex_unlock(&mutex);
    return false;
  }

  mutex_unlock(&mutex);
  return true;
}

uint8_t CSerialRemoteRegs::get_reg_b(const uint16_t addr)
{
  uint8_t result;
  if (!get_reg_b(addr, result)) {
    return 0xFF;
  } else {
    return result;
  }
}

uint16_t CSerialRemoteRegs::get_reg_w(const uint16_t addr)
{
  uint16_t result;
  if (!get_reg_w(addr, result)) {
    return 0xFFFF;
  } else {
    return result;
  }
}

uint32_t CSerialRemoteRegs::get_reg_dw(const uint16_t addr)
{
  uint32_t result;
  if (!get_reg_dw(addr, result)) {
    return 0xFFFFFFFF;
  } else {
    return result;
  }
}

bool CSerialRemoteRegs::set_reg_b(const uint16_t addr, const uint8_t val)
{
  return (reg_op(ROP_WRITE_8, addr, &val, 1, true));
}

bool CSerialRemoteRegs::set_reg_w(const uint16_t addr, const uint16_t val)
{
  return (reg_op(ROP_WRITE_16, addr, (const uint8_t*) &val, 2, true));
}

bool CSerialRemoteRegs::set_reg_dw(const uint16_t addr, const uint32_t val)
{
  return (reg_op(ROP_WRITE_32, addr, (const uint8_t*) &val, 4, true));
}

bool CSerialRemoteRegs::set_reg_mb(const uint16_t addr, const uint8_t* data, const uint8_t len)
{
  uint8_t buf[32];
  if (len > 29) {
    return false;
  }
  buf[0] = len;
  for (int i(0); i<len; i++) {
    buf[i+1] = data[i];
  }
  return (reg_op(ROP_WRITE_MB, addr, buf, len + 1, true));
}
