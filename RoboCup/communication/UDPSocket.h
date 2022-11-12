#ifndef __UDPSocket_H__
#define __UDPSocket_H__

#ifdef WIN32
#include <winsock2.h>
#else
#include <cstring>

#include <arpa/inet.h>

#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include "Types.h"
#include <iostream>

class UDPSocket {
public:
  UDPSocket();
  ~UDPSocket();

  static UDPSocket &instance();
  void Initial(const char *host, int port);

  int Receive(char *msg);
  int Send(const char *msg);

private:
  bool mIsInitialOK;
  sockaddr_in mAddress;

#ifdef WIN32
  SOCKET mSockfd;
#else
  int mSockfd;
#endif
};

#endif
