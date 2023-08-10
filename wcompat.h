#ifndef __WCOMPAT_H
#define __WCOMPAT_H

#ifdef _WIN32
#include <windows.h>
#include <winsock.h>
#include <stdio.h>
#include <stdlib.h>
#define PORT_SPEED   57600
#define usleep(A)    Sleep((A)/1000)
#define wperror(STR) do													\
	{																	\
		DWORD err = GetLastError ();									\
		char *msg = NULL;												\
		FormatMessage (FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | 60, \
		               NULL,											\
		               err,												\
		               LANG_NEUTRAL,									\
		               (char *) &msg,									\
		               2048,											\
		               NULL);											\
		fprintf (stderr, "%s: %s\n", (STR), msg);						\
		LocalFree(msg);													\
	} while (0)
typedef SOCKET socket_t;

#else
#include <unistd.h>
#include <sys/socket.h>	     /* for socket(), bind(), and connect() */
#include <arpa/inet.h>	     /* for sockaddr_in and inet_ntoa() */
#include <pthread.h>	     /* for POSIX threads */
#define PORT_SPEED   B57600
#define wperror      perror
#define closesocket  ::close
typedef unsigned int socket_t;
#define INVALID_SOCKET (socket_t) (~0)
#define SOCKET_ERROR   -1
#endif

#endif
