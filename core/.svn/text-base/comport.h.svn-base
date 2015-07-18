/**
    file: comport.h

    Copyright (C) 2012 Dr. Cirilo Bernardo (cjh.bernardo@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>

    For licensing under different terms or for support, email cjh.bernardo@gmail.com

*/

/*
    Implementation of a generic serial communications port
    This module is asynchronous and provides the user with an
    easy means of setting the communications parameters and
    selecting the physical device.

    This code only supports a "three-wire interface" (GND, Tx, Rx)
    with no handshaking.

    For valid speeds on Linux systems, see /usr/include/asm/termbits.h
    and search for B4800 for a start.
*/

#ifndef __COMPORT_H__
#define __COMPORT_H__

#include <sys/termios.h>
#include <cstdio>

#include "commif.h"

#ifndef MAX_PATH
#ifdef PATH_MAX
#define MAX_PATH PATH_MAX
#else
#define MAX_PATH 260
#endif
#endif

// size of the circular buffer; must be 2^n
#define PBUF_SIZE (256)
// circular buffer mask; must be TBUF_SIZE -1
#define PBUF_MASK (255)

namespace com {

    /// Error message buffer length
    const int ERRLEN = 512;

    class COMPORT : public COMMIF
    {
    private:
        int     fd;                 // Serial port file descriptor number
        struct  termios oldterm;    // Remember the previous settings
        bool    hasterm;            // true when 'oldterm' is set
        struct  COMPARAMS params;   // Remember the port settings for 'reopen'
        char    portname[MAX_PATH]; // Remember the name of the opened device
        char    errmsg[ERRLEN];     // Textual error information
        // internal circular buffer to support faster reading of
        // data when non-zero delimeters are involved
        char tbuf[PBUF_SIZE];
        int cbird;  // Current byte to be read
        int cbiwr;  // Current byte to be written
        inline int bufrdlen(void) { return (cbiwr - cbird) & PBUF_MASK; }
        inline int bufwrlen(void) { return PBUF_MASK - ((cbiwr - cbird) & PBUF_MASK); }
        inline void bufclr(void) { cbird = cbiwr = 0; }

    public:
        COMPORT();
        ~COMPORT();

        /// Open port using parameters supplied.
        /// @param port device path (example: /dev/ttyS0)
        /// @param params communication parameters
        /// @return 0 for success, otherwise -1
        int Open(const char *portname, const COMPARAMS *params = NULL,
                const char *lockid = NULL);
        /// Change the port's baud rate
        int SetBaud(speed_t speed, int timeout = 0, const char* lockid = NULL);

        /// Close the port
        int Close(const char *lockid = NULL);

        /// Close and reopen the port
        /// @return 0 for success, otherwise -1
        int Reopen(const char *lockid = NULL);

        /// Get the port name
        /// @return port name (will be \0 if no port has been opened)
        const char *GetPortName(void) { return portname; }

        /// Check if the port is open
        /// @return true if the port is open
        bool IsOpen(void) { return (fd == -1) ? false : true; }

        /// Write a string to the port
        /// @return 0 for success, otherwise -1 with errno set; if errno = EAGAIN, try sending again later
        int Write(const char* data, int len, int timeout = 0,
                const char* lockid = NULL);

        /// Read data; call 'Select()' to check if data is available.
        /// @param data preallocated buffer
        /// @param len  maximum bytes to read
        /// @return number of bytes read (may be 0) or -1 for fault
        int Read(char *data, int len, int timeout = 0,
                char delim = 0, const char *lockid = NULL);

        /// Write data and wait for a response to be read
        int WriteRead(const char* dataout, int lenout, char* datain,
                    int lenin, int timeout, char delim = 0,
                    const char* lockid = NULL);

        /// Flush the port's input buffer and drain the output buffer
        int Flush(const char *lockid = NULL);

        /// Drain the port's transmit buffer
        int Drain(const char *lockid = NULL);

        /// Check if data is available to be read
        /// @param  duration milliseconds to wait for data
        /// @return 0 if there is data, -1 for fault or timeout
        int Select(unsigned int duration);

        /// Get a pointer to the error message
        /// @return error string
        const char *GetError(void);

        /// Clear the error string
        void ClearError(void);

        /// Obtain exlusive access to the port
        inline int Lock(const char* /*lockid*/ = NULL, int /*timeout*/ = 0) { return 0; }
        /// Relinquish exclusive access to the port
        inline int Unlock(const char* /*lockid*/ = NULL) { return 0; }
    };  // class COMPORT

}; // namespace com

#endif
