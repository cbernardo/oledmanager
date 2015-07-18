/**
    file: commif.h

    Serial Communications Interface

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

#ifndef __COMMIF_H__
#define __COMMIF_H__

#include <termios.h>

namespace com {

    /// Parameters for the serial port communications
    struct COMPARAMS
    {
        speed_t speed;
        int data;
        bool parity;
        bool odd;
        int stop;
        COMPARAMS()
        {
            // note: do not change these defaults; many older software
            // modules rely on these historical settings
            speed = B19200;
            data = 8;
            parity = false;
            odd = false;
            stop = 1;
        }
    #ifdef DEBUGMSG
        void PrintSettings(void)
        {
            fprintf(stderr, "Port Settings:\n");
            fprintf(stderr, "\tspeed : %d\n", speed);
            fprintf(stderr, "\tdata  : %d\n", data);
            fprintf(stderr, "\tparity: %s\n", parity ? (odd ? "odd" : "even") : "none");
            fprintf(stderr, "\tstop  : %d\n", stop);
        }
    #else
        inline void print_settings(void) { }
    #endif
    };

    class COMMIF
    {

    public:
        virtual int Open(const char *portname, const COMPARAMS *params = NULL,
                        const char *lockid = NULL) = 0;
        virtual int Reopen(const char *lockid = NULL) = 0;
        virtual int Close(const char *lockid = NULL) = 0;
        virtual int Flush(const char *lockid = NULL) = 0; ///< drain output and flush (discard) input
        virtual int Drain(const char *lockid = NULL) = 0; ///< drain output

        /// Check if data is available to be read
        /// @param  duration milliseconds to wait for data
        /// @return 0 if there is data, -1 for fault or timeout
        virtual int Select(unsigned int duration) = 0;

        /// Change the port's baud rate
        virtual int SetBaud(speed_t speed, int timeout = 0,
                            const char* lockid = NULL) = 0;

        /// Read up to <len> characters or up to and including <delim> if it is not zero
        /// @return length of string or -1 if something went wrong; if the return is
        ///     -1 and errno is EAGAIN, the operation timed out; if errno is EACCESS
        ///     the routine is blocked by a Lock()
        virtual int Read(char *data, int len, int timeout,
                        char delim = 0, const char *lockid = NULL) = 0;

        /// Write data to the port and return.  Since multiple
        /// threads and modules may be using the port, use the \p WriteRead
        /// method if a single response is expected. To lock out other users
        /// for a period, use \p Lock to obtain a lock, use \p Read and \p Write
        /// as required, and call Unlock() when done.
        virtual int Write(const char* data, int len, int timeout,
                        const char* lockid = NULL) = 0;

        /// Write data and wait for a response to be read; other users are prevented
        /// from accessing the port until data is received or a timeout occurs.
        virtual int WriteRead(const char* dataout, int lenout, char* datain,
                            int lenin, int timeout, char delim = 0,
                            const char* lockid = NULL) = 0;

        /// Obtain exlusive access to the port
        virtual int Lock(const char* lockid, int timeout) = 0;
        /// Relinquish exclusive access to the port
        virtual int Unlock(const char* lockid) = 0;

        /// Retrieve the error string
        virtual const char *GetError(void) = 0;
        /// Clear the error string
        virtual void ClearError(void) = 0;

        /// Get the port name
        /// @return port name (will be \0 if no port has been opened)
        virtual const char *GetPortName(void) = 0;

        /// Check if the port is open
        /// @return true if the port is open
        virtual bool IsOpen(void) = 0;
    };  // class COMMIF

};  // namespace com
#endif
