/*
    file: comport.cpp

    Single user implementation of the COMMIF interface.

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
    This module is provides the user with an easy means of setting the
    communications parameters and selecting the physical device.

    This code only supports a "three-wire interface" (GND, Tx, Rx)
    with no handshaking.

 */

#include <sys/select.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

#include "comport.h"

using namespace com;

COMPORT::COMPORT()
{
    fd = -1;
    portname[0] = 0;
    errmsg[0] = 0;
    hasterm = false;
    bufclr();
}



COMPORT::~COMPORT()
{
    if (fd >= 0) Close();
    fd = -1;
}



int
COMPORT::Open(const char *portname, const COMPARAMS *params, const char * /*lockid*/)
{
    if(portname == NULL)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): invalid port name (NULL)",
                 __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    COMPARAMS lparams;
    if (params) lparams = *params;

    if (fd >= 0) Close();

    fd = open(portname, O_RDWR|O_NOCTTY|O_NONBLOCK|O_NDELAY);
    if (fd == -1)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): could not open port '%s': %s",
                 __FILE__, __LINE__, __FUNCTION__, portname, strerror(errno));
        return -1;
    }

    this->params = lparams;
    if (SetBaud(lparams.speed))
    {
        Close();
        char tmpmsg[ERRLEN];
        snprintf(tmpmsg, ERRLEN, "%s:%d: %s(): could not set requested speed (%d) on port '%s'\n%s",
                 __FILE__, __LINE__, __FUNCTION__, lparams.speed, portname, errmsg);
        snprintf(errmsg, ERRLEN, "%s", tmpmsg);
        return -1;
    }

    // the port is open and for non-blocking operation
    snprintf(this->portname, MAX_PATH, "%s", portname);
    return 0;
}



// Close the port
int COMPORT::Close(const char * /*lockid*/)
{
    if (fd == -1) return -1;
    // Restore previous settings
    tcsetattr(fd, TCSADRAIN, &oldterm);
    close(fd);
    fd = -1;
    hasterm = false;
    return 0;
}



// Write a string to the RS485 port
int
COMPORT::Write(const char* data, int len, int /*timeout*/,
               const char* /*lockid*/)
{
    if (len <= 0)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): data length < 0 : %d",
                __FILE__, __LINE__, __FUNCTION__, len);
        return -1;
    }

    if (data == NULL)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): invalid data pointer (NULL)",
                 __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    if (fd == -1)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): port not open",
                 __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    errno = 0;
    ssize_t ntx = 0;
    ssize_t rtx = len;
    ssize_t bsent = 0;
    while ((rtx) && ((bsent = write(fd, &data[ntx], rtx)) > 0))
    {
        ntx += bsent;
        rtx -= bsent;
        tcdrain(fd);
    }
    if (bsent == -1)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): write incomplete (req: %d, sent %d): %s",
                 __FILE__, __LINE__, __FUNCTION__, len, ntx, strerror(errno));
        if (ntx > 0) return ntx;
        return -1;
    }

    if ( ntx != len)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): info: write incomplete (req: %d, sent %d): %s",
                 __FILE__, __LINE__, __FUNCTION__, len, ntx, strerror(errno));
#ifdef DEBUGMSG
        fprintf(stderr, "%s\n", errmsg);
#endif
    }

    return (int) ntx;
}



// Read max [len] bytes of data into a user buffer
// int COMPORT::Read(char *data, int len)
int
COMPORT::Read(char *data, int len, int timeout,
                 char delim, const char * /*lockid*/)
{
    if (fd == -1) {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): port not open",
                 __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }
    if (len <= 0) {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): invalid buffer length: %d",
                 __FILE__, __LINE__, __FUNCTION__, len);
        return -1;
    }

    struct timeval tov, now;
    if (timeout < 0)
    {
        fprintf(stderr, "%s:%d: %s(): BUG: timeval < 0\n",
                __FILE__, __LINE__, __FUNCTION__);
        timeout = 0;
    }
    if (timeout != 0)
    {
        gettimeofday(&tov, NULL);
        if (timeout >= 1000)
        {
            tov.tv_sec += timeout / 1000;
        }
        tov.tv_usec += (timeout % 1000) * 1000;
        if (tov.tv_usec >= 1000000)
        {
            tov.tv_sec += 1;
            tov.tv_usec %= 1000000;
        }
    }
    int nb = 0; // number of bytes available
    int val, idx;
    idx = 0;

    if ((bufrdlen() > 0) && (delim))
    {
        // read in avaliable data
        int dlen = bufrdlen();
        dlen = bufrdlen();
        if (dlen > nb) dlen = nb;
        for (val = 0; val < dlen; ++val)
        {
            data[idx] = tbuf[cbird++];
            cbird &= PBUF_MASK;
            if (data[idx++] == delim) return idx;
        }
    }

    do
    {
        if (timeout != 0)
        {
            int i;
            while ((i = Select(timeout)) == -1)
            {
                if (errno == EAGAIN) return 0;
                if (errno == EINTR) continue;
                char msg[ERRLEN];
                snprintf(msg, ERRLEN, "%s", errmsg);
                snprintf(errmsg, ERRLEN, "%s:%d: %s(): Select() failed:\n\t%s\n",
                        __FILE__, __LINE__, __FUNCTION__, msg);
                return -1;
            }
            if (i == 0) return idx;   // timed out
        }

        if (ioctl(fd, FIONREAD, &nb) == -1) nb = (len - idx);
        if (nb > (len - idx)) nb = (len - idx);

        errno = 0;
        if (delim)
        {
            int nbf = nb;
            int dlen;
            int ilen;
            while (nbf)
            {
                // read as much data as we can into the circular buffer
                dlen = bufwrlen();
                if (dlen > nbf) dlen = nbf;
                if ((cbiwr + dlen) > PBUF_SIZE)
                {
                    // read until the end of the buffer
                    if ((ilen = read(fd, &tbuf[cbiwr], PBUF_SIZE - cbiwr)) < 0) break;
                    cbiwr = (cbiwr + ilen) & PBUF_MASK;
                    if (cbiwr != 0)
                    {
                        fprintf(stderr, "%s:%d: %s(): unexpected branch: could not read available data: %s\n",
                                 __FILE__, __LINE__, __FUNCTION__, strerror(errno));
                        dlen = PBUF_SIZE - cbiwr;
                    }
                }
                // read in what remains
                if ((ilen = read(fd, &tbuf[cbiwr], dlen)) > 0)
                {
                    cbiwr = (cbiwr + ilen) & PBUF_MASK;
                }
                // transfer from the circular buffer to the user data space
                dlen = bufrdlen();
                for (val = 0; val < dlen; ++val, --nbf)
                {
                    data[idx] = tbuf[cbird++];
                    cbird &= PBUF_MASK;
                    if (data[idx++] == delim) return idx;
                }
                if (ilen == -1) break; // there was a read error; we can read no more
            }
        }
        else
        {
            val = read(fd, &data[idx], nb);
            if (val > 0) idx += val;
        }

        if (idx == len) return idx;

        if ((val == -1) && (errno != EAGAIN) && (errno != EINTR))
        {
            snprintf(errmsg, ERRLEN, "%s:%d: %s(): failed: %s",
                     __FILE__, __LINE__, __FUNCTION__, strerror(errno));
            return -1;
        }
        // check timeout
        gettimeofday(&now, NULL);
        if (now.tv_sec > tov.tv_sec) return idx;
        if ((now.tv_sec == tov.tv_sec) && (now.tv_usec >= tov.tv_usec)) return idx;
        timeout = (tov.tv_sec - now.tv_sec)*1000
                + ((tov.tv_usec - now.tv_usec)/1000);
    } while (timeout);

    return idx;
}



// Write data and wait for a response to be read
int
COMPORT::WriteRead(const char* dataout, int lenout, char* datain,
                   int lenin, int timeout, char delim,
                   const char* /*lockid*/)
{
    if (Write(dataout, lenout) != lenout)
    {
        char msg[ERRLEN];
        snprintf(msg, ERRLEN, "%s", errmsg);
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): WriteRead() failed on Write:\n\t%s\n",
                 __FILE__, __LINE__, __FUNCTION__, msg);
        return -1;
    }

    int bread = Read(datain, lenin, timeout, delim);
    if (bread == -1)
    {
        char msg[ERRLEN];
        snprintf(msg, ERRLEN, "%s", errmsg);
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): WriteRead() failed on Read:\n\t%s\n",
                 __FILE__, __LINE__, __FUNCTION__, msg);
    }
    return bread;
}



int
COMPORT::Reopen(const char * /*lockid*/)
{
    if (Open(portname, &params))
    {
        char tmpmsg[ERRLEN];
        snprintf(tmpmsg, ERRLEN, "%s:%d: %s(): failed:\n%s",
                 __FILE__, __LINE__, __FUNCTION__, errmsg);
        snprintf(errmsg, ERRLEN, "%s", tmpmsg);
        return -1;
    }
    return 0;
}



int
COMPORT::Flush(const char * /*lockid*/)
{
    if (fd < 0) return -1;
    tcdrain(fd);
    tcflush(fd, TCIFLUSH);
    bufclr();
    return 0;
}



int
COMPORT::Drain(const char * /*lockid*/)
{
    if (fd < 0) return -1;
    tcdrain(fd);
    return 0;
}



int
COMPORT::Select(unsigned int duration)
{
    // if there is no open port just pretend there is nothing to do
    if (fd == -1) return 0;
    struct timeval ts;
    if (duration >= 1000)
    {
        ts.tv_sec = duration / 1000;
        ts.tv_usec = (duration % 1000) * 1000;
    }
    else
    {
        ts.tv_sec = 0;
        ts.tv_usec = duration * 1000;
    }

    fd_set rdfd;
    FD_ZERO(&rdfd);
    FD_SET(fd, &rdfd);

    return select(fd+1, &rdfd, NULL, NULL, &ts);
}

int
COMPORT::SetBaud(speed_t speed, int /*timeout*/, const char* /*lockid*/)
{
    termios newterm;
    tcgetattr(fd, &newterm);
    if (!hasterm)
    {
        oldterm = newterm;
        hasterm = true;
    }

    // set to RAW mode
    newterm.c_iflag =0;
    newterm.c_oflag = 0;
    newterm.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    newterm.c_cflag &= ~CSIZE;

    switch (params.data)
    {
        case 7:
            newterm.c_cflag &= ~CS8;
            newterm.c_cflag |= CS7;
            break;
        case 8:
            newterm.c_cflag &= ~CS7;
            newterm.c_cflag |= CS8;
            break;
        default:
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): unsupported # of data bits: %d\n",
                 __FILE__, __LINE__, __FUNCTION__, params.data);
        return -1;
    }

    if (params.parity)
    {
        newterm.c_cflag |= PARENB;
        if (params.odd)
            newterm.c_cflag |= PARODD;
        else
            newterm.c_cflag &= ~PARODD;
    }
    else
    {
        newterm.c_cflag &= ~PARENB;
        newterm.c_cflag &= ~PARODD;
    }

    if (params.stop == 2)
        newterm.c_cflag |= CSTOPB;
    else
        newterm.c_cflag &= ~CSTOPB;

    // Use 'local' mode (no modem control lines) and enable reads
    newterm.c_cflag |= CLOCAL | CREAD;

    cfsetospeed(&newterm, speed);
    cfsetispeed(&newterm, speed);

    // Impose the new settings
    if(tcsetattr(fd, TCSADRAIN, &newterm) == -1)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): could not set comm parameters: %s",
                 __FILE__, __LINE__, __FUNCTION__, strerror(errno));
        return -1;
    }

    // verify that the speed matches what was requested
    tcgetattr(fd, &newterm);
    speed_t NEWSPEED = cfgetospeed(&newterm);
    params.speed = NEWSPEED;
    if (NEWSPEED != speed)
    {
        snprintf(errmsg, ERRLEN, "%s:%d: %s(): speed not supported by hardware",
                 __FILE__, __LINE__, __FUNCTION__);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return 0;
}


const char* COMPORT::GetError(void)
{
    return errmsg;
}

void COMPORT::ClearError(void)
{
    errmsg[0] = 0;
}
