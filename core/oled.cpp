/**
    file: oled.cpp

    Implementation of the uAMOLED/uLCD driver for 4DSystems' PICASO
    series graphics processor.  This is the serial version
    only, not the 4D Graphics Language version.

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
    Notes:

    1. On initialization, touchpad is OFF by default.  We can track the
        activation of the touchpad and pass the user an error result if
        a touch command was issued while the device was inactive.

    2. Be careful when using the Touch features since they may block
        indefinitely.  The safest thing to do is to use WaitTouch()
        with a short timeout (~1s?) followed by the Status and Coordinates
        modes of GetTouchCoordinates().

    3. With the Suspend (Sleep) functions, only specify one of
        P0/P1/Touch/Serial for the wakeup condition. To be safe,
        only set 1 bit of the mode rather than combining bits.

    Improvements to consider:

    1. When a partial coordinate response is received we should enforce
        a timeout; it makes no sense to wait forever once a response has
        begun.

*/


#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <linux/limits.h>
#include <string.h>
#include <stdlib.h>
#include <sched.h>

#include "oled.h"
#include "comport.h"

using namespace disp;

#define CHECK_READY(arg) do {\
    if ((state != LCD_BUSY) && (state != LCD_IDLE)) {\
        fprintf(stderr, __FILE__ ":%d: " arg ": device not ready\n", __LINE__);\
        errno = ENOTCONN;\
        return -1;\
} } while (0)

#define ERROUT(fmt, args...) fprintf(stderr, "%s:%d: %s(): " fmt,\
    __FILE__, __LINE__, __FUNCTION__,##args)

#define ERRMSG(fmt, args...) snprintf(errmsg, PGDERRLEN, "%s:%d: %s(): " fmt,\
    __FILE__, __LINE__, __FUNCTION__,##args)

#define CHECK_INACTIVE do {\
    if (state == LCD_INACTIVE) {\
        ERRMSG("display inactive");\
        return -1; }\
    } while (0)

#define CHECK_BUSY do {\
    if (state == LCD_BUSY) {\
        ERRMSG("display busy");\
        return -1; }\
    } while (0)

// thread callback routine
void *procthread(void *arg)
{
    PGD *pp;
    if (!arg)
    {
        ERROUT("argument is NULL");
        exit(-1);
    }

    pp = (PGD *)arg;
    while (!pp->Process());

    pthread_exit(NULL);
}



PGD::PGD()
{
    halt = 0;
    state = LCD_INACTIVE;
    procloop = 0;
    errmsg[0] = 0;
    baud = DB_9600;
    curcmd = PG_NONE;
    curdata = NULL;
    brcv = 0;
    callback = NULL;
    usrobj = NULL;
}

PGD::~PGD()
{
    Close();
    callback = NULL;
    usrobj = NULL;
    return;
}



int
PGD::SetCallback(void (*cb)(class PGD*, PGDCMD, bool, void *), void *obj)
{
    CHECK_BUSY;
    callback = cb;
    usrobj = obj;
    return 0;
}



/**************************************
    serial port management routines
**************************************/
// open a port and connect
int
PGD::Connect(const char *portname)
{
    CHECK_BUSY;
    curcmd = PG_NONE;
    curdata = NULL;
    brcv = 0;

    if (port.IsOpen()) Close();

    com::COMPARAMS parm;
    /* W32 */
    parm.speed = B9600;

    if (port.Open(portname, &parm))
    {
        ERRMSG("could not open port (see below)\n%s", port.GetError());
        return -1;
    }

    // as per the manual, waste 500ms before communicating
    /* W32 */
    struct timeval tov;
    struct timeval now;

    gettimeofday(&now, NULL);
    tov = now;

    tov.tv_usec += 500000;
    if (tov.tv_usec >= 1000000)
    {
        tov.tv_sec += 1;
        tov.tv_usec %= 1000000;
    }

    while ((now.tv_sec < tov.tv_sec) || ((now.tv_sec == tov.tv_sec)
            && (now.tv_usec < tov.tv_sec)))
    {
        usleep(50000);
        gettimeofday(&now, NULL);
    }

    if (autobaud()) return -1;

    if(SetBaud(DB_MAX)) ERROUT("\n%s\n", errmsg);

    if (pthread_create(&procloop, NULL, procthread, this))
    {
        ERRMSG("could not create processing thread: %s\n", strerror(errno));
        Close();
        return -1;
    }

    return 0;
}





// close a port
void
PGD::Close(void)
{
    if (!port.IsOpen()) return;
    errmsg[0] = 0;
    halt = true;

    if (state == LCD_BUSY)
    {
        ERRMSG("port is closing");
        PGDCMD tmpcmd = curcmd;
        curcmd = PG_NONE;
        curdata = NULL;
        state = LCD_IDLE;
        if (callback) callback(this, tmpcmd, false, usrobj);
    }

    if (state != LCD_INACTIVE)
    {
        if (SetBaud(DB_9600))
        {
            ERROUT("cannot restore default bitrate; device will require manual reset\n%s\n",
                  errmsg);
        }
    }

    port.Close();
    state = LCD_INACTIVE;
    /* W32 */
    if (procloop) pthread_join(procloop, NULL);

    return;
}


/*****************************************************
                 LOW LEVEL COMMANDS
*****************************************************/

int
PGD::autobaud(void)
{
    int i = 4;
    int res;
    while (i--)
    {
        port.Flush();
        if ((res = port.Write("U", 1)) < 0) usleep(20);
        if ((res >= 0) && (!waitACK(20))) break;
        if (i == 0)
        {
            ERRMSG("timed out, no ACK received");
            return -1;
        }
    }

    baud = DB_9600;
    /* W32 */
    portspeed = B9600;
    state = LCD_IDLE;
    return 0;
}


int
PGD::SetBaud(enum disp::DBAUD speed)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (speed == baud) return 0;    // nothing to be done

    // test if the selected speed is supported
    /* W32 */
    speed_t tspeed;
    switch (speed)
    {
#ifdef WIN32
        case DB_9600:
            tspeed = 9600;
            break;
        case DB_57600:
            tspeed = 57600;
            break;
        case DB_115200:
            tspeed = 115200;
            break;
        case DB_128000:
            tspeed = 128000;
            break;
        case DB_256000:
            tspeed = 256000;
            break;
#else
        case DB_9600:
            tspeed = B9600;
            break;
        case DB_57600:
            tspeed = B57600;
            break;
        case DB_115200:
            tspeed = B115200;
            break;
        case DB_128000:
        case DB_256000:
            ERRMSG("bitrate not supported in Linux: %d", speed);
            return -1;
#endif
        default:
            ERRMSG("unsupported bitrate: %d", speed);
            return -1;
    }
    if (port.SetBaud(tspeed))
    {
        ERRMSG("bitrate not supported on system/hardware (see below)\n%s",
               port.GetError());
        return -1;
    }
    usleep(50);
    if (port.SetBaud(portspeed) < 0)
    {
        ERRMSG("cannot revert to original bitrate");
        return -1;
    }
    usleep(50);

    char cmd[8] = "Q      ";
    cmd[1] = speed & 0xff;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 2)) != 2)
    {
        ERRMSG("failed to send SetBaud command (see below)\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    // the PICASO chip seems to always return 0xFF so we ignore the result
    // unless it is a NACK
    if (waitACKNACK(100) == 1)
    {
        ERRMSG("NACK on SetBaud() request");
        return 1;
    }

    if (port.SetBaud(tspeed))
    {
        ERRMSG("could not switch host bitrate after switching display bitrate;\n"
                "\n\tdisplay will require a manual reset. See message below.\n%s",
                port.GetError());
        return -2;
    }

    baud = speed;
    portspeed = tspeed;

    return 0;
}



int
PGD::Version(struct disp::PGDVER *ver, bool display)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char msg[8];
    int res;

    /* W32 */
    port.Flush();
    if (display)
        res = port.Write("V\x01", 2);
    else
        res = port.Write("V\x00", 2);

    if (res != 2)
    {
        ERRMSG("could not query version (see message below)\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    if (display)
        res = port.Read(msg, 5, 500);
    else
        res = port.Read(msg, 5, 50);

    if (res < 0)
    {
        ERRMSG("no response");
        return -1;
    }
    if (res != 5)
    {
        ERRMSG("incomplete response packet (%d bytes, 5 expected)", res);
        return -1;
    }

    if (ver)
    {
        switch (msg[0])
        {
            case 0:
            case 1:
            case 2:
                ver->display_type = msg[0];
                break;
            default:
                ver->display_type = DEV_UNKNOWN;
                break;
        }

        ver->hardware_rev = msg[1] & 0xff;
        ver->firmware_rev = msg[2] & 0xff;

        ver->hres = convertRes(msg[3]);
        ver->vres = convertRes(msg[4]);
    }

    return 0;
}


unsigned int
PGD::convertRes(char rescode)
{
    switch (rescode)
    {
        case '\x22':
            return 220;
        case '\x24':
            return 240;
        case '\x28':
            return 128;
        case '\x32':
            return 320;
        case '\x60':
            return 160;
        case '\x64':
            return 64;
        case '\x76':
            return 176;
        case '\x96':
            return 96;
    }
    ERROUT("unexpected resolution code: 0x%.2X\n", rescode);
    return 0;
}



int
PGD::ReplaceBackground(ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[8] = "B      ";
    cmd[2] = color & 0xff;
    cmd[1] = (color >> 8) & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 3)) != 3)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(2500);
}





int
PGD::Clear(void)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    port.Flush();
    int res;
    if ((res = port.Write("E", 1)) != 1)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        return -1;
    }

    return waitACKNACK(100);
}





int
PGD::Ctl(uchar mode, uchar value)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[8] = "Y      ";

    switch (mode)
    {
        case 0:
            // Backlight Control
            if ((value != 0) && (value != 1))
            {
                ERRMSG("invalid value for Backlight Control (%d); valid values are 0,1", value);
                return -1;
            }
            break;
        case 1:
            // Display ON/OFF
            if ((value != 0) && (value != 1))
            {
                ERRMSG("invalid value for Display ON/OFF (%d); valid values are 0,1", value);
                return -1;
            }
            break;
        case 2:
            // Contrast adjust
            break;
        case 3:
            // Display Shutdown/Powerup
            if ((value != 0) && (value != 1))
            {
                ERRMSG("invalid value for Display Powerup/Shutdown (%d); valid values are 0,1", value);
                return -1;
            }
            break;
        case 4:
            // Orientation
           if ((value < 1) || (value > 4))
            {
                ERRMSG("invalid value for Display Orientation (%d); valid values are 1..4", value);
                return -1;
            }
            break;
        case 5:
            // Touch Control
            if ((value < 0) || (value > 2))
            {
                ERRMSG("invalid value for Touch Control (%d); valid values are 0..2", value);
                return -1;
            }
            break;
        case 6:
            // image format
            if ((value != 0) && (value != 1))
            {
                ERRMSG("invalid value for Image Format (%d); valid values are 0,1", value);
                return -1;
            }
            break;
        case 8:
            // Protect FAT
            if ((value != 0) && (value != 2))
            {
                ERRMSG("invalid value for Protect FAT (%d); valid values are 0,2", value);
                return -1;
            }
            break;
        default:
            ERRMSG("invalid value for Control Mode (%d); valid values are 0..6,8", value);
            return -1;
    }

    cmd[1] = mode;
    cmd[2] = value;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 3) != 3))
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}



int
PGD::SetVolume(uchar value)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[8] = "v      ";

    if ((value < 0) || (value > 0xff)
         || ((value > 3) && (value < 8))
         || ((value > 127) && (value < 0xfd)))
    {
        ERRMSG("invalid value for Volume Control (%d); valid values are 0..3, 8..127, 253..255", value);
        return -1;
    }

    cmd[1] = value;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 2)) != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}





int
PGD::Suspend(uchar options, uchar duration)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[8] = "Z      ";

    if ((options & 0x10))
    {
        ERRMSG("invalid value for Suspend (Sleep); bit 4 (0x10) must not be set");
        return -1;
    }

    if ((options & 0x2f) == 0x22)
    {
        // incompatible: Wake-Up on touch + Touch Off
        ERRMSG("Wake on Touch was specified with Touch OFF\n");
        return -1;
    }

    cmd[1] = options;
    cmd[2] = duration;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 3)) != 3)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    switch (waitACKNACK(100))
    {
        case 0:
            return 0;
        case 1:
            return 1;
        case 2:
            if (options & 0x0f)
            {
                curcmd = PG_SLEEP;
                curdata = NULL;
                state = LCD_BUSY;
            }
            return 2;
    }

    return -1;
}





int
PGD::ReadPin(uchar pin, uchar *status)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[8] = "i      ";

    if ((pin < 0) || (pin > 15))
    {
        ERRMSG("invalid pin (%d); valid values are 0..15", pin);
        return -1;
    }

    if (!status)
    {
        ERRMSG("invalid pointer to status byte");
        return -1;
    }

    cmd[1] = pin;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 2)) != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    res = port.Read(cmd, 1, 100);
    if (res != 1)
    {
        ERRMSG("no response (see below)\n%s", port.GetError());
        return -1;
    }

    *status = cmd[0];
    return 0;
}




int
PGD::WritePin(uchar pin, uchar value)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[8] = "y      ";

    if ((pin < 0) || (pin > 15))
    {
        ERRMSG("invalid pin (%d); valid values are 0..15", pin);
        return -1;
    }

    if ((value != 0) && (value != 1))
    {
        ERRMSG("invalid pin value (%d); valid values are 0,1", value);
        return -1;
    }

    cmd[1] = pin;
    cmd[2] = value;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 3)) != 3)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}





int
PGD::ReadBus(uchar *status)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[2] = "a";

    if (!status)
    {
        ERRMSG("invalid pointer to status byte");
        return -1;
    }

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 1)) != 1)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        return -1;
    }

    res = port.Read(cmd, 1, 100);
    if (res != 1)
    {
        ERRMSG("no response (see below)\n%s", port.GetError());
        return -1;
    }

    *status = cmd[0];
    return 0;
}



int
PGD::WriteBus(uchar value)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[2] = "W";

    cmd[1] = value;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 2)) != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }
    return waitACKNACK(100);
}



/*****************************************************
               GRAPHICS COMMANDS
*****************************************************/

int
PGD::AddBitmap(uchar group, uchar index, const uchar *data, int datalen)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[160];

    switch (group)
    {
        case 0:
            // 8x8 bitmap
            if (datalen != 8)
            {
                ERRMSG("invalid data length for group 0, length must be 8");
                return -1;
            }
            if ((index < 0) || (index > 63))
            {
                ERRMSG("invalid index for group 0, index must be 0..63");
                return -1;
            }
            break;
        case 1:
            // 16x16 bitmap
            if (datalen != 32)
            {
                ERRMSG("invalid data length for group 1, length must be 32");
                return -1;
            }
            if ((index < 0) || (index > 15))
            {
                ERRMSG("invalid index for group 1, index must be 0..15");
                return -1;
            }
            break;
        case 2:
            // 32x32 bitmap
            if (datalen != 128)
            {
                ERRMSG("invalid data length for group 2, length must be 128");
                return -1;
            }
            if ((index < 0) || (index > 7))
            {
                ERRMSG("invalid index for group 2, index must be 0..7");
                return -1;
            }
            break;
        default:
            ERRMSG("invalid group (%d); valid values are 0..2", group);
            return -1;
    }

    cmd[0] = 'A';
    cmd[1] = group;
    cmd[2] = index;
    memcpy(&cmd[3], data, datalen);
    datalen += 3;

    port.Flush();
    int wr;
    if ((wr = port.Write(cmd, datalen)) != datalen)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (wr > 0) return -2;
        return -1;
    }
    port.Drain();

    return waitACKNACK(200);
}






int
PGD::DrawBitmap(uchar group, uchar index, ushort x, ushort y, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[10];

    switch (group)
    {
        case 0:
            // 8x8 bitmap
            if ((index < 0) || (index > 63))
            {
                ERRMSG("invalid index for group 0, index must be 0..63");
                return -1;
            }
            break;
        case 1:
            // 16x16 bitmap
            if ((index < 0) || (index > 15))
            {
                ERRMSG("invalid index for group 1, index must be 0..15");
                return -1;
            }
            break;
        case 2:
            // 32x32 bitmap
            if ((index < 0) || (index > 7))
            {
                ERRMSG("invalid index for group 2, index must be 0..7");
                return -1;
            }
            break;
        default:
            ERRMSG("invalid group (%d); valid values are 0..2", group);
            return -1;
    }

    cmd[0] = 'D';
    cmd[1] = group;
    cmd[2] = index;
    cmd[3] = (x >> 8) & 0xff;
    cmd[4] = x & 0xff;
    cmd[5] = (y >> 8) & 0xff;
    cmd[6] = y & 0xff;
    cmd[7] = (color >> 8) & 0xff;
    cmd[8] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 9)) != 9)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}





int
PGD::Circle(ushort x, ushort y, ushort radius, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[10];

    cmd[0] = 'C';
    cmd[1] = (x >> 8) & 0xff;
    cmd[2] = x & 0xff;
    cmd[3] = (y >> 8) & 0xff;
    cmd[4] = y & 0xff;
    cmd[5] = (radius >> 8) & 0xff;
    cmd[6] = radius & 0xff;
    cmd[7] = (color >> 8) & 0xff;
    cmd[8] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 9)) != 9)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}





int
PGD::Triangle(ushort x1, ushort y1, ushort x2, ushort y2,
                      ushort x3, ushort y3, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[16];

    cmd[0] = 'G';
    cmd[1] = (x1 >> 8) & 0xff;
    cmd[2] = x1 & 0xff;
    cmd[3] = (y1 >> 8) & 0xff;
    cmd[4] = y1 & 0xff;
    cmd[5] = (x2 >> 8) & 0xff;
    cmd[6] = x2 & 0xff;
    cmd[7] = (y2 >> 8) & 0xff;
    cmd[8] = y2 & 0xff;
    cmd[9] = (x3 >> 8) & 0xff;
    cmd[10] = x3 & 0xff;
    cmd[11] = (y3 >> 8) & 0xff;
    cmd[12] = y3 & 0xff;
    cmd[13] = (color >> 8) & 0xff;
    cmd[14] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 15)) != 15)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}





int
PGD::DrawIcon(ushort x, ushort y, ushort width, ushort height,
              uchar colormode, const uchar *data, int datalen)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if ((colormode != 0x08)&&(colormode != 0x10))
    {
        ERRMSG("invalid color mode (%ud); valid values are 0x08 and 0x10 only", colormode);
        return -1;
    }

    int dsize; // calculated payload size
    dsize = width*height;

    if (colormode == 0x10) dsize *= 2;

    if (dsize != datalen)
    {
        ERRMSG("invalid data length for color mode 0x%.2d (size = %d, expected %d)",
               colormode, datalen, dsize);
        return -1;
    }

    char *cmd = new char[dsize + 10];
    if (cmd == NULL)
    {
        ERRMSG("could not allocate memory");
        return -1;
    }
    cmd[0] = 'I';
    cmd[1] = (x >> 8) & 0xff;
    cmd[2] = x & 0xff;
    cmd[3] = (y >> 8) & 0xff;
    cmd[4] = y & 0xff;
    cmd[5] = (width >> 8) & 0xff;
    cmd[6] = width & 0xff;
    cmd[7] = (height >> 8) & 0xff;
    cmd[8] = height & 0xff;
    cmd[9] = colormode;
    memcpy(&cmd[10], data, datalen);

    dsize += 10;
    int res = port.Write(cmd, dsize);
    delete [] cmd;
    if (res != dsize) return res;

    return waitACKNACK(400);
}



int
PGD::SetBackground(ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[4] = "K  ";
    cmd[1] = (color >> 8) & 0xff;
    cmd[2] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 3)) != 3)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}



int
PGD::Line(ushort x1, ushort y1, ushort x2, ushort y2, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[12];
    cmd[0] = 'L';
    cmd[1] = (x1 >> 8) & 0xff;
    cmd[2] = x1 & 0xff;
    cmd[3] = (y1 >> 8) & 0xff;
    cmd[4] = y1 & 0xff;
    cmd[5] = (x2 >> 8) & 0xff;
    cmd[6] = x2 & 0xff;
    cmd[7] = (y2 >> 8) & 0xff;
    cmd[8] = y2 & 0xff;
    cmd[9] = (color >> 8) & 0xff;
    cmd[10] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 11)) != 11)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}





int
PGD::Polygon(uchar vertices, ushort *xp, ushort *yp, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if ((vertices < 3) || (vertices > 7))
    {
        ERRMSG("invalid number of vertices (%d); valid range is 3..7", vertices);
        return -1;
    }
    if ((!xp)||(!yp))
    {
        ERRMSG("invalid vertex list (NULL pointer)");
        return -1;
    }

    char cmd[32];
    cmd[0] = 'g';
    cmd[1] = vertices;

    int i;
    int idx = 1;
    ushort val;
    for (i = 0; i < vertices; ++i)
    {
        val = xp[i];
        cmd[++idx] = (val >> 8) & 0xff;
        cmd[++idx] = val & 0xff;
        val = yp[i];
        cmd[++idx] = (val >> 8) & 0xff;
        cmd[++idx] = val & 0xff;
    }
    cmd[++idx] = (color >> 8) & 0xff;
    cmd[++idx] = color & 0xff;
    ++idx;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, idx)) != idx)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}


int
PGD::Rectangle(ushort x1, ushort y1, ushort x2, ushort y2, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[12];

    cmd[0] = 'r';
    cmd[1] = (x1 >> 8) & 0xff;
    cmd[2] = x1 & 0xff;
    cmd[3] = (y1 >> 8) & 0xff;
    cmd[4] = y1 & 0xff;
    cmd[5] = (x2 >> 8) & 0xff;
    cmd[6] = x2 & 0xff;
    cmd[7] = (y2 >> 8) & 0xff;
    cmd[8] = y2 & 0xff;
    cmd[9] = (color >> 8) & 0xff;
    cmd[10] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 11)) != 11)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}





int
PGD::Ellipse(ushort x, ushort y, ushort rx, ushort ry, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[12];

    cmd[0] = 'e';
    cmd[1] = (x >> 8) & 0xff;
    cmd[2] = x & 0xff;
    cmd[3] = (y >> 8) & 0xff;
    cmd[4] = y & 0xff;
    cmd[5] = (rx >> 8) & 0xff;
    cmd[6] = rx & 0xff;
    cmd[7] = (ry >> 8) & 0xff;
    cmd[8] = ry & 0xff;
    cmd[9] = (color >> 8) & 0xff;
    cmd[10] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 11)) != 11)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



int
PGD::WritePixel(ushort x, ushort y, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[8];

    cmd[0] = 'P';
    cmd[1] = (x >> 8) & 0xff;
    cmd[2] = x & 0xff;
    cmd[3] = (y >> 8) & 0xff;
    cmd[4] = y & 0xff;
    cmd[5] = (color >> 8) & 0xff;
    cmd[6] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 7)) != 7)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



int
PGD::ReadPixel(ushort x, ushort y, ushort *color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[6];

    cmd[0] = 'R';
    cmd[1] = (x >> 8) & 0xff;
    cmd[2] = x & 0xff;
    cmd[3] = (y >> 8) & 0xff;
    cmd[4] = y & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 5)) != 5)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }


    res = port.Read(cmd, 2, 200);
    if (res < 0)
    {
        ERRMSG("no response");
        return -1;
    }
    if (res != 2)
    {
        ERRMSG("incomplete response packet (%d bytes, 5 expected)", res);
        return -1;
    }

    *color = (cmd[1] & 0xff) | ((cmd[0] & 0xff) << 8);
    return 0;
}



int
PGD::CopyPaste(ushort xsrc, ushort ysrc, ushort xdst, ushort ydst,
                       ushort width, ushort height)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[16];

    cmd[0] = 'c';
    cmd[1] = (xsrc >> 8) & 0xff;
    cmd[2] = xsrc & 0xff;
    cmd[3] = (ysrc >> 8) & 0xff;
    cmd[4] = ysrc & 0xff;
    cmd[5] = (xdst >> 8) & 0xff;
    cmd[6] = xdst & 0xff;
    cmd[7] = (ydst >> 8) & 0xff;
    cmd[8] = ydst & 0xff;
    cmd[9] = (width >> 8) & 0xff;
    cmd[10] = width & 0xff;
    cmd[11] = (height >> 8) & 0xff;
    cmd[12] = height & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 13)) != 13)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(2000);
}



int PGD::ReplaceColor(ushort x1, ushort y1, ushort x2, ushort y2,
                      ushort oldcolor, ushort newcolor)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[16];

    cmd[0] = 'k';
    cmd[1] = (x1 >> 8) & 0xff;
    cmd[2] = x1 & 0xff;
    cmd[3] = (y1 >> 8) & 0xff;
    cmd[4] = y1 & 0xff;
    cmd[5] = (x2 >> 8) & 0xff;
    cmd[6] = x2 & 0xff;
    cmd[7] = (y2 >> 8) & 0xff;
    cmd[8] = y2 & 0xff;
    cmd[9] = (oldcolor >> 8) & 0xff;
    cmd[10] = oldcolor & 0xff;
    cmd[11] = (newcolor >> 8) & 0xff;
    cmd[12] = newcolor & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 13)) != 13)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(5000);
}





int
PGD::PenSize(uchar size)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if ((size != 0) && (size != 1))
    {
        ERRMSG("invalid pen size (%d); valid values are 0,1", size);
        return -1;
    }

    port.Flush();
    int res;
    if (size)
        res = port.Write("p\x01", 2);
    else
        res = port.Write("p\x00", 2);

    if (res != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}



/*****************************************************
                    TEXT COMMANDS
*****************************************************/

int
PGD::SetFont(uchar size)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if ((size < 0) || (size > 3))
    {
        ERRMSG("invalid font size (%d); valid values are 0..3", size);
        return -1;
    }

    char cmd[2];
    cmd[0] = 'F';
    cmd[1] = size;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 2)) != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}



int
PGD::SetOpacity(uchar mode)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if ((mode != 0) && (mode != 1))
    {
        ERRMSG("invalid text opacity mode (%d); valid values are 0,1", mode);
        return -1;
    }

    port.Flush();
    int res;
    if (mode)
        res = port.Write("O\x01", 2);
    else
        res = port.Write("O\x00", 2);

    if (res != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}



int
PGD::ShowChar(uchar glyph, uchar col, uchar row, ushort color)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[6];

    cmd[0] = 'T';
    cmd[1] = glyph;
    cmd[2] = col;
    cmd[3] = row;
    cmd[4] = (color >> 8) & 0xff;
    cmd[5] = color & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 6)) != 6)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(100);
}



int
PGD::ScaleChar(uchar glyph, ushort x, ushort y, ushort color, uchar xmul, uchar ymul)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[10];

    cmd[0] = 't';
    cmd[1] = glyph;
    cmd[2] = (x >> 8) & 0xff;
    cmd[3] = x & 0xff;
    cmd[4] = (y >> 8) & 0xff;
    cmd[5] = y & 0xff;
    cmd[6] = (color >> 8) & 0xff;
    cmd[7] = color & 0xff;
    cmd[8] = xmul;
    cmd[9] = ymul;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 10)) != 10)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(5000);
}



int
PGD::ShowString(uchar col, uchar row, uchar font, ushort color, const char *data)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[270];
    int dlen;
    if (!data)
    {
        ERRMSG("invalid string pointer (NULL)");
        return -1;
    }

    if ((dlen = strlen(data)) == 0) return 0; // nothing to do
    if (dlen > 256) dlen = 256;

    cmd[0] = 's';
    cmd[1] = col;
    cmd[2] = row;
    cmd[3] = font;
    cmd[4] = (color >> 8) & 0xff;
    cmd[5] = color & 0xff;
    memcpy(&cmd[6], data, dlen);
    cmd[dlen + 6] = 0;
    dlen += 7;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, dlen)) != dlen)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(400);
}





int
PGD::ScaleString(ushort x, ushort y, uchar font, ushort color, uchar width,
                 uchar height, const char *data)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[270];
    int dlen;
    if (!data)
    {
        ERRMSG("invalid string pointer (NULL)");
        return -1;
    }

    if ((dlen = strlen(data)) == 0) return 0; // nothing to do
    if (dlen > 256) dlen = 256;

    cmd[0] = 'S';
    cmd[1] = (x >> 8) & 0xff;
    cmd[2] = x & 0xff;
    cmd[3] = (y >> 8) & 0xff;
    cmd[4] = y & 0xff;
    cmd[5] = font;
    cmd[6] = (color >> 8) & 0xff;
    cmd[7] = color & 0xff;
    cmd[8] = width;
    cmd[9] = height;
    memcpy(&cmd[10], data, dlen);
    cmd[dlen + 10] = 0;
    dlen += 11;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, dlen)) != dlen)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(5000);
}





int
PGD::Button(bool pressed, ushort x, ushort y, ushort bcolor, uchar font,
            ushort tcolor, uchar xmul, uchar ymul, const char *text)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[270];
    int dlen;
    if (!text)
    {
        ERRMSG("invalid string pointer (NULL)");
        return -1;
    }

    if ((dlen = strlen(text)) == 0) return 0; // nothing to do
    if (dlen > 256) dlen = 256;

    cmd[0] = 'b';
    if (pressed)
        cmd[1] = 1;
    else
        cmd[1] = 0;
    cmd[2] = (x >> 8) & 0xff;
    cmd[3] = x & 0xff;
    cmd[4] = (y >> 8) & 0xff;
    cmd[5] = y & 0xff;
    cmd[6] = (bcolor >> 8) & 0xff;
    cmd[7] = bcolor & 0xff;
    cmd[8] = font;
    cmd[9] = (tcolor >> 8) & 0xff;
    cmd[10] = tcolor & 0xff;
    cmd[11] = xmul;
    cmd[12] = ymul;
    memcpy(&cmd[13], text, dlen);
    cmd[dlen + 13] = 0;
    dlen += 14;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, dlen)) != dlen)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(2000);
}





/*****************************************************
                  TOUCHPAD COMMANDS
*****************************************************/

int PGD::GetTouch(uchar mode, ushort *points)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[5] = "o   ";

    cmd[1] = mode;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 2)) != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    if (mode <= 3)
    {
        curcmd = PG_TOUCH_DATA;
        curdata = points;
        brcv = 0;
        state = LCD_BUSY;
        return 2;
    }

    res = port.Read(cmd, 4, 100);

    if (res < 0)
    {
        ERRMSG("no response");
        return -1;
    }
    if (res != 4)
    {
        char msg[512];
        int i, j, k;
        j = 0;
        for (i = 0; i < res; ++i)
        {
            k = snprintf(&msg[j], 512 - j, "[0x%.2X] ", cmd[i]&0xff);
            j +=k;
        }
        ERRMSG("incomplete response packet (%d bytes, 4 expected): %s", res, msg);
        return -1;
    }

    points[0] = ((cmd[0] << 8) & 0xff00) | (cmd[1] & 0xff);
    points[1] = ((cmd[2] << 8) & 0xff00) | (cmd[3] & 0xff);

    return 0;
}


int
PGD::WaitTouch(ushort timeout)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[4] = "w  ";

    cmd[1] = (timeout >> 8) & 0xff;
    cmd[2] = timeout & 0xff;
    port.Flush();
    int res;
    if ((res = port.Write(cmd, 3)) != 3)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    switch (waitACKNACK(0))
    {
        case 0:
            return 0;
        case 1:
            return 1;
        case 2:
            curcmd = PG_TOUCH_WAIT;
            curdata = NULL;
            state = LCD_BUSY;
            return 2;
        default:
            break;
    }

    return -1;
}


int
PGD::SetRegion(ushort x1, ushort y1, ushort x2, ushort y2)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[10] = "u        ";

    cmd[1] = (x1 >> 8) & 0xff;
    cmd[2] = x1 & 0xff;
    cmd[3] = (y1 >> 8) & 0xff;
    cmd[4] = y1 & 0xff;
    cmd[5] = (x2 >> 8) & 0xff;
    cmd[6] = x2 & 0xff;
    cmd[7] = (y2 >> 8) & 0xff;
    cmd[8] = y2 & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 9)) != 9)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



// wait for an ACK; all other characters are rejected.
// returns 0 for success, -1 for comms fault, +2 for timeout
int
PGD::waitACK(int timeout)
{
    /* W32 */
    struct timeval tov;
    struct timeval now;

    gettimeofday(&now, NULL);
    tov = now;

    if (timeout < 2) timeout = 2;
    if (timeout >= 1000)
    {
        tov.tv_sec += (timeout / 1000);
    }
    tov.tv_usec += (timeout % 1000) * 1000;
    if (tov.tv_usec >= 1000000)
    {
        tov.tv_sec += 1;
        tov.tv_usec %= 1000000;
    }

    int i, nb;
    char msg[64];
    while ((now.tv_sec < tov.tv_sec) || ((now.tv_sec == tov.tv_sec)
            && (now.tv_usec < tov.tv_sec)))
    {
        nb = port.Read(msg, 64, 10);
        if (nb == -1)
        {
            ERRMSG("failed (see message below)\n%s", port.GetError());
            return -1;
        }
        for (i = 0; i < nb; ++i) if (msg[i] == '\x06') return 0;
        gettimeofday(&now, NULL);
    }
    ERRMSG("timeout");
    return 2;
}

// wait for a NACK; all other characters are rejected.
// returns 1 for NACK, -1 for comms fault, 0 for timeout
int
PGD::waitNACK(int timeout)
{
    /* W32 */
    struct timeval tov;
    struct timeval now;

    gettimeofday(&now, NULL);
    tov = now;

    if (timeout < 2) timeout = 2;
    if (timeout >= 1000)
    {
        tov.tv_sec += (timeout / 1000);
    }
    tov.tv_usec += (timeout % 1000) * 1000;
    if (tov.tv_usec >= 1000000)
    {
        tov.tv_sec += 1;
        tov.tv_usec %= 1000000;
    }

    int i, nb;
    char msg[64];
    while ((now.tv_sec < tov.tv_sec) || ((now.tv_sec == tov.tv_sec)
            && (now.tv_usec < tov.tv_sec)))
    {
        nb = port.Read(msg, 64, 10);
        if (nb == -1)
        {
            ERRMSG("failed (see message below)\n%s", port.GetError());
            return -1;
        }
        for (i = 0; i < nb; ++i) if (msg[i] == '\x15') return 1;
        gettimeofday(&now, NULL);
    }
    return 0;
}

// wait for either an ACK or a NACK while rejecting other characters
// return -1 for comms fault, 0 for ACK, +1 for NACK, +2 for timeout
int
PGD::waitACKNACK(int timeout)
{
    /* W32 */
    struct timeval tov;
    struct timeval now;

    gettimeofday(&now, NULL);
    tov = now;

    if (timeout >= 1000)
    {
        tov.tv_sec += (timeout / 1000);
    }
    tov.tv_usec += (timeout % 1000) * 1000;
    if (tov.tv_usec >= 1000000)
    {
        tov.tv_sec += 1;
        tov.tv_usec %= 1000000;
    }

    int i, nb;
    char msg[4];
    while ((now.tv_sec < tov.tv_sec) || ((now.tv_sec == tov.tv_sec)
            && (now.tv_usec <= tov.tv_sec)))
    {
        nb = port.Read(msg, 4, 10);
        if (nb == -1)
        {
            ERRMSG("failed (see message below)\n%s", port.GetError());
            return -1;
        }
        for (i = 0; i < nb; ++i)
        {
            if (msg[i] == '\x06') return 0;
            if (msg[i] == '\x15') return 1;
        }
        gettimeofday(&now, NULL);
    }
    ERRMSG("timeout");
    return 2;
}



int
PGD::Process(void)
{
    if (halt) return -1;
    if (state != LCD_BUSY)
    {
        usleep(100000);
        return 0;
    }
    PGDCMD tmpcmd = curcmd;

    switch (curcmd)
    {
        case PG_NONE:
            ERROUT("unexpected case: cmd = PG_NONE while state = LCD_BUSY\n");
            ERRMSG("unexpected case: cmd = PG_NONE while state = LCD_BUSY");
            state = LCD_IDLE;
            if (callback) callback(this, PG_NONE, false, usrobj);
            return 0;
        case PG_SLEEP:
        case PG_TOUCH_WAIT:
            switch (waitACKNACK(200))
            {
                case 0:
                    curcmd = PG_NONE;
                    state = LCD_IDLE;
                    if (callback) callback(this, tmpcmd, true, usrobj);
                    return 0;
                case 1:
                    snprintf(errmsg, PGDERRLEN, "NACK");
                    curcmd = PG_NONE;
                    state = LCD_IDLE;
                    if (callback) callback(this, tmpcmd, false, usrobj);
                    return 0;
                case -1:
                    curcmd = PG_NONE;
                    state = LCD_IDLE;
                    if (callback) callback(this, tmpcmd, false, usrobj);
                    return 0;
                case 2:
                    return 0;
                default:
                    ERRMSG("unexpected return from waitACKNACK (only 0,1,2,-1 accepted)");
                    curcmd = PG_NONE;
                    state = LCD_IDLE;
                    if (callback) callback(this, tmpcmd, false, usrobj);
                    return 0;
            }
            break;
        case PG_TOUCH_DATA:
            do {
                int nb = port.Read(&datain[brcv], 4 - brcv, 100);
                if (nb < 0)
                {
                    ERRMSG("PG_TOUCH_DATA: communications fault, see message below\n%s",
                           port.GetError());
                    curcmd = PG_NONE;
                    state = LCD_IDLE;
                    if (callback) callback(this, tmpcmd, false, usrobj);
                    return 0;
                }
                if (nb)
                {
                    brcv += nb;
                    if (brcv == 4)
                    {
                        if (!curdata)
                        {
                            ERRMSG("data pointer is NULL");
                            curcmd = PG_NONE;
                            state = LCD_IDLE;
                            if (callback) callback(this, tmpcmd, false, usrobj);
                            return 0;
                        }
                        unsigned short *sp = (unsigned short *)curdata;
                        sp[0] = ((datain[0] << 8) & 0xff00) | (datain[1] & 0xff);
                        sp[1] = ((datain[2] << 8) & 0xff00) | (datain[3] & 0xff);
                        curcmd = PG_NONE;
                        state = LCD_IDLE;
                        if (callback) callback(this, tmpcmd, true, usrobj);
                        return 0;
                    }
                }
            } while (0);
            return 0;
        default:
            ERROUT("unsupported command: 0x%.2X\n", curcmd&0xff);
            ERRMSG("unsupported command: 0x%.2X", curcmd&0xff);
            curcmd = PG_NONE;
            state = LCD_IDLE;
            if (callback) callback(this, PG_NONE, false, usrobj);
            return 0;
    }
    ERROUT("BUG: unexpected code branch\n");
    ERRMSG("BUG: unexpected code branch");
    curcmd = PG_NONE;
    state = LCD_IDLE;
    if (callback) callback(this, PG_NONE, false, usrobj);
    return 0;
}


/* uSDHC Card Operations */
/* Initialize Memory Card, p. 51 */
int PGD::SDInit(void)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    port.Flush();
    int res;
    if ((res = port.Write("@i", 2)) != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}

/* Set Address Pointer of Card */
int PGD::SDSetAddrRaw(unsigned int addr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[10] = "@A       ";

    cmd[2] = (addr >> 24) & 0xff;
    cmd[3] = (addr >> 16) & 0xff;
    cmd[4] = (addr >> 8) & 0xff;
    cmd[5] = addr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 6)) != 6)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}

/* Read Byte from Card */
int PGD::SDReadByteRaw(char *data)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    port.Flush();
    int res;
    if ((res = port.Write("@r", 2)) != 2)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return port.Read(data, 1, 200);
}

/* Write Byte to Card */
int PGD::SDWriteByteRaw(char data)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[4] = "@w ";
    cmd[2] = data;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 3)) != 3)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



/* Read Sector Block from Card */
int PGD::SDReadSectRaw(unsigned int sectaddr, char *data, int datalen)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (datalen < 512)
    {
        ERRMSG("datalen must be at least 512 (== %d)", datalen);
        return -1;
    }

    if (sectaddr > 0x00ffffff)
    {
        ERRMSG("invalid sector address (%.8X), must be <= 0x00ffffff", sectaddr);
        return -1;
    }

    char cmd[6] = "@R   ";
    cmd[2] = (sectaddr >> 16) & 0xff;
    cmd[3] = (sectaddr >> 8) & 0xff;
    cmd[4] = sectaddr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 5)) != 5)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return port.Read(data, 512, 500);
}



/* Write Sector Block to Card */
int PGD::SDWriteSectRaw(unsigned int sectaddr, const char *data, int datalen)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (sectaddr > 0x00ffffff)
    {
        ERRMSG("invalid sector address (%.8X), must be <= 0x00ffffff", sectaddr);
        return -1;
    }

    char cmd[520];
    cmd[0] = '@';
    cmd[1] = 'W';
    cmd[2] = (sectaddr >> 16) & 0xff;
    cmd[3] = (sectaddr >> 8) & 0xff;
    cmd[4] = sectaddr & 0xff;
    memcpy(&cmd[5], data, 512);

    if (datalen != 512)
    {
        ERRMSG("datalen must be 512 (== %d)", datalen);
        return -1;
    }

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 517)) != 517)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}


/* Screen Copy and Save to Card */
int PGD::SDScreenCopyRaw(ushort x, ushort y, ushort width, ushort height,
                         unsigned int sectaddr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (sectaddr > 0x00ffffff)
    {
        ERRMSG("invalid sector address (%.8X), must be <= 0x00ffffff", sectaddr);
        return -1;
    }

    char cmd[14] = "@C           ";
    cmd[2] = (x >> 8) & 0xff;
    cmd[3] = x & 0xff;
    cmd[4] = (y >> 8) & 0xff;
    cmd[5] = y & 0xff;
    cmd[6] = (width >> 8) & 0xff;
    cmd[7] = width & 0xff;
    cmd[8] = (height >> 8) & 0xff;
    cmd[9] = height & 0xff;
    cmd[10] = (sectaddr >> 16) & 0xff;
    cmd[11] = (sectaddr >> 8) & 0xff;
    cmd[12] = sectaddr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 13)) != 13)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



/* Display Image / Icon from Card */
int PGD::SDShowImageRaw(ushort x, ushort y, ushort width, ushort height,
                        uchar colormode, unsigned int sectaddr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (sectaddr > 0x00ffffff)
    {
        ERRMSG("invalid sector address (%.8X), must be <= 0x00ffffff", sectaddr);
        return -1;
    }

    if ((colormode != 0x08) && (colormode != 0x10))
    {
        ERRMSG("invalid color mode (%.2X), must be 0x08 (8-bit) or 0x10 (16-bit)", colormode & 0xff);
        return -1;
    }

    char cmd[14] = "@I           ";
    cmd[2] = (x >> 8) & 0xff;
    cmd[3] = x & 0xff;
    cmd[4] = (y >> 8) & 0xff;
    cmd[5] = y & 0xff;
    cmd[6] = (width >> 8) & 0xff;
    cmd[7] = width & 0xff;
    cmd[8] = (height >> 8) & 0xff;
    cmd[9] = height & 0xff;
    cmd[10] = colormode;
    cmd[11] = (sectaddr >> 16) & 0xff;
    cmd[12] = (sectaddr >> 8) & 0xff;
    cmd[13] = sectaddr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 14)) != 14)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



/* Display Object from Card */
int PGD::SDShowObjectRaw(unsigned int byteaddr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[6] = "@O   ";
    cmd[2] = (byteaddr >> 24) & 0xff;
    cmd[3] = (byteaddr >> 16) & 0xff;
    cmd[4] = (byteaddr >> 8) & 0xff;
    cmd[5] = byteaddr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 6)) != 6)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



/* Display Video / Animation from Card, new format image data */
int PGD::SDShowVideoRaw(ushort x, ushort y, uchar delay, unsigned int sectaddr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    // XXX - can we test if we have the video format set correctly (new format)

    if (sectaddr > 0x00ffffff)
    {
        ERRMSG("invalid sector address (%.8X), must be <= 0x00ffffff", sectaddr);
        return -1;
    }

    char cmd[10] = "@V       ";
    cmd[2] = (x >> 8) & 0xff;
    cmd[3] = x & 0xff;
    cmd[4] = (y >> 8) & 0xff;
    cmd[5] = y & 0xff;
    cmd[6] = delay;
    cmd[7] = (sectaddr >> 16) & 0xff;
    cmd[8] = (sectaddr >> 8) & 0xff;
    cmd[9] = sectaddr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 10)) != 10)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}

/* Display Video / Animation from Card, old format image data */
int PGD::SDShowVideoRaw(ushort x, ushort y, ushort width, ushort height,
                        uchar colormode, uchar delay, ushort frames,
                        unsigned int sectaddr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    // XXX - can we test if we have the video format set correctly (old format)

    if (sectaddr > 0x00ffffff)
    {
        ERRMSG("invalid sector address (%.8X), must be <= 0x00ffffff", sectaddr);
        return -1;
    }

    if ((colormode != 0x08) && (colormode != 0x10))
    {
        ERRMSG("invalid color mode (%.2X), must be 0x08 (8-bit) or 0x10 (16-bit)", colormode & 0xff);
        return -1;
    }

    char cmd[17] = "@V              ";
    cmd[2] = (x >> 8) & 0xff;
    cmd[3] = x & 0xff;
    cmd[4] = (y >> 8) & 0xff;
    cmd[5] = y & 0xff;
    cmd[6] = (width >> 8) & 0xff;
    cmd[7] = width & 0xff;
    cmd[8] = (height >> 8) & 0xff;
    cmd[9] = height & 0xff;
    cmd[10] = colormode;
    cmd[11] = delay;
    cmd[12] = (frames >> 8) & 0xff;
    cmd[13] = frames & 0xff;
    cmd[14] = (sectaddr >> 16) & 0xff;
    cmd[15] = (sectaddr >> 8) & 0xff;
    cmd[16] = sectaddr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 17)) != 17)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



/*  Run 4DSL Script from Card */
int PGD::SDRunScriptRaw(unsigned int byteaddr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    char cmd[6] = "@P   ";
    cmd[2] = (byteaddr >> 24) & 0xff;
    cmd[3] = (byteaddr >> 16) & 0xff;
    cmd[4] = (byteaddr >> 8) & 0xff;
    cmd[5] = byteaddr & 0xff;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, 6)) != 6)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitNACK(200);
}


/* SD FAT16 COMMANDS, starts at p.63 */
/* Read File From Card */
int PGD::SDReadFileFAT(void **data, unsigned int *size, const char *filename)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (!data)
    {
        ERRMSG("invalid data handle (NULL)");
        return -1;
    }

    if (!size)
    {
        ERRMSG("invalid size pointer (NULL)");
        return -1;
    }

    if (!filename)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }

    int len = strlen(filename);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    char cmd[20];
    cmd[0] = '@';
    cmd[1] = 'a';
    cmd[2] = 50;    // handshaking
    /* W32 */
    snprintf(&cmd[3], 13, "%s", filename);
    len += 4;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, len)) != len)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    int nb;
    if ((nb = port.Read(cmd, 4, 500)) == -1)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        return -1;
    }
    if (!nb)
    {
        port.Write("\x15", 1);  // attempt to cancel the transaction
        ERRMSG("timeout: no response");
        return -2;
    }
    if ((nb == 1) && (cmd[0] == '\x15')) return 1;
    if (nb != 4)
    {
        port.Write("\x15", 1);
        ERRMSG("unexpected response size (%d); expected 4", nb);
        return -2;
    }

    unsigned int fs = ((cmd[0] & 0xff) << 24) | ((cmd[1] & 0xff) << 16)
            | ((cmd[2] & 0xff) << 8) | (cmd[3] & 0xff);
    if (fs == 0)
    {
        // file size is zero; there is nothing to read
        port.Write("\x15", 1);
        *size = 0;
        return 0;
    }

    char *dp = new char[fs];
    if (!dp)
    {
        port.Write("\x15", 1);
        ERRMSG("could not allocate data (%u bytes)", fs);
        return -1;
    }
    *data = dp;
    *size = fs;

    // calculate the number of blocks and the size of the last block
    unsigned int nblk, nres;
    unsigned int i, idx, bs;
    nblk = fs / 50;
    nres = fs % 50;
    if (nres) ++nblk;

    // read in each block
    for (i = 0; i < nblk; ++i)
    {
        port.Write("\x06", 1);
        idx = i * 50;
        bs = 50;
        if ((i == nblk -1) && (nres)) bs = nres;
        while (bs)
        {
            nb = port.Read(&dp[idx], bs, 500);
            if (nb == -1)
            {
                *data = NULL;
                *size = 0;
                delete [] dp;
                ERRMSG("failed to read %d bytes of data; see message below\n%s",
                       fs, port.GetError());
                return -2;
            }
            if (nb == 0)
            {
                *data = NULL;
                *size = 0;
                delete [] dp;
                ERRMSG("failed to read %d bytes of data (timeout)", fs);
                return -2;
            }
            idx += nb;
            bs -= nb;
        }
    }
    // check the last byte; it should be an ACK
    return waitACK(100);
}


/* Write File To Card */
int PGD::SDWriteFileFAT(const void *data, unsigned int size, const char *filename, bool append)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (!data)
    {
        ERRMSG("invalid data pointer (NULL)");
        return -1;
    }

    if (!filename)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }

    unsigned int len = strlen(filename);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    char cmd[24];
    cmd[0] = '@';
    cmd[1] = 't';
    /* W32 */
    snprintf(&cmd[3], 13, "%s", filename);


    cmd[len + 4] = (size >> 24) & 0xff;
    cmd[len + 5] = (size >> 16) & 0xff;
    cmd[len + 6] = (size >> 8) & 0xff;
    cmd[len + 7] = size & 0xff;

    unsigned int nblk, nresid;
    port.Flush();
    if (size <= 100)
    {
        cmd[2] = 0;     // no handshaking
        nblk = 1;
        nresid = size;
    }
    else
    {
        cmd[2] = 50;    // handshaking, max. 50 bytes
        nblk = size / 50;
        nresid = size % 50;
        if (nresid) ++nblk;
    }
    if (append) cmd[2] |= 0x80;

    unsigned int i, idx, bs;
    char *dp = (char *)data;
    idx = 0;
    for (i = 0; i < nblk; ++i)
    {
        bs = 50;
        if ((i == nblk -1) && (nresid)) bs = nresid;
        switch (waitACKNACK(1000))
        {
            case 0:
                break;
            case 1:
                if (i == 0) return 1;   // NACK on the first packet
                ERRMSG("NACK after packet %d", i + 1);
                return -2;
            default:
                ERRMSG("write problems after packet %d", i + 1);
                return -1;
        }
        idx = i * 50;
        if (port.Write(&dp[idx], bs) != (int) bs)
        {
            ERRMSG("failed; see message below\n%s", port.GetError());
            return -2;
        }
    }
    return waitACKNACK(1000);
}



/* Erase File From Card */
int PGD::SDEraseFileFAT(const char *filename)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (!filename)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }

    int len = strlen(filename);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    char cmd[20];
    cmd[0] = '@';
    cmd[1] = 'e';
    /* W32 */
    snprintf(&cmd[2], 13, "%s", filename);
    len += 3;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, len)) != len)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}

/* List Directory From Card */
int PGD::SDListDirFAT(const char *pattern, std::list<std::string> *dir)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (!pattern)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }
    int len = strlen(pattern);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    char cmd[20];
    cmd[0] = '@';
    cmd[1] = 'd';
    /* W32 */
    snprintf(&cmd[2], 13, "%s", pattern);
    len += 3;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, len)) != len)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    int i;
    int nb;
    char buf[512];
    std::string direntry;
    dir->clear();
    direntry.clear();
    while ((nb = port.Read(buf, 512, 500)) > 0)
    {
        for (i = 0; i < nb; ++i)
        {
            if ((buf[i] == '\x0a') || (buf[i] == '\x06') || (buf[i] != '\x15'))
            {
                if (direntry.size()) dir->push_back(direntry);
                direntry.clear();
                if (buf[i] == '\x06') return dir->size();
                if (buf[i] == '\x15')
                {
                    ERRMSG("failed after acquiring %d entries (NACK)", dir->size());
                    return -1;
                }
                continue;
            }
            direntry += buf[i];
        }
    }
    if (nb == -1)
    {
        ERRMSG("failed after acquiring %d entries; see message below\n%s",
               dir->size(), port.GetError());
        return -1;
    }

    ERROUT("TIMEOUT; no ACK or NACK detected; %d entries", dir->size());
    ERRMSG("TIMEOUT; no ACK or NACK detected; %d entries", dir->size());
    return -1;
}

/* Screen Copy and Save to Card */
int PGD::SDScreenCopyFAT(ushort x, ushort y, ushort width, ushort height,
                         const char *filename)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (!filename)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }
    int len = strlen(filename);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    char cmd[24];
    cmd[0] = '@';
    cmd[1] = 'c';
    cmd[2] = (x >> 8) & 0xff;
    cmd[3] = x & 0xff;
    cmd[4] = (y >> 8) & 0xff;
    cmd[5] = y & 0xff;
    cmd[6] = (width >> 8) & 0xff;
    cmd[7] = width & 0xff;
    cmd[8] = (height >> 8) & 0xff;
    cmd[9] = height & 0xff;
    /* W32 */
    snprintf(&cmd[10], 13, "%s", filename);
    len += 11;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, len)) != len)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



/* Display Image / Icon from Card */
int PGD::SDShowImageFAT(const char *filename, ushort x, ushort y, unsigned int imgaddr)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (imgaddr > 0x00ffffff)
    {
        ERRMSG("invalid image sector address (%.8X), must be <= 0x00ffffff", imgaddr);
        return -1;
    }

    if (!filename)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }
    int len = strlen(filename);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    char cmd[24];
    cmd[0] = '@';
    cmd[1] = 'm';
    /* W32 */
    snprintf(&cmd[2], 13, "%s", filename);
    cmd[3 + len] = (x >> 8) & 0xff;
    cmd[4 + len] = x & 0xff;
    cmd[5 + len] = (y >> 8) & 0xff;
    cmd[6 + len] = y & 0xff;
    cmd[7 + len] = (imgaddr >> 16) & 0xff;
    cmd[8 + len] = (imgaddr >> 8) & 0xff;
    cmd[9 + len] = imgaddr & 0xff;
    len += 10;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, len)) != len)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}



/* Play Audio WAV file from Card */
int PGD::SDPlayAudioFAT(const char *filename, uchar option)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (!filename)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }
    int len = strlen(filename);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    if (option > 5)
    {
        ERRMSG("invalid option (%u); valid range is 0..5", option);
        return -1;
    }

    char cmd[20];
    cmd[0] = '@';
    cmd[1] = 'l';
    cmd[2] = option;
    /* W32 */
    snprintf(&cmd[3], 13, "%s", filename);
    len += 4;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, len)) != len)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}

/* Run 4DSL Script from Card */
int PGD::SDRunScriptFAT(const char *filename)
{
    CHECK_INACTIVE;
    CHECK_BUSY;

    if (!filename)
    {
        ERRMSG("invalid filename (NULL pointer)");
        return -1;
    }
    int len = strlen(filename);
    if ((len < 1)|| (len > 12))
    {
        ERRMSG("invalid filename length; must be 1..12 characters");
        return -1;
    }

    char cmd[20];
    cmd[0] = '@';
    cmd[1] = 'p';
    /* W32 */
    snprintf(&cmd[2], 13, "%s", filename);
    len += 3;

    port.Flush();
    int res;
    if ((res = port.Write(cmd, len)) != len)
    {
        ERRMSG("failed; see message below\n%s", port.GetError());
        if (res > 0) return -2;
        return -1;
    }

    return waitACKNACK(200);
}
