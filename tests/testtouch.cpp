/**
    file: testtouch.cpp

    This program tests the touch functions of the uOLED graphics display.

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

#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#include "oled.h"


#define WHITE (0xffff)
#define BLACK (0x0000)
#define RED (0xf800)
#define GREEN (0x07e0)
#define BLUE (0x001f)
#define PURPLE (0xf81f)
#define ORANGE (0xf8f0)
#define YELLOW (0xffe0)

extern char *optarg;
extern int optopt;


void printUsage(void)
{
    fprintf(stderr, "Usage: testtouch {-p serial_device} {-h}\n");
    fprintf(stderr, "\t-p: serial_device (default /dev/ttyUSB0)\n");
    fprintf(stderr, "\t-h: display usage and exit\n");
    return;
}


using namespace disp;

// globals for communications between callback and main routine
struct GLOBS {
    volatile bool wait;     // signals between callback and main thread
    volatile bool result;   // success flag for the callback
};

void usrcb(class PGD* pgd, PGDCMD cmd, bool result, void *obj)
{
    if (!pgd)
    {
        fprintf(stderr, "%s:%d: %s(): Invalid PGD pointer in callback\n",
                __FILE__, __LINE__, __FUNCTION__);
        return;
    }
    if (!obj)
    {
        fprintf(stderr, "%s:%d: %s(): Invalid user object pointer in callback\n",
                __FILE__, __LINE__, __FUNCTION__);
        return;
    }
    GLOBS *glob = (GLOBS *)obj;
    glob->wait = false;
    glob->result = result;
    return;
}


int main(int argc, char **argv)
{
    const char *port = "/dev/ttyUSB0";

    int inchar;
    while ((inchar = getopt(argc, argv, ":p:h")) > 0)
    {
        if (inchar == 'h')
        {
            printUsage();
            return 0;
        }
        if (inchar == 'p')
        {
            port = optarg;
            continue;
        }
        if (inchar == '?')
        {
            fprintf(stderr, "unknown option: '%c'\n", optopt);
            printUsage();
            return -1;
        }
        if (inchar == ':')
        {
            fprintf(stderr, "missing value for option: '%c'\n", optopt);
            printUsage();
            return -1;
        }
    }

    PGD oled;
    PGDVER ver;
    GLOBS globs;
    globs.wait = false;
    oled.SetCallback(usrcb, &globs);

    printf("\n\n* Attempting to connect to display: ");
    if (oled.Connect(port))
    {
        printf("FAILED\n");
        printf("%s\n", oled.GetError());
        fflush(stdout);
        return -1;
    }
    printf("OK\n");

    printf("* Baud code: 0x%.2X\n", oled.GetBaud());

    printf("* Retrieving version information (no display): ");
    if (oled.Version(&ver, 0))
    {
        printf("FAILED\n");
        printf("%s\n", oled.GetError());
        fflush(stdout);
        oled.Close();
        return -1;
    }
    printf("OK\n");
    printf("\tDisplay Type: ");
    switch (ver.display_type)
    {
        case DEV_OLED:
            printf("OLED\n");
            break;
        case DEV_LCD:
            printf("LCD\n");
            break;
        case DEV_VGA:
            printf("VGA\n");
            break;
        default:
            printf("UNKNOWN (0x%.2X)", ver.display_type);
            break;
    }
    printf("\tHardware Revision: 0x%.2X\n", ver.hardware_rev);
    printf("\tFirmware Revision: 0x%.2X\n", ver.firmware_rev);
    printf("\tHoriz. Pixels: %d\n", ver.hres);
    printf("\tVert. Pixels : %d\n", ver.vres);

    // activate the touch pad
    if (oled.Ctl(DM_TOUCHPAD, 0))
    {
        printf("* could not activate touchpad; bailing out\n");
        oled.Close();
        return -1;
    }

    unsigned short coord[2];
    printf("* touch test, default orientation: ");
    fflush(stdout);
    oled.Clear();
    oled.Ctl(4, 3);
    oled.ShowString(0, 1, 0x13, YELLOW, "Default orientation");
    oled.ShowString(0, 3, 0x13, YELLOW, "Touch screen to continue");
    globs.wait = true;
    globs.result = false;
    switch(oled.GetTouch(TM_PRESS, coord))
    {
        case 0:
            printf("ACK\n");
            globs.wait = false;
            break;
        case 1:
            printf("NACK\n");
            globs.wait = false;
            break;
        case 2:
            while (globs.wait) usleep(100000);
            break;
        default:
            break;
    }
    if (globs.result)
    {
        printf("OK: [%d, %d]\n", coord[0], coord[1]);
    }
    else
    {
        printf("FAIL/NACK\n");
    }
    usleep(2000000);

    printf("* touch test, 180 deg rotation: ");
    fflush(stdout);
    oled.Clear();
    oled.Ctl(4, 4);
    oled.ShowString(0, 1, 0x13, YELLOW, "180 deg orientation");
    oled.ShowString(0, 3, 0x13, YELLOW, "Touch screen to continue");
    globs.wait = true;
    globs.result = false;
    switch(oled.GetTouch(TM_PRESS, coord))
    {
        case 0:
            printf("ACK\n");
            globs.wait = false;
            break;
        case 1:
            printf("NACK\n");
            globs.wait = false;
            break;
        case 2:
            while (globs.wait) usleep(100000);
            break;
        default:
            break;
    }
    if (globs.result)
    {
        printf("OK: [%d, %d]\n", coord[0], coord[1]);
    }
    else
    {
        printf("FAIL/NACK\n");
    }
    usleep(2000000);

    printf("* touch test, 90 deg rotation: ");
    fflush(stdout);
    oled.Clear();
    oled.Ctl(4, 1);
    oled.ShowString(0, 1, 0x13, YELLOW, "90 deg orientation");
    oled.ShowString(0, 3, 0x13, YELLOW, "Touch screen to continue");
    globs.wait = true;
    globs.result = false;
    switch(oled.GetTouch(TM_PRESS, coord))
    {
        case 0:
            printf("ACK\n");
            globs.wait = false;
            break;
        case 1:
            printf("NACK\n");
            globs.wait = false;
            break;
        case 2:
            while (globs.wait) usleep(100000);
            break;
        default:
            break;
    }
    if (globs.result)
    {
        printf("OK: [%d, %d]\n", coord[0], coord[1]);
    }
    else
    {
        printf("FAIL/NACK\n");
    }
    usleep(2000000);

    printf("* touch test, 270 deg rotation: ");
    fflush(stdout);
    oled.Clear();
    oled.Ctl(4, 2);
    oled.ShowString(0, 1, 0x13, YELLOW, "270 deg orientation");
    oled.ShowString(0, 3, 0x13, YELLOW, "Touch screen to continue");
    globs.wait = true;
    globs.result = false;
    switch(oled.GetTouch(TM_PRESS, coord))
    {
        case 0:
            printf("ACK\n");
            globs.wait = false;
            break;
        case 1:
            printf("NACK\n");
            globs.wait = false;
            break;
        case 2:
            while (globs.wait) usleep(100000);
            break;
        default:
            break;
    }
    if (globs.result)
    {
        printf("OK: [%d, %d]\n", coord[0], coord[1]);
    }
    else
    {
        printf("FAIL/NACK\n");
    }
    usleep(2000000);

    oled.Ctl(4, 3);
    oled.Clear();
    oled.Close();
    return 0;
}
