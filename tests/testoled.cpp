/**
    file: testoled.cpp

    This program tests the functions of the uOLED graphics display.

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

const float pi = 3.1415927;

void printUsage(void)
{
    fprintf(stderr, "Usage: testoled {-p serial_device} {-b} {-h}\n");
    fprintf(stderr, "\t-p: serial_device (default /dev/ttyUSB0)\n");
    fprintf(stderr, "\t-b: include `replace background' test\n");
    fprintf(stderr, "\t-h: display usage and exit\n");
    return;
}

void calctime(struct timeval ts, struct timeval te)
{
    int sec, msec;
    sec = te.tv_sec - ts.tv_sec;
    msec = te.tv_usec - ts.tv_usec;
    if (msec < 0)
    {
        --sec;
        msec += 1000000;
    }
    msec /= 1000;
    printf("\tTime for completion: %d s, %d msec\n", sec, msec);
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

int drawStar(unsigned short midx, unsigned short midy, unsigned short rad, PGD *pgd);
int testRWPIN(PGD *pgd, int i);
int testRWBUS(PGD *pgd);
int drawPoly(unsigned short midx, unsigned short midy, PGD *pgd);
int drawEllipse(unsigned short midx, unsigned short midy, PGD *pgd);
int drawTriangle(unsigned short midx, unsigned short midy, PGD *pgd);
int drawCircle(unsigned short width, unsigned short height, PGD *pgd);
int drawRect(unsigned short width, unsigned short height, PGD *pgd);
int testPixRW(unsigned short width, unsigned short height, PGD *pgd);
int testBMsmall(unsigned short width, unsigned short height, PGD *pgd);
int testBMmed(unsigned short width, unsigned short height, PGD *pgd);
int testBMbig(unsigned short width, unsigned short height, PGD *pgd);
int testTouch(PGD *pgd, GLOBS *globs);

int main(int argc, char **argv)
{
    const char *port = "/dev/ttyUSB0";
    bool test_bkgd = false;

    int inchar;
    while ((inchar = getopt(argc, argv, ":p:hb")) > 0)
    {
        if (inchar == 'h')
        {
            printUsage();
            return 0;
        }
        if (inchar == 'b')
        {
            test_bkgd = true;
            continue;
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

    // horizontal center of display
    unsigned short midx;
    // parameters for volume test
    const int NVOL = 10;
    uchar VOLS[NVOL] = {0xff, 0x3f, 0xfd, 0x03, 0xfe, 0x01, 0x02, 0x7f, 0x08, 0x00};
    char TVOL[NVOL][32];
    // parameters for Copy-Paste
    const int NCPT = 4;
    unsigned short XCP[NCPT];
    unsigned short YCP[NCPT];
    unsigned short cpx;
    unsigned short cpy;
    // parameters for orientation test
    const char OR[4][12] = {"LANDSCAPE\0\0", "LANDSCAPE_R", "PORTRAIT\0\0\0", "PORTRAIT_R\0"};
    // parameters for string test
    unsigned short xposT;
    unsigned short yposT;
    // parameters for ReplaceBackground test
    const int NBK = 5;  // NOTE: must be > 4 with first 4 colors non-zero
    // 16-bit color: R4R3R2R1R0G5G4G3  G2G1G0B4B3B2B1B0
    unsigned short BKGD[NBK] = {0x7BEF, RED, GREEN, BLUE, BLACK};



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

    // NOTE: The version response is significantly delayed
    // if we request the information to be displayed on the screen.
    int i;
    struct timeval ts, te;
    for (i = 0; i < 2; ++i)
    {
        if (i == 0)
            printf("* Retrieving version information (no display): ");
        else
            printf("* Retrieving version information (with display): ");

        gettimeofday(&ts, NULL);
        if (oled.Version(&ver, i))
        {
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            oled.Close();
            return -1;
        }
        gettimeofday(&te, NULL);
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
        calctime(ts, te);
    }
goto LATEST;
    printf("[waiting ~5s]\n");
    usleep(5000000);

    printf("* Clearing Screen: ");
    if (oled.Clear())
    {
        printf("FAILED\n");
        printf("%s\n", oled.GetError());
        fflush(stdout);
        return -1;
    }
    printf("OK\n");

    if (test_bkgd)
    {
        for (i = 0; i < NBK; ++i)
        {
            printf("* Replacing background with 0x%.4X: ", BKGD[i]&0xffff);
            gettimeofday(&ts, NULL);
            if (oled.ReplaceBackground(BKGD[i]))
            {
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                return -1;
            }
            gettimeofday(&te, NULL);
            printf("OK\n");
            calctime(ts, te);
            usleep(500000);
        }
    }

    printf("* String 'TEST': ");
    xposT = 8;
    yposT = 10;
    switch (oled.ShowString(xposT, yposT, 3, WHITE, "TEST"))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            return -1;
    }
    usleep(3000000);

    for (i = 0; i < 2; ++i)
    {
        printf("* CTL Backlight %s: ", i ? "ON" : "OFF");
        switch (oled.Ctl(0, i))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                return -1;
        }
        usleep(500000);
    }

    for (i = 0; i < 2; ++i)
    {
        printf("* CTL Display %s: ", i ? "ON" : "OFF");
        switch (oled.Ctl(1, i))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                return -1;
        }
        usleep(500000);
    }

    for (i = 0; i < 256; i += 127)
    {
        printf("* CTL Contrast 0x%.2X: ", i);
        switch (oled.Ctl(2, i))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                return -1;
        }
        usleep(1000000);
    }

    for (i = 4; i >= 0; --i)
    {
        printf("* CTL Orientation %s: ", i ? OR[i-1] : OR[2]);
        switch (oled.Ctl(4, i ? i : 3))
        {
            case 0:
                printf("OK\n");
                if (i) oled.ShowString(0, 2*i, 1, BKGD[i-1], OR[i-1]);
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                return -1;
        }
        usleep(1000000);
    }
    printf("* Untested CTL modes at this point include:\n\t[Touch Control]\n"
            "\t[Image Format]\n\t[Protect FAT]\n");
    usleep(3000000);

    // TEST: int  CopyPaste(ushort xsrc, ushort ysrc, ushort xdst, ushort ydst,
    //                ushort width, ushort height);
    // Intent of test: Cut and paste the text 'TEST'
    XCP[0] = XCP[2] = xposT*12 - 48;
    XCP[1] = XCP[3] = xposT*12 + 48;
    YCP[0] = YCP[1] = (yposT -1.5)*16;
    YCP[2] = YCP[3] = (yposT +1.5)*16;
    cpx = xposT*12;
    cpy = yposT*16;
    printf("* Pen Size [1 = wireframe]: ");
    switch (oled.PenSize(1))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            return -1;
    }
    printf("* Rectangle (should box in the test text) : ");
    switch (oled.Rectangle(cpx-2, cpy-2, cpx + 12*4 +4, cpy + 20, RED))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            return -1;
    }
    printf("* Copy/Paste\n");
    for (i = 0; i < NCPT; ++i)
    {
        printf("\tTrial #%d: [src = %d, %d] [dst = %d, %d] [dim = %d, %d]",
               i+1, cpx, cpy, XCP[i], YCP[i], 12*4, 16);
        switch (oled.CopyPaste(cpx, cpy, XCP[i], YCP[i], 12*4, 16))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                return -1;
        }
    }

    // TEST: int  SetVolume(uchar value)
    snprintf(TVOL[0], 32, "[UNMUTE]");
    snprintf(TVOL[1], 32, "[MID VOL]");
    snprintf(TVOL[2], 32, "[UP 1]");
    snprintf(TVOL[3], 32, "[DOWN 1]");
    snprintf(TVOL[4], 32, "[UP 8]");
    snprintf(TVOL[5], 32, "[DOWN 8]");
    snprintf(TVOL[6], 32, "[INVALID]");
    snprintf(TVOL[7], 32, "[MAX VOL]");
    snprintf(TVOL[8], 32, "[MIN VOL]");
    snprintf(TVOL[9], 32, "[MUTE]");
    for (i = 0; i < NVOL; ++i)
    {
        printf("* Volume 0x%.2X %s: ", VOLS[i], TVOL[i]);
        switch (oled.SetVolume(VOLS[i]))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                return -1;
        }
    }

    // TEST: int  ReadPin(uchar pin, uchar *status)
    // TEST: int  WritePin(uchar pin, uchar value)
    printf("* ReadPin/WritePin\n");
    for (i = 0; i < 16; ++i)
    {
        if (testRWPIN(&oled, i))
        {
            oled.Close();
            return -1;
        }
    }

    // TEST: int  ReadBus(uchar *status)
    // TEST: int  WriteBus(uchar value)
    printf("* ReadBus/WriteBus\n");
    if (testRWBUS(&oled))
    {
        oled.Close();
        return -1;
    }

    // TEST: int  ShowChar(uchar glyph, uchar col, uchar row, ushort color)
    usleep(3000000);
    oled.Clear();
    printf("* ShowChar:\n");
    for (i = 0; i < 4; ++i)
    {
        printf("\tFont(%d): ", i);
        switch (oled.SetFont(i))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                oled.Close();
                return -1;
        }
        printf("\t'%c': ", 0x41 + i);
        switch (oled.ShowChar(0x41+i, i*2, i, BKGD[i]))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                oled.Close();
                return -1;
        }
        printf("\t'%c': ", 0x61 + i);
        switch (oled.ShowChar(0x61+i, i*2 + 1, i, BKGD[i]))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAILED\n");
                printf("%s\n", oled.GetError());
                fflush(stdout);
                oled.Close();
                return -1;
        }
    }

    // TEST: int  ScaleChar(uchar glyph, ushort x, ushort y, ushort color, uchar width, uchar height)
    usleep(5000000);
    oled.Clear();
    printf("* ScaleChar 'A': ");
    gettimeofday(&ts, NULL);
    switch (oled.ScaleChar('A', 0, 0, RED, 20, 20))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            oled.Close();
            return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);

    // TEST: int  ScaleString(ushort x, ushort y, uchar font, ushort color, uchar width, uchar height,
    //                     const char *data);
    usleep(5000000);
    oled.Clear();
    printf("* ScaleString \"Scale\": ");
    gettimeofday(&ts, NULL);
    switch (oled.ScaleString(0, 3, 1, RED, 6, 6, "Scale"))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            oled.Close();
            return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);

    // TEST: int  Line(ushort x1, ushort y1, ushort x2, ushort y2, ushort color)
    usleep(5000000);
    oled.Clear();
    printf("* Line Test (star):\n");
    oled.ShowString(3, 1, 3, PURPLE, "This is a star");
    midx = ver.hres/2 -1;
    if (drawStar(midx, ver.vres/2 -1, midx -1.0, &oled))
    {
        oled.Close();
        return -1;
    }

    usleep(5000000);
    oled.Clear();
    printf("* Polygon Test:\n");
    oled.ShowString(4, 1, 3, ORANGE, "Polygon Test");
    if (drawPoly(midx, ver.vres/2 -1, &oled))
    {
        oled.Close();
        return -1;
    }
    usleep(10000000);

    // TEST: int  Ellipse(ushort x, ushort y, ushort rx, ushort ry, ushort color)
    // NOTE: test Pen Size as well as Ellipse - use wireframe to draw 2 concentric
    // circles over 2 equal elipses with swapped major/minor axes
    oled.Clear();
    printf("* Ellipse and PenSize Test:\n");
    oled.ShowString(4, 1, 3, GREEN, "Ellipse Test");
    if (drawEllipse(midx, ver.vres/2 -1, &oled))
    {
        oled.Close();
        return -1;
    }
    usleep(5000000);


    // TEST: int  Triangle(ushort x1, ushort y1, ushort x2, ushort y2,
    //              ushort x3, ushort y3, ushort color)
    printf("* Triangle bug test; see if the filled triangle renders correctly;\n");
    printf("\tthe triangle is outlined in blue, the fill is red.\n");
    oled.Clear();
    oled.ShowString(1, 1, 3, BLUE, "Triangle Bug Test");
    oled.ShowString(1, 4, 1, BLUE, "The filled triangle does not render correctly");
#define VX1 (0)
#define VY1 (60)
#define VX2 (0)
#define VY2 (319)
#define VX3 (239)
#define VY3 (160)
    oled.PenSize(0);
    oled.Triangle(VX1, VY1, VX2, VY2, VX3, VY3, RED);
    oled.PenSize(1);
    oled.Triangle(VX1, VY1, VX2, VY2, VX3, VY3, BLUE);
    usleep(10000000);
    oled.Clear();
    printf("* Triangle Test:\n");
    oled.ShowString(3, 1, 3, BLUE, "Triangle  Test");
    gettimeofday(&ts, NULL);
    if (drawTriangle(ver.hres, ver.vres, &oled))
    {
        oled.Close();
        return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);
    usleep(15000000);

    // TEST: int  Circle(ushort x, ushort y, ushort radius, ushort color)
    printf("* Circle Test:\n");
    oled.Clear();
    oled.ShowString(4, 1, 3, WHITE, "Circle Test");
    if (drawCircle(ver.hres, ver.vres, &oled))
    {
        oled.Close();
        return -1;
    }
    usleep(5000000);

    // TEST: int  Rectangle(ushort x1, ushort y1, ushort x2, ushort y2, ushort color)
    printf("* Rectangle Test:\n");
    oled.Clear();
    oled.ShowString(3, 1, 3, BLUE, "Rectangle Test");
    if (drawRect(ver.hres, ver.vres, &oled))
    {
        oled.Close();
        return -1;
    }
    usleep(5000000);

    // TEST: int  SetOpacity(uchar mode)
    printf("* Opacity [transparent]: ");
    switch (oled.SetOpacity(0))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            oled.Close();
            return -1;
    }
    oled.ShowString(2, 6, 3, BLACK, "TRANSPARENT TEXT");
    // TEST: int  SetBackground(ushort color)
    printf("* Set Background [green]: ");
    switch (oled.SetBackground(GREEN))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            oled.Close();
            return -1;
    }
    printf("* Opacity [solid]: ");
    switch (oled.SetOpacity(1))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            oled.Close();
            return -1;
    }
    oled.ShowString(4, 8, 3, RED, "OPAQUE TEXT");
    oled.SetBackground(BLACK);
    usleep(5000000);

    // TEST: int  ReplaceColor(ushort x1, ushort y1, ushort x2, ushort y2,
    //                      ushort oldcolor, ushort newcolor)
    printf("* ReplaceColor [RED, YEL]: ");
    gettimeofday(&ts, NULL);
    switch (oled.ReplaceColor(0, 0, ver.hres-1, ver.vres-1, RED, YELLOW))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", oled.GetError());
            fflush(stdout);
            oled.Close();
            return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);

    // TEST: int  WritePixel(ushort x, ushort y, ushort color)
    // TEST: int  ReadPixel(ushort x, ushort y, ushort *color)
    printf("* Read/Write pixel:\n");
    gettimeofday(&ts, NULL);
    if (testPixRW(ver.hres, ver.vres,&oled))
    {
        oled.Close();
        return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);
    usleep(500000);

    // TEST: int  AddBitmap(uchar group, uchar index, const uchar *data, int datalen)
    // TEST: DrawBitmap(uchar group, uchar index, ushort x, ushort y, ushort color)
    printf("* Add/Draw bitmap (small):\n");
    oled.Clear();
    oled.ShowString(2, 0, 1, RED, "SMALL BITMAP");
    gettimeofday(&ts, NULL);
    if (testBMsmall(ver.hres, ver.vres, &oled))
    {
        oled.Close();
        return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);
    usleep(2000000);

    printf("* Add/Draw bitmap (medium):\n");
    oled.Clear();
    oled.ShowString(2, 0, 1, RED, "MEDIUM BITMAP");
    gettimeofday(&ts, NULL);
    if (testBMmed(ver.hres, ver.vres, &oled))
    {
        oled.Close();
        return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);
    usleep(2000000);

    printf("* Add/Draw bitmap (big):\n");
    oled.Clear();
    oled.ShowString(2, 0, 1, RED, "HUGE BITMAP");
    gettimeofday(&ts, NULL);
    if (testBMbig(ver.hres, ver.vres, &oled))
    {
        oled.Close();
        return -1;
    }
    gettimeofday(&te, NULL);
    calctime(ts, te);
    usleep(2000000);

    // TEST: int  Button(bool pressed, ushort x, ushort y, ushort bcolor, uchar font,
    //                ushort tcolor, uchar xmul, uchar ymul, const char *text);
    LATEST:
    switch (oled.Button(false, 0, 50, YELLOW, 3, BLACK, 1, 1, "TEST A"))
    {
        // XXX -
    }
    for (i = 1; i < 10; ++ i)
    {
        oled.Button((i % 2) ? false : true, 0, 50, YELLOW, 3, BLACK, 1, 1, "TEST A");
        usleep(2000000);
    }
    usleep(2000000);

    // TEST: int  DrawIcon(ushort x, ushort y, ushort width, ushort height,
    //              uchar colormode, const uchar *data, int datalen)
    oled.Ctl(4, 1); // Landscape format; note as per documentation that it is the
                    // default orientation setting and not the current orientation
                    // setting which affects the rendering of Draw Image Icon
    oled.Clear();
    printf("* Draw Icon (render image): ");
    fflush(stdout);
    do {
        unsigned short data[76800];
        size_t fs;
        FILE *fp = fopen("test.img", "r");
        if ((fp) && ((fs = fread(data, 2, 76800, fp)) == 76800))
        {
            fclose(fp);
            gettimeofday(&ts, NULL);
            switch (oled.DrawIcon(0, 0, 320, 240, 16, (unsigned char *)data, 153600))
            {
                case 0:
                    printf("OK\n");
                    gettimeofday(&te, NULL);
                    calctime(ts, te);
                    break;
                case 1:
                    printf("FAIL (NACK)\n");
                    break;
                case 2:
                    printf("[timeout]\n");
                    break;
                default:
                    printf("FAIL (see message below)\n%s\n", oled.GetError());
                    break;
            }
        }
        else
        {
            if (fp) fclose(fp);
            printf("FAIL (cannot load image): %s\n", (fs >= 0) ? "file is too short" : strerror(errno));
        }
    } while (0);
    usleep(2000);
    oled.Ctl(4, 3); // Portrait format
    usleep(15000000);

    printf("* Touch tests\n");
    oled.Clear();
    oled.ShowString(0, 1, 3, RED, "Touch Tests");
    if (testTouch(&oled, &globs))
    {
        oled.Close();
        return -1;
    }
    usleep(2000000);

    // TEST SLEEP: int  Suspend(uchar options, uchar duration)
    printf("* Suspend, wake on Touch [10s timeout]: ");
    fflush(stdout);
    oled.Clear();
    oled.ShowString(0, 1, 3, RED, "Suspended for 10s");
    oled.ShowString(0, 4, 1, YELLOW, "touch screen to wake up");
    globs.wait = true;
    switch(oled.Suspend(2, 10))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        case 2:
            while (globs.wait) usleep(200000);
            break;
        default:
            printf("FAIL\n");
    }
    printf("%s\n", globs.result ? "OK" : "NACK/FAIL");

    oled.Clear();
    oled.ShowString(0, 1, 3, RED, "Tests Completed");

    printf("* End of tests\n\n");
    usleep(10000000);
    oled.Close();
    return 0;
} // main()



int drawStar(unsigned short midx, unsigned short midy, unsigned short rad, PGD *pgd)
{
    unsigned short starX[5], starY[5];
    float alph = pi/2.0;
    float alphi = pi/2.5;
    int idx[6] = {0, 2, 4, 1, 3, 0};
    int i, j, k;
    for (i = 0; i < 5; ++i)
    {
        starX[i] = midx + rad*cosf(alph);
        starY[i] = midy + rad*sinf(alph);
        alph += alphi;
    }
    for (i = 0; i < 5; ++i)
    {
        printf("\tLine %d: ", i + 1);
        j = idx[i];
        k = idx[i+1];
        switch (pgd->Line(starX[j], starY[j], starX[k], starY[k], PURPLE))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAIL\n");
                printf("%s\n", pgd->GetError());
                fflush(stdout);
                return -1;
        }
    }
    return 0;
}



int testRWPIN(PGD *pgd, int i)
{
    unsigned char pinstate;
    printf("\tPin %d:\n", i);
    printf("\t\tRead: ");
    switch (pgd->ReadPin(i, &pinstate))
    {
        case 0:
            printf("OK [pin = %c]\n", pinstate ? '1' : '0');
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\t\tWrite [1]: ");
    switch (pgd->WritePin(i, 1))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\t\tRead: ");
    switch (pgd->ReadPin(i, &pinstate))
    {
        case 0:
            printf("OK [pin = %c]\n", pinstate ? '1' : '0');
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\t\tWrite [0]: ");
    switch (pgd->WritePin(i, 0))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\t\tRead: ");
    switch (pgd->ReadPin(i, &pinstate))
    {
        case 0:
            printf("OK [pin = %c]\n", pinstate ? '1' : '0');
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    return 0;
}



int testRWBUS(PGD *pgd)
{
    unsigned char pinstate;
    printf("\tRead: ");
    switch (pgd->ReadBus(&pinstate))
    {
        case 0:
            printf("OK [P8..P15 = 0x%.2X]\n", pinstate);
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tWrite [0xff]: ");
    switch (pgd->WriteBus(0xff))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tRead: ");
    switch (pgd->ReadBus(&pinstate))
    {
        case 0:
            printf("OK [P8..P15 = 0x%.2X]\n", pinstate);
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tWrite [0x00]: ");
    switch (pgd->WriteBus(0x00))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tRead: ");
    switch (pgd->ReadBus(&pinstate))
    {
        case 0:
            printf("OK [P8..P15 = 0x%.2X]\n", pinstate);
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAILED\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    return 0;
}



int drawPoly(unsigned short midx, unsigned short midy, PGD *pgd)
{
    unsigned short polyX[5][7], polyY[5][7];
    float alph;
    float alphi;
    float rad;
    int i, j, k;
    unsigned short col[5] = {ORANGE, RED, GREEN, BLUE, WHITE};
    for (i = 0; i < 5; ++i)
    {
        k = i + 3;
        alph = pi/2.0;
        alphi = 2.0*pi/k;
        rad = (i+1)*(midx -1.0)/5.0;
        for (j = 0; j < k; ++j)
        {
            polyX[i][j] = midx + rad*cosf(alph);
            polyY[i][j] = midy + rad*sinf(alph);
            alph += alphi;
        }
    }

    for (i = 0; i < 5; ++i)
    {
        printf("\t%d-sided polygon: ", i + 3);
        switch (pgd->Polygon(i +3, polyX[i], polyY[i], col[i]))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAIL\n");
                printf("%s\n", pgd->GetError());
                fflush(stdout);
                return -1;
        }
    }
    return 0;
}

int drawEllipse(unsigned short midx, unsigned short midy, PGD *pgd)
{
    // TEST: int  Ellipse(ushort x, ushort y, ushort rx, ushort ry, ushort color)
    // test Pen Size as well as Ellipse - use wireframe to draw 2 concentric
    // circles over 2 equal elipses with swapped major/minor axes
    unsigned short major = midx -1;
    unsigned short minor = midx/4;

    printf("\tPenSize(0): ");
    switch (pgd->PenSize(0))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tSolid Ellipse 1: ");
    switch (pgd->Ellipse(midx, midy, major, minor, GREEN))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tSolid Ellipse 2: ");
    switch (pgd->Ellipse(midx, midy, minor, major, BLUE))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tPenSize(1): ");
    switch (pgd->PenSize(1))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tWire Ellipse (circle) 1: ");
    minor <<= 1;
    switch (pgd->Ellipse(midx, midy, minor, minor, RED))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tWire Ellipse (circle) 2: ");
    switch (pgd->Ellipse(midx, midy, major, major, PURPLE))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    return 0;
}

int triangle(unsigned short x1, unsigned short y1, unsigned short x2, unsigned short y2,
             unsigned short x3, unsigned short y3, unsigned short color, PGD *pgd)
{
    printf("\ttriangle [(%d, %d), (%d, %d), (%d, %d), 0x%.2X]: ",
          x1, y1, x2, y2, x3, y3, color & 0xffff);
    switch (pgd->Triangle(x1, y1, x2, y2, x3, y3, color))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    return 0;
}

int drawTriangle(unsigned short width, unsigned short height, PGD *pgd)
{
    // Escher's impossible triangle
    float cos30 = cosf(pi/6.0);
    //float L = width -1;
    float L = 219;  // this value is used to skirt a bug in the uOLED firmware
    float h = L*cos30;
    float t = h/12.0;

    unsigned short x[20], y[20];

    x[0] = t/cos30;
    x[1] = L - 2*x[0];
    x[2] = L - x[0];
    x[3] = x[0]/2;
    x[4] = x[3] + x[0];
    x[5] = x[4] + x[0];
    x[6] = x[1]- x[0]/2;
    x[7] = x[6] + x[0];
    x[8] = x[7] + x[0];
    x[9] = x[0];
    x[10] = x[0]*3;
    x[11] = L - x[10];
    x[12] = x[1];
    x[13] = L/2;
    x[14] = x[13] - x[0]/2;
    x[15] = x[13] + x[0]/2;
    x[16] = x[13];
    x[17] = x[16] + x[0];
    x[18] = x[14];
    x[19] = x[15];


    y[0] = y[1] = y[2] = height/2 - h/2;
    y[3] = y[4] = y[5] = y[6] = y[7] = y[8] = y[0] + t;
    y[9] = y[10] = y[11] = y[12] = y[3] + t;
    y[13] = y[0] + h - 4*t;
    y[14] = y[15] = y[13] + t;
    y[16] = y[17] = y[14] + t;
    y[18] = y[19] = y[16] + t;

    pgd->PenSize(0);
#define TRI(I, J, K, C) do {\
    if (triangle(x[I], y[I], x[J], y[J], x[K], y[K], C, pgd)) return -1;\
    } while (0)
    // large triangles
    TRI(7, 3, 18, GREEN);
    TRI(4, 19, 8, BLUE);
    TRI(0, 16, 2, RED);
    // inner triangles
    TRI(4, 19, 8, BLUE);
    TRI(4, 14, 6, GREEN);
    TRI(5, 13, 6, RED);
    // black center
    TRI(10, 13, 11, BLACK);

    // red rhomboids
    TRI(0, 3, 4, RED);
    TRI(3, 9, 4, RED);
    TRI(6, 11, 12, RED);
    TRI(6, 12, 7, RED);
    // blue rhomboids
    TRI(1, 7, 2, BLUE);
    TRI(2, 7, 8, BLUE);
    // green rhomboids
    TRI(16, 18, 19, GREEN);
    TRI(16, 19, 17, GREEN);

#define LINES(I,J) do {\
    pgd->Line(x[I], y[I], x[J], y[J], BLACK);} while (0)

    // outer outline
    LINES(0, 3);
    LINES(3, 18);
    LINES(18, 19);
    LINES(19, 8);
    LINES(8, 2);
    LINES(2, 0);
    // red outline
    LINES(8, 4);
    LINES(4, 0);
    LINES(3, 9);
    LINES(9, 4);
    LINES(10, 12);
    // green outline
    LINES(4, 16);
    LINES(16, 18);
    LINES(16, 17);
    LINES(5, 13);
    // blue outline
    LINES(14, 11);
    LINES(18, 12);

    return 0;
}


int drawCircle(unsigned short width, unsigned short height, PGD *pgd)
{
    int i, idx;
    int rad = (width -1)/2;
    int midy = height/2;
    unsigned short COL[5] = {RED, GREEN, BLUE, ORANGE, WHITE};
    pgd->PenSize(0);
    idx = 0;
    for (i = rad; i > 0; i -= rad/6)
    {
        printf("\tFilled Circle [%d]: ", i);
        switch (pgd->Circle(rad, midy, i, COL[(idx++)%5]))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAIL\n");
                printf("%s\n", pgd->GetError());
                fflush(stdout);
                return -1;
        }
    }
    pgd->PenSize(1);
    for (i = rad; i > 0; i -= rad/12)
    {
        printf("\tWire Circle [%d]: ", i);
        switch (pgd->Circle(rad, midy, i, BLACK))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAIL\n");
                printf("%s\n", pgd->GetError());
                fflush(stdout);
                return -1;
        }
    }
    return 0;
}

int drawRect(unsigned short width, unsigned short height, PGD *pgd)
{
    int i, idx;

    int h = height/16;
    int midx = (width -1)/2;
    int w = midx/6;
    unsigned short COL[5] = {RED, GREEN, BLUE, ORANGE, WHITE};
    pgd->PenSize(0);
    idx = 0;
    int x0, y0, x1, y1;
    for (i = 0; i < 5; ++i)
    {
        printf("\tFilled Rect #%d: ", i);
        x0 = i*w;
        x1 = width -i*w -1;
        y0 = 32 + i*h;
        y1 = height -(i*h) -1;
        switch (pgd->Rectangle(x0, y0, x1, y1, COL[(idx++)%5]))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAIL\n");
                printf("%s\n", pgd->GetError());
                fflush(stdout);
                return -1;
        }
    }
    pgd->PenSize(1);
    h /= 2;
    w /= 2;
    for (i = 0; i < 12; ++i)
    {
        printf("\tWire Rect #%d: ", i);
        x0 = i*w;
        x1 = width -i*w -1;
        y0 = 32 + i*h;
        y1 = height -(i*h) -1;
        switch (pgd->Rectangle(x0, y0, x1, y1, BLACK))
        {
            case 0:
                printf("OK\n");
                break;
            case 1:
                printf("NACK\n");
                break;
            default:
                printf("FAIL\n");
                printf("%s\n", pgd->GetError());
                fflush(stdout);
                return -1;
        }
    }
    return 0;
}


int testPixRW(unsigned short width, unsigned short height, PGD *pgd)
{
    int x, y;
    unsigned short color;
    x = width/2;
    y = 0;
    printf("\tRead pixel (%d, %d): ", x, y);
    switch (pgd->ReadPixel(x, y, &color))
    {
        case 0:
            printf("OK [color = 0x%.4X]\n", color&0xffff);
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    color = ~color;
    printf("\tWrite pixel (%d, %d) [0x%.4X]: ", x, y, color & 0xffff);
    switch (pgd->WritePixel(x, y, color))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tRead pixel (%d, %d): ", x, y);
    switch (pgd->ReadPixel(x, y, &color))
    {
        case 0:
            printf("OK [color = 0x%.4X]\n", color&0xffff);
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    for (y = 1; y < height; ++y)
    {
        pgd->ReadPixel(x, y, &color);
        color = ~color;
        pgd->WritePixel(x, y, color);
    }
    return 0;
}

int testBMsmall(unsigned short width, unsigned short height, PGD *pgd)
{
    uchar icon[8] = {0x40, 0x80, 0x19, 0x26, 0x74, 0x98, 0x01, 0x02};
    printf("\tAdd Bitmap [0, 63]: ");
    switch (pgd->AddBitmap(0, 63, icon, 8))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tDraw Bitmap [0, 63, (0, 8), GREEN]: ");
    switch (pgd->DrawBitmap(0, 63, 0, 8, GREEN))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    int x, y;
    for (y = 8; y < height; y += 8)
    {
        for (x = 0; x < width; x += 8)
        {
            pgd->DrawBitmap(0, 63, x, y, GREEN);
        }
    }
    return 0;
}

int testBMmed(unsigned short width, unsigned short height, PGD *pgd)
{
    uchar icon[32] = {0x02, 0x02, 0x04, 0x04, 0x04, 0x0a, 0x04, 0x11,
        0x87, 0x30, 0x40, 0xc8, 0x20, 0x84, 0x11, 0x02,
        0x0a, 0x02, 0x07, 0xfe, 0x08, 0x10, 0x10, 0x10,
        0x20, 0x28, 0x40, 0x44, 0x80, 0x82, 0x01, 0x01};
    printf("\tAdd Bitmap [1, 15]: ");
    switch (pgd->AddBitmap(1, 15, icon, 32))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tDraw Bitmap [1, 15, (0, 8), RED]: ");
    switch (pgd->DrawBitmap(1, 15, 0, 8, RED))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    int x, y;
    for (y = 8; y < height; y += 16)
    {
        for (x = 0; x < width; x += 16)
        {
            pgd->DrawBitmap(1, 15, x, y, RED);
        }
    }
    return 0;
}

int testBMbig(unsigned short width, unsigned short height, PGD *pgd)
{
    uchar icon[128] = {
        0x0f, 0xf8, 0x1f, 0xf0, 0x08, 0x18, 0x18, 0x10,
        0x10, 0x18, 0x18, 0x08, 0x20, 0x0c, 0x30, 0x04,
        0xc0, 0x06, 0x60, 0x03, 0x80, 0x03, 0xc0, 0x01,
        0x80, 0x01, 0x80, 0x01, 0x80, 0x03, 0xc0, 0x01,
        0x80, 0x06, 0x60, 0x01, 0xf8, 0x0c, 0x30, 0x1f,
        0x2c, 0x18, 0x18, 0x34, 0x26, 0x19, 0x98, 0x64,
        0xa3, 0x1a, 0x58, 0xc5, 0x71, 0x9c, 0x39, 0x8e,
        0x08, 0xd8, 0x1b, 0x10, 0x04, 0x38, 0x1a, 0x20,
        0x04, 0x38, 0x1a, 0x20, 0x08, 0xd8, 0x1b, 0x10,
        0x71, 0x9c, 0x39, 0x8e, 0xa3, 0x1a, 0x58, 0xc5,
        0x26, 0x19, 0x98, 0x64, 0x2c, 0x18, 0x18, 0x34,
        0xf8, 0x0c, 0x30, 0x1f, 0x80, 0x06, 0x60, 0x01,
        0x80, 0x03, 0xc0, 0x01, 0x80, 0x01, 0x80, 0x01,
        0x80, 0x03, 0xc0, 0x01, 0xc0, 0x06, 0x60, 0x03,
        0x20, 0x0c, 0x30, 0x04, 0x10, 0x18, 0x18, 0x08,
        0x08, 0x18, 0x18, 0x10, 0x0f, 0xf8, 0x1f, 0xf0,
    };
    printf("\tAdd Bitmap [2, 7]: ");
    switch (pgd->AddBitmap(2, 7, icon, 128))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    printf("\tDraw Bitmap [2, 7, (0, 8), BLUE]: ");
    switch (pgd->DrawBitmap(2, 7, 0, 8, BLUE))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            fflush(stdout);
            return -1;
    }
    int x, y;
    for (y = 8; y < height; y += 32)
    {
        for (x = 0; x < width; x += 32)
        {
            pgd->DrawBitmap(2, 7, x, y, BLUE);
        }
    }
    return 0;
}



int testTouch(PGD *pgd, GLOBS *globs)
{
    printf("* Touch pad OFF: ");
    switch (pgd->Ctl(DM_TOUCHPAD, 1))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            return -1;
    }
    printf("* Touch Get Status: ");
    volatile unsigned short coord[2];
    switch (pgd->GetTouch(TM_STATUS, (unsigned short *) coord))
    {
        case 0:
            printf("OK ");
            switch (coord[0]&0xff)
            {
                case 0:
                    printf("[no activity]\n");
                    break;
                case 1:
                    printf("[press]\n");
                    break;
                case 2:
                    printf("[release]\n");
                    break;
                case 3:
                    printf("[moving]\n");
                    break;
                default:
                    printf("[unexpected: 0x%.2X]\n", coord[0]&0xff);
                    break;
            }
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            return -1;
    }
    printf("* Touch Get Coords: ");
    switch (pgd->GetTouch(TM_COORD, (unsigned short *) coord))
    {
        case 0:
            printf("OK [%d, %d]\n", coord[0], coord[1]);
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            return -1;
    }
    printf("* Touch pad ON: ");
    switch (pgd->Ctl(DM_TOUCHPAD, 0))
    {
        case 0:
            printf("OK\n");
            break;
        case 1:
            printf("NACK\n");
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            return -1;
    }
    globs->wait = true;
    unsigned short wt = 5000;
    printf("* WaitTouch [%u]: ", wt);
    fflush(stdout);
    switch(pgd->WaitTouch(wt))
    {
        case 0:
            printf("ACK\n");
            globs->wait = false;
            break;
        case 1:
            printf("NACK\n");
            globs->wait = false;
            break;
        case 2:
            do {
                int i = 0;
                while (globs->wait)
                {
                    usleep(wt*100);
                    if (++i > 15) break;
                }
            } while (0);
            break;
        default:
            break;
    }
    if (globs->wait)
        printf("[no response after timeout]\n");
    else
        printf("%s\n", globs->result ? "ACK" : "NACK/FAIL");
    if (globs->result)
    {
        if (!pgd->GetTouch(TM_STATUS, (unsigned short *) coord))
            printf("> status: %d\n", coord[0]&0xff);
        if (!pgd->GetTouch(TM_COORD, (unsigned short *) coord))
            printf("> coordinates [%d, %d]\n", coord[0], coord[1]);
    }
    printf("* Touch Wait Press: ");
    fflush(stdout);
    pgd->ShowString(0, 4, 1, YELLOW, "touch screen to continue");
    globs->wait = true;
    switch (pgd->GetTouch(TM_PRESS, (unsigned short *) coord))
    {
        case 0:
        case 2:
            break;
        default:
            printf("FAIL\n");
            printf("%s\n", pgd->GetError());
            return -1;
    }
    while (globs->wait)
    {
        usleep(200000);
    }
    printf("OK [%d, %d]\n", coord[0], coord[1]);
    return 0;
}
