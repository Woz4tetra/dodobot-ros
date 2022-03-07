#ifndef __DODOBOT_SD_H__
#define __DODOBOT_SD_H__

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <AnimatedGIF.h>

#include "dodobot.h"
#include "display-dodobot.h"


using namespace dodobot_display;


#define minimum(a,b)     (((a) < (b)) ? (a) : (b))


namespace dodobot_sd
{
    const int chipSelect = BUILTIN_SDCARD;
    bool initialized = false;

    void print_dir(File dir, int num_spaces);
    void print_spaces(int num);

    void setup()
    {
        if (SD.begin(chipSelect)) {
            initialized = true;
        }
        else {
            dodobot_serial::println_error("SD card failed to initialize!");
        }
    }

    void list_dir(String dirname)
    {
        if (!initialized) {
            return;
        }
        File root = SD.open(dirname.c_str());

        while (true) {
            File entry = root.openNextFile();
            if (!entry) {
                break;
            }

            if (!entry.isDirectory()) {
                DODOBOT_SERIAL_WRITE_BOTH("listdir", "sd", entry.name(), entry.size());
            }
            entry.close();
        }
    }

    void list_all_files()
    {
        if (!initialized) {
            return;
        }
        File root = SD.open("/");
        print_dir(root, 0);
    }

    void print_dir(File dir, int num_spaces)
    {
        while (true) {
            File entry = dir.openNextFile();
            if (!entry) {
                break;
            }
            print_spaces(num_spaces);
            INFO_SERIAL.print(entry.name());
            if (entry.isDirectory()) {
                INFO_SERIAL.println("/");
                print_dir(entry, num_spaces+2);
            } else {
                // files have sizes, directories do not
                print_spaces(48 - num_spaces - strlen(entry.name()));
                INFO_SERIAL.print("  ");
                INFO_SERIAL.println(entry.size(), DEC);
            }
            entry.close();
        }
    }

    void print_spaces(int num) {
        for (int i=0; i < num; i++) {
            INFO_SERIAL.print(" ");
        }
    }

    //
    // File from serial
    //

    String dest_path;
    bool file_is_open = false;

    // uint32_t file_max_len = 0x1020;
    // uint8_t* file_buffer = new uint8_t[file_max_len];

    uint32_t file_size = 0;
    int32_t prev_segment_index = 0;

    File file;

    void set_dest_path(String path) {
        dest_path = path;
        dodobot_serial::println_info("Setting path to: %s", dest_path.c_str());
    }

    void open_file()
    {
        if (file_is_open) {
            return;
        }
        const char* path = dest_path.c_str();
        if (SD.exists(path)) {
            SD.remove(path);
        }
        dodobot_serial::println_info("Opening path: %s", path);
        file = SD.open(path, FILE_WRITE);
        dodobot_serial::println_info("File name: %s", file.name());
        file_is_open = true;
        file_size = 0;
    }

    void delete_file(String filename)
    {
        const char* path = filename.c_str();
        if (SD.exists(path)) {
            dodobot_serial::println_info("Deleting file: %s", path);
            SD.remove(path);
        }
    }

    void close_file()
    {
        if (!file_is_open) {
            return;
        }
        dodobot_serial::println_info("File name: %s", file.name());
        file.close();
        file_is_open = false;
    }

    bool append_to_buffer(char* file_bytes, uint32_t segment_size)
    {
        if (dest_path.length() == 0) {
            dodobot_serial::println_error("Destination path not set for file");
            return false;
        }
        if (!file_is_open) {
            dodobot_serial::println_error("Destination file is not open");
            return false;
        }

        // if (file_offset + segment_size > file_max_len) {
        //     dodobot_serial::println_error("Loaded image is larger than max size: %d > %d", file_offset + segment_size, file_max_len);
        //     return false;
        // }
        // memcpy(file_buffer + file_offset, file_bytes, segment_size);
        // file_size = file_offset + segment_size;
        file.write(file_bytes, segment_size);
        file_size += segment_size;
        dodobot_serial::println_info("File len: %d", file_size);
        return true;
    }

    // bool write_buffer()
    // {
    //     if (dest_path.length() == 0) {
    //         dodobot_serial::println_error("Destination path not set for file");
    //         return false;
    //     }
    //     size_t written = file.write(file_buffer, file_size);
    //     file.close();
    //     if (written != file_size) {
    //         dodobot_serial::println_error("Wrote a different number of bytes from expected: %d != %d", (int)written, (int)file_size);
    //         return false;
    //     }
    //     return true;
    // }

    //
    // Animated GIFs
    //

    AnimatedGIF gif;
    GIFINFO gif_info;
    static File FSGifFile; // temp gif file holder

    static int xOffset = 0;
    static int yOffset = 0;

    static void * GIFOpenFile(const char *fname, int32_t *pSize)
    {
        //log_d("GIFOpenFile( %s )\n", fname );
        FSGifFile = SD.open(fname);
        if (FSGifFile) {
            *pSize = FSGifFile.size();
            return (void *)&FSGifFile;
        }
        return NULL;
    }


    static void GIFCloseFile(void *pHandle)
    {
        File *f = static_cast<File *>(pHandle);
        if (f != NULL) {
            f->close();
        }
    }


    static int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
    {
        int32_t iBytesRead;
        iBytesRead = iLen;
        File *f = static_cast<File *>(pFile->fHandle);
        // Note: If you read a file all the way to the last byte, seek() stops working
        if ((pFile->iSize - pFile->iPos) < iLen) {
            iBytesRead = pFile->iSize - pFile->iPos - 1; // <-- ugly work-around
        }
        if (iBytesRead <= 0) {
            return 0;
        }
        iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
        pFile->iPos = f->position();
        return iBytesRead;
    }


    static int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition)
    {
        int i = micros();
        File *f = static_cast<File *>(pFile->fHandle);
        f->seek(iPosition);
        pFile->iPos = (int32_t)f->position();
        i = micros() - i;
        //log_d("Seek time = %d us\n", i);
        return pFile->iPos;
    }


    static void TFTDraw(int x, int y, int w, int h, uint16_t* lBuf )
    {
        tft.drawRGBBitmap(x + xOffset, y + yOffset, lBuf, w, h);
    }


    // Draw a line of image directly on the LCD
    void GIFDraw(GIFDRAW *pDraw)
    {
        uint8_t *s;
        uint16_t *d, *usPalette, usTemp[320];
        int x, y, iWidth;

        iWidth = pDraw->iWidth;
        if (iWidth > tft.width()) {
            iWidth = tft.width();
        }
        usPalette = pDraw->pPalette;
        y = pDraw->iY + pDraw->y; // current line

        s = pDraw->pPixels;
        if (pDraw->ucDisposalMethod == 2) {// restore to background color
            for (x=0; x<iWidth; x++) {
                if (s[x] == pDraw->ucTransparent) {
                    s[x] = pDraw->ucBackground;
                }
            }
            pDraw->ucHasTransparency = 0;
        }
        // Apply the new pixels to the main image
        if (pDraw->ucHasTransparency) { // if transparency used
            uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
            int x, iCount;
            pEnd = s + iWidth;
            x = 0;
            iCount = 0; // count non-transparent pixels
            while (x < iWidth) {
                c = ucTransparent-1;
                d = usTemp;
                while (c != ucTransparent && s < pEnd) {
                    c = *s++;
                    if (c == ucTransparent) { // done, stop
                        s--; // back up to treat it like transparent
                    }
                    else { // opaque
                        *d++ = usPalette[c];
                        iCount++;
                    }
                } // while looking for opaque pixels
                if (iCount) { // any opaque pixels?
                    TFTDraw( pDraw->iX+x, y, iCount, 1, (uint16_t*)usTemp );
                    x += iCount;
                    iCount = 0;
                }
                // no, look for a run of transparent pixels
                c = ucTransparent;
                while (c == ucTransparent && s < pEnd) {
                    c = *s++;
                    if (c == ucTransparent) {
                        iCount++;
                    }
                    else {
                        s--;
                    }
                }
                if (iCount) {
                    x += iCount; // skip these
                    iCount = 0;
                }
            }
        } else {
            s = pDraw->pPixels;
            // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
            for (x=0; x<iWidth; x++) {
                usTemp[x] = usPalette[*s++];
            }
            TFTDraw( pDraw->iX, y, iWidth, 1, (uint16_t*)usTemp );
        }
    } /* GIFDraw() */

    void setGIFoffset(int x, int y)
    {
        xOffset = x;
        yOffset = y;
    }

    bool is_gif_loaded = false;
    int32_t current_frame = 0;

    int loadGIF(const char* path)
    {
        if (!SD.exists(path)) {
            dodobot_serial::println_error("Path %s doesn't exist", path);
            return -1;
        }

        gif.begin(LITTLE_ENDIAN_PIXELS);

        if (!gif.open(path, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
            dodobot_serial::println_error("Could not open gif %s", path);
            return -1;
        }


        dodobot_serial::println_info("Opened GIF. Canvas size: %dx%d\n", gif.getCanvasWidth(), gif.getCanvasHeight());

        setGIFoffset(0, 0);

        if (gif.getInfo(&gif_info)) {
            dodobot_serial::println_info("frame count: %d\n", gif_info.iFrameCount);
            dodobot_serial::println_info("duration: %d ms\n", gif_info.iDuration);
            dodobot_serial::println_info("max delay: %d ms\n", gif_info.iMaxDelay);
            dodobot_serial::println_info("min delay: %d ms\n", gif_info.iMinDelay);
        }

        is_gif_loaded = true;
        return gif_info.iFrameCount;
    }

    int drawGIFframe()
    {
        if (!is_gif_loaded) {
            return -1;
        }
        int frameDelay = 0;
        int result = gif.playFrame(true, &frameDelay);
        current_frame++;
        if (result != -1) {
            return result;
        }
        if (current_frame >= gif_info.iFrameCount) {
            return 0;
        }

        return 1;
    }

    void closeGIF() {
        if (!is_gif_loaded) {
            return;
        }
        is_gif_loaded = false;
        gif.close();
    }

    // JPEGs

    void printJPEGInfo()
    {
        dodobot_serial::println_info("JPEG image info");
        dodobot_serial::println_info("  Width      : %d", JpegDec.width);
        dodobot_serial::println_info("  Height     : %d", JpegDec.height);
        dodobot_serial::println_info("  Components : %d", JpegDec.comps);
        dodobot_serial::println_info("  MCU / row  : %d", JpegDec.MCUSPerRow);
        dodobot_serial::println_info("  MCU / col  : %d", JpegDec.MCUSPerCol);
        dodobot_serial::println_info("  Scan type  : %d", JpegDec.scanType);
        dodobot_serial::println_info("  MCU width  : %d", JpegDec.MCUWidth);
        dodobot_serial::println_info("  MCU height : %d", JpegDec.MCUHeight);
    }

    const uint32_t image_array_max_len = 0x10000;
    uint8_t* image_array = new uint8_t[image_array_max_len];

    void drawJPEG(const char* path, int16_t xpos, int16_t ypos)
    {
        File file = SD.open(path);
        if (!file) {
            dodobot_serial::println_error("Failed to open path: %s", path);
        }

        uint32_t image_array_size = 0;
        while (file.available()) {
        	image_array[image_array_size++] = file.read();
        }
        JpegDec.decodeArray(image_array, image_array_size);
        printJPEGInfo();

        // retrieve infomration about the image
        uint16_t *pImg;
        uint16_t mcu_w = JpegDec.MCUWidth;
        uint16_t mcu_h = JpegDec.MCUHeight;
        uint32_t max_x = JpegDec.width;
        uint32_t max_y = JpegDec.height;

        // Jpeg images are drawn as a set of image block (tiles) called Minimum Coding Units (MCUs)
        // Typically these MCUs are 16x16 pixel blocks
        // Determine the width and height of the right and bottom edge image blocks
        uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
        uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

        // save the current image block size
        uint32_t win_w = mcu_w;
        uint32_t win_h = mcu_h;

        uint32_t screen_w = tft.width();
        uint32_t screen_h = tft.height();

        // record the current time so we can measure how long it takes to draw an image
        uint32_t drawTime = millis();

        // save the coordinate of the right and bottom edges to assist image cropping
        // to the screen size
        max_x += xpos;
        max_y += ypos;

        // read each MCU block until there are no more
        tft.startWrite();
        // tft.setAddrWindow(xpos, ypos, minimum(max_x, screen_w), minimum(max_y, screen_h));
        while ( JpegDec.read()) {

            // save a pointer to the image block
            pImg = JpegDec.pImage;

            // calculate where the image block should be drawn on the screen
            unsigned int mcu_x = JpegDec.MCUx * mcu_w + xpos;
            unsigned int mcu_y = JpegDec.MCUy * mcu_h + ypos;

            // check if the image block size needs to be changed for the right and bottom edges
            if (mcu_x + mcu_w <= max_x) { win_w = mcu_w; }
            else { win_w = min_w; }

            if (mcu_y + mcu_h <= max_y) { win_h = mcu_h; }
            else { win_h = min_h; }

            // calculate how many pixels must be drawn
            uint32_t mcu_pixels = win_w * win_h;

            // draw image block if it will fit on the screen
            if ( ( mcu_x + win_w) <= screen_w && ( mcu_y + win_h) <= screen_h) {
                // open a window onto the screen to paint the pixels into
                // tft.startWrite();
                tft.setAddrWindow(mcu_x, mcu_y, win_w, win_h);
                // push all the image block pixels to the screen
                while (mcu_pixels--) {
                    tft.writeColor(*pImg++, 1); // Send to TFT 16 bits at a time
                }
                // tft.endWrite();
            }

            // stop drawing blocks if the bottom of the screen has been reached
            // the abort function will close the file
            else if ( ( mcu_y + win_h) >= screen_h) {
                JpegDec.abort();
            }

        }
        tft.endWrite();

        // calculate how long it took to draw the image
        drawTime = millis() - drawTime; // Calculate the time it took

        // print the results to the serial port
        dodobot_serial::println_info("Total render time was: %d ms", drawTime);
    }

    //
    // Breakout levels
    //

    uint16_t max_files = 100;
    String* list_filenames = new String[max_files];
    uint32_t num_breakout_files = 0;

    bool list_breakout_files()
    {
        if (!initialized) {
            return false;
        }
        File root = SD.open("/");
        num_breakout_files = 0;
        while (true) {
            File entry = root.openNextFile();
            if (!entry) {
                break;
            }
            if (!entry.isDirectory())
            {
                if (strstr(entry.name(), "BR-") != NULL)
                {
                    String name = entry.name();
                    list_filenames[num_breakout_files++] = name;
                    if (num_breakout_files >= max_files) {
                        break;
                    }
                }
            }
            entry.close();
        }
        return num_breakout_files > 0;
    }

    uint32_t max_level_size = 100;
    char* level_array = new char[max_level_size];
    uint32_t level_size = 0;

    String read_breakout_file(String* loaded_name, uint32_t index)
    {
        if (num_breakout_files == 0) {
            return "";
        }
        if (index >= num_breakout_files) {
            index = num_breakout_files - 1;
        }
        *loaded_name = list_filenames[index];
        File file = SD.open(list_filenames[index].c_str());
        if (!file) {
            return "";
        }

        level_size = 0;
        while (file.available()) {
        	level_array[level_size++] = file.read();
        }
        level_array[level_size] = '\0';
        file.close();

        String level_config = String(level_array);
        return level_config;
    }

    int prev_breakout_index = 100000;
    String load_breakout_level(String* loaded_name, int& requested_index)
    {
        if (requested_index < -1) {  // -1: random index
            return "";
        }
        if (!list_breakout_files()) {
            return "";
        }

        if (requested_index == -1)
        {
            randomSeed(micros());
            do {
                requested_index = (int)random(0, num_breakout_files);
            }
            while (requested_index == prev_breakout_index);
        }
        prev_breakout_index = requested_index;

        String level_config = read_breakout_file(loaded_name, requested_index);

        return level_config;
    }
}

#endif  // __DODOBOT_SD_H__
