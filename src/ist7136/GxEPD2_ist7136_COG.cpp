#include "GxEPD2_ist7136_COG.h"

GxEPD2_ist7136_COG::GxEPD2_ist7136_COG(int16_t cs, int16_t dc, int16_t rst, int16_t busy) : GxEPD2_EPD(cs, dc, rst, busy, LOW, 10000000, WIDTH, HEIGHT, panel, hasColor, hasPartialUpdate, hasFastPartialUpdate)
{
}

void GxEPD2_ist7136_COG::clearScreen(uint8_t value)
{
    writeScreenBuffer(0x10);
    writeScreenBuffer(0x11, 0x00);
    refresh();
    writeScreenBuffer(0x10);
    writeScreenBuffer(0x11, 0x00);
}

void GxEPD2_ist7136_COG::writeScreenBuffer(uint8_t reg, uint8_t value)
{   
    _writeCommand(reg);
    _startTransfer();
    for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
    {
        _transfer(value);
    }
    _endTransfer();
    delay(1); //yield() avoid WDT on ESP8266 and ESP32
}

void GxEPD2_ist7136_COG::writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
    uint16_t wb = (w + 7) / 8;                                      // width bytes, bitmaps are padded
    x -= x % 8;                                                     // byte boundary
    w = wb * 8;                                                     // byte boundary
    int16_t x1 = x < 0 ? 0 : x;                                     // limit
    int16_t y1 = y < 0 ? 0 : y;                                     // limit
    int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x;   // limit
    int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
    int16_t dx = x1 - x;
    int16_t dy = y1 - y;
    w1 -= dx;
    h1 -= dy;
    if ((w1 <= 0) || (h1 <= 0))
        return;
    
    _writeCommand(0x10);
    _startTransfer();
    for (int16_t i = 0; i < h1; i++)
    {
        for (int16_t j = 0; j < w1 / 8; j++)
        {
            uint8_t data;
            // use wb, h of bitmap for index!
            uint16_t idx = mirror_y ? j + dx / 8 + uint16_t((h - 1 - (i + dy))) * wb : j + dx / 8 + uint16_t(i + dy) * wb;
            if (pgm)
            {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
                data = pgm_read_byte(&bitmap[idx]);
#else
                data = bitmap[idx];
#endif
            }
            else
            {
                data = bitmap[idx];
            }
            if (invert)
                data = ~data;
            _transfer(data);
        }
    }
    _endTransfer();
    delay(1);            // yield() to avoid WDT on ESP8266 and ESP32
    writeScreenBuffer(0x11, 0x00);
}

void GxEPD2_ist7136_COG::writeImage(const uint8_t *black, const uint8_t *color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
    if (black)
    {
        writeImage(black, x, y, w, h, invert, mirror_y, pgm);
    }
}

void GxEPD2_ist7136_COG::writeNative(const uint8_t *data1, const uint8_t *data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
    if (data1)
    {
        writeImage(data1, x, y, w, h, invert, mirror_y, pgm);
    }
}

void GxEPD2_ist7136_COG::get_busy_status(uint8_t busy)
{
    busy = digitalRead(_busy);
}

void GxEPD2_ist7136_COG::refresh()
{
    _Init_Full();
    _waitWhileBusy("refresh", _busy_timeout);
    _write_command_data(0x15, 0x3c);
}

void GxEPD2_ist7136_COG::powerOff(void)
{
    _PowerOff();
}

void GxEPD2_ist7136_COG::hibernate()
{
    _PowerOff();
}

void GxEPD2_ist7136_COG::_PowerOn()
{
    if (!_power_is_on)
    {
        _writeCommand(0x04);
        _waitWhileBusy("_PowerOn", power_on_time);
    }
    _power_is_on = true;
}

void GxEPD2_ist7136_COG::_PowerOff()
{
    if (_power_is_on)
    {
        _waitWhileBusy("_PowerOff", power_off_time);
        _write_command_data(0x09,0x7f);
        _write_command_data(0x05,0x7d);
        _write_command_data(0x09,0x00);
        _waitWhileBusy("_PowerOff", power_off_time);
        digitalWrite(_dc, LOW);
        digitalWrite(_cs, LOW);
        digitalWrite(_rst, LOW);
    }
    _power_is_on = false;
}

void GxEPD2_ist7136_COG::_InitDisplay()
{
    if (_hibernating)
        _reset();

    // Initial COG
    _write_command_data(0x05, 0x7d);
    delay(200);
    _write_command_data(0x05, 0x00);
    delay(10);
    //_write_command_data(0xc2, 0x3f); // values from original library maby needs to be enabled
    _write_command_data(0xd8, 0x00);
    _write_command_data(0xd6, 0x00);
    _write_command_data(0xa7, 0x10);
    delay(100);
    _write_command_data(0xa7, 0x00);
    delay(100);
    _write_command_data(0x44, 0x00);
    _write_command_data(0x45,0x80);
    _write_command_data(0xa7, 0x10);
    delay(100);
    _write_command_data(0xa7, 0x00);
    delay(100);
    _write_command_data(0x44, 0x06);
    _write_command_data(0x45, 0x80);
    _write_command_data(0xa7, 0x10);
    delay(100);
    _write_command_data(0xa7, 0x00);
    delay(100);
    _write_command_data(0x60, 0x25);
    _write_command_data(0x61, 0x00);
    _write_command_data(0x02, 0x00);
}

void GxEPD2_ist7136_COG::_write_command_data(uint8_t command, uint8_t data)
{
    _writeCommand(command);
    _writeData(data);
}

void GxEPD2_ist7136_COG::_write_command_data(uint8_t command, uint8_t * data, size_t size)
{
    _writeCommand(command);
    _writeData(data, size);
}

void GxEPD2_ist7136_COG::_soft_start_DCDC()
{
    uint8_t reg51[] = {0x50, 0x01, 0x0a, 0x01};
    uint8_t reg09[] = {0x1f, 0x9f, 0x7f, 0xff};
    _write_command_data(0x51, &reg51[1], 2);

    //First Stage
    for(int i = 1; i <= 4; i++)
    {
        _write_command_data(0x09, reg09[0]);
        reg51[1] = i;
        _write_command_data(0x51, &reg51[0], 2);
        _write_command_data(0x09, reg09[1]);
        delay(2);
    }

    //Second Stage
    for(int i = 1; i <= 10; i++)
    {
        _write_command_data(0x09, reg09[0]);
        reg51[3] = i;
        _write_command_data(0x51, &reg51[2], 2);
        _write_command_data(0x09, reg09[1]);
        delay(2);
    }

    //Third Stage
    for(int i = 3; i <= 10; i++)
    {
        _write_command_data(0x09, reg09[2]);
        reg51[3] = i;
        _write_command_data(0x51, &reg51[2], 2);
        _write_command_data(0x09, reg09[3]);
        delay(2);
    }
    // Fourth Stage 
    for(int i = 9; i >= 2; i--)
    {
        _write_command_data(0x09, reg09[2]);
        reg51[2] = i;
        _write_command_data(0x51, &reg51[2], 2);
        _write_command_data(0x09, reg09[3]);
        delay(2);
    }
    _write_command_data(0x09, reg09[3]);
    delay(10);
}

void GxEPD2_ist7136_COG::_Init_Full()
{
    _InitDisplay();
    _soft_start_DCDC();
    _PowerOn();
    _using_partial_mode = false;
}


void GxEPD2_ist7136_COG::_reset()
{
    pinMode(_rst, OUTPUT);
    delay(200);                          // delay_ms 5ms
    digitalWrite(_rst, HIGH); // RES# = 1
    delay(20);                          // delay_ms 5ms
    digitalWrite(_rst, LOW);
    delay(200);
    digitalWrite(_rst, HIGH);
    delay(50);

    digitalWrite(_cs, HIGH); // CS# = 1

    delay(5);
}