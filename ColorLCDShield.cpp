/*
  LCDShield.cpp - Arduino Library to control a Nokia 6100 LCD, 
  specifically that found on SparkFun's Color LCD Shield.
  This code should work for both Epson and Phillips display drivers 
  normally found on the Color LCD Shield.
	
  License: CC BY-SA 3.0: Creative Commons Share-alike 3.0. Feel free 
  to use and abuse this code however you'd like. If you find it useful
  please attribute, and SHARE-ALIKE!
  
  This is based on code by Mark Sproul, and Peter Davenport.
  Thanks to Coleman Sellers and Harold Timmis for help getting it to work with the Phillips Driver 7-31-2011
*/

#include "ColorLCDShield.h"

namespace
{

void LCDCommand(uint8_t data)
{
    cbi(SPCR, SPE); // Temporarily disable hardware SPI

    cbi(LCD_PORT_CS, CS);     // enable chip

    cbi(LCD_PORT_DIO, DIO);   // output low on data out (9th bit low = command)

    cbi(LCD_PORT_SCK, SCK);   // send clock pulse
    sbi(LCD_PORT_SCK, SCK);   // send clock pulse

    // Do the rest via hardware SPI
    sbi(SPCR, SPE); // Enable SPI
    SPDR = data; // Send data

    // Wait until finished
    while (!(SPSR & (1<<SPIF)))
        ;

    LCD_PORT_CS	|=	(1<<CS);  // disable
}

// Inlined to get some extra performance (at the cost of space)
void LCDData(uint8_t data)
{
    cbi(SPCR, SPE); // Temporarily disable hardware SPI

    cbi(LCD_PORT_CS, CS);     // enable chip

    sbi(LCD_PORT_DIO, DIO);   // output high on data out (9th bit high = data)

    cbi(LCD_PORT_SCK, SCK);   // send clock pulse
    sbi(LCD_PORT_SCK, SCK);   // send clock pulse

    // Do the rest via hardware SPI
    sbi(SPCR, SPE); // Enable SPI
    SPDR = data; // Send data

    // Wait until finished
    while (!(SPSR & (1<<SPIF)))
        ;

    LCD_PORT_CS	|=	(1<<CS);  // disable
}

// Use this for speed (trading space)
#define FastLCDData(data) \
 { \
    cbi(SPCR, SPE);  \
    cbi(LCD_PORT_CS, CS); \
    sbi(LCD_PORT_DIO, DIO); \
    cbi(LCD_PORT_SCK, SCK); \
    sbi(LCD_PORT_SCK, SCK); \
    sbi(SPCR, SPE); \
    SPDR = data; \
    while (!(SPSR & (1<<SPIF))) \
        ; \
    LCD_PORT_CS	|=	(1<<CS); \
 }

}

LCDShield::LCDShield()
{
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
	DDRB = ((1<<DIO)|(1<<SCK));     //Set DIO and SCK pins on PORTB as outputs
	DDRH = ((1<<CS)|(1<<LCD_RES));  //Set CS and RES pins PORTH as outputs
#else
	DDRB = ((1<<CS)|(1<<DIO)|(1<<SCK)|(1<<LCD_RES));  //Set the control pins as outputs
#endif

    // UNDONE: Why is this here??
	DDRD	=	0x00;
	PORTD	=	0xFF;

    // Initialize hardware SPI
    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);
    // Use standard 4 MHz SPI clock. SPI is disabled when sending a command/data flag
    SPCR = (1<<SPE) | (1<<MSTR);
}

void LCDShield::init(uint8_t type)
{
	driver = type;

	delay(200);

	cbi(LCD_PORT_SCK, SCK);     //CLK = LOW
	cbi(LCD_PORT_DIO, DIO);     //DIO = LOW
	delayMicroseconds(10);
	sbi(LCD_PORT_CS, CS);       //CS = HIGH
	delayMicroseconds(10);
	cbi(LCD_PORT_RES, LCD_RES); //RESET = LOW
	delay(200);
	sbi(LCD_PORT_RES, LCD_RES); //RESET = HIGH
	delay(200);
	sbi(LCD_PORT_SCK, SCK);     // SCK = HIGH
	sbi(LCD_PORT_DIO, DIO);     // DIO = HIGH
	delayMicroseconds(10);

    if (type == EPSON)
    {
        LCDCommand(DISCTL);   // display control(EPSON)
        LCDData(0x0C);        // 12 = 1100 - CL dividing ratio [don't divide] switching period 8H (default)
        LCDData(0x20);
        LCDData(0x00);
        LCDData(0x01);

        LCDCommand(COMSCN);   // common scanning direction(EPSON)
        LCDData(0x01);

        LCDCommand(OSCON);    // internal oscialltor ON(EPSON)

        LCDCommand(SLPOUT);   // sleep out(EPSON)

        LCDCommand(PWRCTR);   // power ctrl(EPSON)
        LCDData(0x0F);        //everything on, no external reference resistors

        LCDCommand(DISINV);   // invert display mode(EPSON)

        LCDCommand(DATCTL);   // data control(EPSON)
        LCDData(0x03);        // correct for normal sin7
        LCDData(0x00);        // normal RGB arrangement
        LCDData(0x02);        // 16-bit Grayscale Type A

        LCDCommand(VOLCTR);   // electronic volume, this is the contrast/brightness(EPSON)
        LCDData(0x24);        // volume (contrast) setting - fine tuning, original
        LCDData(0x03);        // internal resistor ratio - coarse adjustment

        LCDCommand(NOP);      // nop(EPSON)

        delayMicroseconds(200);

        LCDCommand(DISON);    // display on(EPSON)

    }
    else
    {
        LCDCommand(SLEEPOUT); //sleep out(PHILLIPS)
        LCDCommand(BSTRON);   //Booset On(PHILLIPS)

        LCDCommand(COLMOD);   // Set Color Mode(PHILLIPS)
        LCDData(0x03);

        LCDCommand(MADCTL);   // Memory Access Control(PHILLIPS)
        LCDData(0x0); // UNDONE: Configurable?

        LCDCommand(SETCON);   // Set Contrast(PHILLIPS)
        LCDData(0x30);

        LCDCommand(NOPP);     // nop(PHILLIPS)

        delayMicroseconds(200);

        LCDCommand(DISPON);   // display on(PHILLIPS)
    }
}

void LCDShield::clear(int color)
{
    if (driver == EPSON) // if it's an Epson
	{
		LCDCommand(PASET);
		LCDData(0);
        LCDData(COL_HEIGHT-1);

		LCDCommand(CASET);
		LCDData(0);
        LCDData(ROW_LENGTH-1);

		LCDCommand(RAMWR);
	}
	else // otherwise it's a phillips
	{
		LCDCommand(PASETP);
		LCDData(0);
        LCDData(COL_HEIGHT-1);

		LCDCommand(CASETP);
		LCDData(0);
        LCDData(ROW_LENGTH-1);

		LCDCommand(RAMWRP);
	}

    for (unsigned int i=0; i < (ROW_LENGTH*COL_HEIGHT)/2; i++)
	{
		LCDData((color>>4)&0x00FF);
        LCDData(((color&0x0F)<<4) | ((color>>8) & 0xF));
        LCDData(color & 0x0FF);
	}
}

void LCDShield::contrast(uint8_t setting)
{
    if (driver == EPSON)
    {
        LCDCommand(VOLCTR);      // electronic volume, this is the contrast/brightness(EPSON)
        LCDData(setting);        // volume (contrast) setting - course adjustment,  -- original was 24
        LCDCommand(NOP);         // nop(EPSON)
    }
    else
    {
        LCDCommand(SETCON);
        LCDData(setting);
    }
}

void LCDShield::setPixel(int color, uint8_t x, uint8_t y)
{
    if (driver == EPSON) // if it's an epson
	{
        LCDCommand(CASET);  // column start/end ram
        LCDData(x);
        LCDData(ENDCOL);

		LCDCommand(PASET);  // page start/end ram
        LCDData(y);
		LCDData(ENDPAGE);

		LCDCommand(RAMWR);  // write
		LCDData((color>>4)&0x00FF);
		LCDData(((color&0x0F)<<4)|(color>>8));
		LCDData(color&0x0FF);
	}
	else  // otherwise it's a phillips
	{
		LCDCommand(PASETP); // page start/end ram
        LCDData(y);
        LCDData(y);

		LCDCommand(CASETP); // column start/end ram
        LCDData(x);
        LCDData(x);

		LCDCommand(RAMWRP); // write

        LCDData((uint8_t)((color>>4)&0x00FF));
        LCDData((uint8_t)(((color&0x0F)<<4)|0x00));
	}
}

void LCDShield::setCircle(uint8_t x0, uint8_t y0, uint8_t radius, int color)
{
	int f = 1 - radius;
	int ddF_x = 0;
	int ddF_y = -2 * radius;
	int x = 0;
	int y = radius;

	setPixel(color, x0, y0 + radius);
	setPixel(color, x0, y0 - radius);
	setPixel(color, x0 + radius, y0);
	setPixel(color, x0 - radius, y0);

	while(x < y)
	{
		if(f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x + 1;

		setPixel(color, x0 + x, y0 + y);
		setPixel(color, x0 - x, y0 + y);
		setPixel(color, x0 + x, y0 - y);
		setPixel(color, x0 - x, y0 - y);
		setPixel(color, x0 + y, y0 + x);
		setPixel(color, x0 - y, y0 + x);
		setPixel(color, x0 + y, y0 - x);
		setPixel(color, x0 - y, y0 - x);
	}
}

void LCDShield::setChar(char c, uint8_t x, uint8_t y, int fColor, int bColor)
{
	int             i,j;
	unsigned int    nCols;
	unsigned int    nRows;
	unsigned int    nBytes;
    uint8_t   PixelRow;
    uint8_t   Mask;
	unsigned int    Word0;
	unsigned int    Word1;
    uint16_t fontindex;

    nCols = pgm_read_byte(&(FONT8x16[0][0]));
    nRows = pgm_read_byte(&(FONT8x16[0][1]));
    nBytes = pgm_read_byte(&(FONT8x16[0][2]));
    fontindex = (nBytes * (c - 0x1F)) + nBytes - 1;

    if (driver == EPSON)	// If it's an epson
	{
		// Row address set (command 0x2B)
		LCDCommand(PASET);
        LCDData(y);
        LCDData(y + nRows - 1);

		// Column address set (command 0x2A)
		LCDCommand(CASET);
        LCDData(x);
        LCDData(x + nCols - 1);

		// WRITE MEMORY
		LCDCommand(RAMWR);
    }
    else
    {
        // Row address set (command 0x2B)
        LCDCommand(PASETP);
        LCDData(y);
        LCDData(y + nRows - 1);

        // Column address set (command 0x2A)
        LCDCommand(CASETP);
        LCDData(x);
        LCDData(x + nCols - 1);

        // WRITE MEMORY
        LCDCommand(RAMWRP);
    }

    for (i = 0; i < nRows; ++i)
    {
        PixelRow = pgm_read_byte((uint8_t *)FONT8x16 + fontindex + i);

        // loop on each pixel in the row (left to right)
        // Note: we do two pixels each loop
        Mask = 0x80;
        for (j = 0; j < nCols; j += 2)
        {
            // if pixel bit set, use foreground color; else use the background color
            // now get the pixel color for two successive pixels
            if ((PixelRow & Mask) == 0)
                Word0 = bColor;
            else
                Word0 = fColor;

            Mask = Mask >> 1;
            if ((PixelRow & Mask) == 0)
                Word1 = bColor;
            else
                Word1 = fColor;

            Mask = Mask >> 1;

            // use this information to output three data bytes
            LCDData((Word0 >> 4) & 0xFF);
            LCDData(((Word0 & 0xF) << 4) | ((Word1 >> 8) & 0xF));
            LCDData(Word1 & 0xFF);
        }
    }
}

void LCDShield::setStr(const char *pString, uint8_t x, uint8_t y, int fColor, int bColor)
{
	// loop until null-terminator is seen
	while (*pString != 0x00) {
		// draw the character
		setChar(*pString++, x, y, fColor, bColor);
		// advance the y position
        x = x + 8;
        // bail out if x exceeds 131
        if (x > 131) break;
	}
}

void LCDShield::setLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, int color)
{
	int dy = y1 - y0; // Difference between y0 and y1
	int dx = x1 - x0; // Difference between x0 and x1
	int stepx, stepy;

	if (dy < 0)
	{
		dy = -dy;
		stepy = -1;
	}
	else
		stepy = 1;

	if (dx < 0)
	{
		dx = -dx;
		stepx = -1;
	}
	else
		stepx = 1;

	dy <<= 1; // dy is now 2*dy
	dx <<= 1; // dx is now 2*dx
	setPixel(color, x0, y0);

	if (dx > dy) 
	{
		int fraction = dy - (dx >> 1);
		while (x0 != x1)
		{
			if (fraction >= 0)
			{
				y0 += stepy;
				fraction -= dx;
			}
			x0 += stepx;
			fraction += dy;
			setPixel(color, x0, y0);
		}
	}
	else
	{
		int fraction = dx - (dy >> 1);
		while (y0 != y1)
		{
			if (fraction >= 0)
			{
				x0 += stepx;
				fraction -= dy;
			}
            y0 += stepy;
            fraction += dx;
            setPixel(color, x0, y0);
		}
	}
}

void LCDShield::setRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool fill,
                        int color)
{
	// check if the rectangle is to be filled
    if (fill)
	{
		int xDiff;
	
		if(x0 > x1)
			xDiff = x0 - x1; //Find the difference between the x vars
		else
			xDiff = x1 - x0;
	
		while(xDiff > 0)
		{
			setLine(x0, y0, x0, y1, color);
		
			if(x0 > x1)
				x0--;
			else
				x0++;
		
			xDiff--;
		}

	}
	else 
	{
		// best way to draw an unfilled rectangle is to draw four lines
		setLine(x0, y0, x1, y0, color);
		setLine(x0, y1, x1, y1, color);
		setLine(x0, y0, x0, y1, color);
		setLine(x1, y0, x1, y1, color);
	}
}

#ifdef WITH_SPARKFUN_LOGO
void LCDShield::printLogo(void)
{
	int x = 4, y = 25, logo_ix = 0, z;
	char logo;

	for (logo_ix = 0; logo_ix < 1120; logo_ix++)
	{
		logo = logo_spark[logo_ix];
		for (z = 0; z < 8; z++)
		{
			if ((logo & 0x80) == 0x80) setPixel(RED, y, x);
			x++;
			if (x == 132)
			{
				x = 4;
				y++;
			}
			logo <<= 1;
		}
	}
}
#endif

void LCDShield::drawPixels(uint8_t *pixels, uint16_t pxsize, uint8_t x0, uint8_t y0,
                           uint8_t x1, uint8_t y1)
{
    // Draw raw pixels in 12 bit colour format. It is assumed that pixels are combined
    // (1.5 byte each) to speed things up.
    // UNDONE: Only supports PHILIPS

    LCDCommand(PASETP); // page start/end ram
    LCDData(y0);
    LCDData(y1);

    LCDCommand(CASETP); // column start/end ram
    LCDData(x0);
    LCDData(x1);

    LCDCommand(RAMWRP); // write

    for (uint16_t i=0; i<pxsize; ++i)
        FastLCDData(pixels[i]);

    LCDCommand(NOPP);
}

void LCDShield::off(void)
{
    if (driver == EPSON)	// If it's an epson
		LCDCommand(DISOFF);
	else // otherwise it's a phillips
		LCDCommand(DISPOFF);
}

void LCDShield::on(void)
{
    if (driver == EPSON)	// If it's an epson
		LCDCommand(DISON);
	else // otherwise it's a phillips
		LCDCommand(DISPON);
}

