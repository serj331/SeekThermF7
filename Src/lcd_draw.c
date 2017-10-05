#include "stm32f7xx_hal.h"
#include "fonts.h"

extern uint16_t* LCDLay1FrameBuf;

void LCD_DrawPixel(uint16_t xPos, uint16_t yPos, uint16_t color) {
	LCDLay1FrameBuf[480 * yPos + xPos] = color;
}

void LCD_DrawChar(sFONT* font, uint16_t xPos, uint16_t yPos, char c) {
	int fontWidthB = ((Font24.Width + 7) / 8);
	int fontSize =  Font24.Height * fontWidthB;
	uint8_t ascii = c;
	for(int i = 0; i < Font24.Height; i++) {
		for(int j = 0; j < Font24.Width; j++) {
			if((Font24.table[(i * fontWidthB + (j / 8)) + (fontSize * (ascii - ' '))] >> (7 - j % 8)) & 1) {
				LCD_DrawPixel(xPos + j, yPos + i, 0xFF1F);
			} else {
				//background
			}
		}
	}
}

void LCD_DrawString(sFONT* font, uint16_t xPos, uint16_t yPos, char* str) {
	char c;
	while((c = *str++) != '\0') {
		LCD_DrawChar(font, xPos, yPos, c);
		xPos += font->Width;
	}
}
