#include "displays.h"

void lcd_display_line(int8_t * text, uint8_t row)
{

	text[20] = NULL0; // limit the string to 20 chars
	S_WriteCmdXLCD(0b10000000 | row); // SetDDRamAddr
	putsXLCD(text);
}
