/* Switch on and off ISR enables */
#include "crit.h"
#include "pyro.h"

volatile void s_crit(uint8_t mode) // Start critical section of code that needs protection for the ISR
{ // remember old high and low int bits so they can be restored correctly
	static uint8_t H_tmp = 0; // keep track of time spent in critical section

	_asm nop _endasm // asm code to disable compiler optimizations
	LCRIT[CRITC] = INTCONbits.GIEL;
	if (mode & 0x2) INTCONbits.GIEL = LOW;
	HCRIT[CRITC] = INTCONbits.GIEH;
	if (mode & 0x1) INTCONbits.GIEH = LOW;
	critc_count++; // track counts
	CRITC++; // array nest level index
	if (CRITC > critc_level) critc_level = CRITC; // track nesting checking
	if ((uint8_t) CRITC == (uint8_t) CRITC1) { // only update at the first level
		H_tmp = INTCONbits.GIEH;
		INTCONbits.GIEH = LOW;
		INTCONbits.GIEH = H_tmp;
	}
}

volatile void e_crit(void) // End section of code that need protection from ISR
{
	static uint8_t H_tmp = 0;

	_asm nop _endasm // asm code to disable compiler optimizations
	if (CRITC) {
		CRITC--;
		if (!CRITC) { // only update at the first level
			H_tmp = INTCONbits.GIEH;
			INTCONbits.GIEH = LOW;
			INTCONbits.GIEH = H_tmp;
		}
		INTCONbits.GIEH = HCRIT[CRITC];
		INTCONbits.GIEL = LCRIT[CRITC];
	} else { // default to all interrupts on with overflow
		INTCONbits.GIEH = HIGH;
		INTCONbits.GIEL = HIGH;
		TXREG2 = '*'; // restore with no save, output to terminal port
	}
}

volatile void clear_crit(void)
{
	CRITC = NULL0;
	INTCONbits.GIEH = HIGH;
	INTCONbits.GIEL = HIGH;
}

void write_data_eeprom(uint8_t data, uint8_t count, uint16_t addr, uint16_t offset)
{

	//  eeprom data array: 0=CHECKMARK checksum, 1=length of array 2=start of array data, array offset, writes must be protected from ISR
	if (addr == NULL0) { // only write header when on addr 0
		s_crit(HL);
		Busy_eep();
		Write_b_eep(0 + offset, CHECKMARK); //      write checksum  at byte 0 of the offset
		Busy_eep();
		Busy_eep();
		Write_b_eep(1 + offset, count); // length of data
		Busy_eep();
		e_crit();
	}
	s_crit(HL);
	Busy_eep();
	Write_b_eep(addr + 2 + offset, data); //  data
	Busy_eep();
	e_crit();
}

uint8_t read_data_eeprom(uint16_t addr, uint16_t offset)
{
	Busy_eep();
	return Read_b_eep(addr + offset);
}
