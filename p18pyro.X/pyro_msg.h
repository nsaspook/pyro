#ifndef MANDM_MSG_H_INCLUDED
#define MANDM_MSG_H_INCLUDED

#include "mandm_defs.h"

/* spinner defines */
#define MAX_SHAPES  6
const rom int8_t spin[MAX_SHAPES][20] = {
	"||//--", // classic LCD version with no \ character
	"||//--\\\\", // classic
	"OOOOOO--__-", // eye blink
	"vv<<^^>>", // point spinner
	"..**x#x#XX||--", // warp portal
	"..ooOOoo" // ball bouncer
};


const rom int8_t \
menutext_template[] = "\
xxxx0\
xxxx1\
xxxx2\
xxxx3\
xxxx4\
xxxx5\
xxxx6\
xxxx7\
xxxx8\
xxxx9\
xxx10\
xxx11\
xxx12\
xxx13\
xxx14\
xxx15\
xxx16\
xxx17\
xxx18\
xxx19\
xxx20\
xxx21\
xxx22\
xxx23\
xxx24\
xxx25\
xxx26\
xxx27\
xxx28\
xxx29\
xxx30\
xxx31\
xxx32\
xxx33\
xxx34\
xxx35\
xxx36\
xxx37\
xxx38\
xxx39\
";

const rom int8_t \
menutext[] = "\
                V80 M      E220 M     GSD M     V80 MS    E220 QEI  V810 MS    VAR V    HELP   \
     \
  x20\
xxx21\
xxx22\
xxx23\
xxx24\
xxx25\
xxx26\
xxx27\
     \
V810_\
MS   \
xxx31\
xxx32\
xxx33\
xxx34\
xxx35\
xxx36\
xxx37\
xxx38\
xxx39\
";

const rom int8_t \
menuselect_free[MAX_MENU + 1][22] = {
	" V80 Manipulator   F",
	" E220 Manipulator  F",
	" GSD Manipulator   F",
	" V80 Var Mass-Slit F",
	" E220 QEI MOTOR    F",
	" Vista Mass-Slit   F",
	" Varian Valves     F",
	" Test Procedure    F",
	" vista 1           F",
	" vista 2           F",
	" vista 3           F",
	" vista 4           F",
	" vista 5           F",
	" Vista Mass-Slit   F",
	" V810 Manipulator  F",
	" vista 8           F",
	" vista 9           F",
	" vista 10          F"
};

const rom int8_t \
menuselect_track[MAX_MENU + 1][22] = {
	" V80 Manipulator   T",
	" E220 Manipulator  T",
	" GSD Manipulator   T",
	" V80 Var Mass-Slit T",
	" E220 QEI MOTOR    T",
	" Vista Mass-Slit   T",
	" Varian Valves     T",
	" Test Procedure    T",
	" Exit              T",
	" V80 Manipulator   T",
	" E220 Manipulator  T",
	" GSD Manipulator   T",
	" vista 5           T",
	" Vista Mass-Slit   T",
	" V810 Manipulator  T",
	" Varian Valves     T",
	" Test Procedure    T",
	" Exit              T"
};

const rom int8_t \
lowbatt0[] = "\n\r Reducing battey Ah rating due to possible discharged battery.\n\r",
	zero0[] = " All current inputs are at zero current. ",
	zero1[] = "\n\r Current sensors zero setpoints have been recalibrated.\n\r",
	zero2[] = "\n\r NOT changing current sensors zero calibration!\n\r",
	zero3[] = " PV Current sensor zero setpoint has been recalibrated.\n\r",
	adcg0[] = "\n\r Select analog channel to calibrate. Enter 0-8 or n to exit. ",
	adcg1[] = "\n\r Analog channel has been recalibrated.\n\r",
	adcg2[] = "\n\r NOT changing analog channel gain calibration!\n\r",
	adcg3[] = "\n\r Press M/m to increase adc offset or L/l to decrease offset. Press Y to save or Q to exit and NOT save.\n\r",
	adcg4[] = "\n\r Saving analog channel gain calibration in EEPROM!\n\r",
	tweak0[] = "\n\r 1: AHIR Ah in  REAL\r\n 2: AHI  Ah in  ADJUSTED\r\n 3: AHO  Ah out REAL\r\n 4: AHOP Ah out ADJUSTED\r\n Select parameter to modify. Enter number or n to exit. \n\r",
	tweak1[] = " Exiting, Parameter has been modified.\n\r",
	tweak2[] = " Exiting, NOT changing Parameter!\n\r",
	tweak3[] = "\r\n Press M/m to increase or L/l to decrease in 100 or 5 units per button press.\r\n Press Y to save or Q to exit and NOT save.\n\r",
	tweak4[] = " Updating parameter in active process memory!\n\r",
	tweak5[] = "\r\n 10X, Press M/m to increase or L/l to decrease in 1000 or 50 units per button press.\r\n Press Y to save or Q to exit and NOT save.\n\r",
	tweak_t1[] = "AHIR",
	tweak_t2[] = "AHI",
	tweak_t3[] = "AHO",
	tweak_t4[] = "AHOP",
	tweak_tw[] = "Wrong Parameter",
	hello0[] = "\n\r Implant Motor Tester for PIC18F8722 frederick.brooks@microchip.com MANDM ",
	hello1[] = " S/N 000001 ",
	hold0[] = "\n\r ccvoltage Monitor is held in the current state until released, \n\r Press y key to release. ",
	hold1[] = "\n\r ccvoltage Monitor has been released and is running.\n\r",
	sync0[] = "\n\r Set battery currently charging to 100% SOC? y/n ",
	sync1[] = "\n\r Battery SOC set to 100%.\n\r",
	sync2[] = "\n\r Battery SOC NOT changed.\n\r",
	keycmds0[] = "\n\r KEY CMDS: # Display run data, ! Hold Program,  T run exerciser tests\n\r",
	keycmds1[] = " K lock program causing reboot, D toggle RS-232 motor tracking logging.\r\n",
	battheader0[] = " # CI  mV B1  mV B2    Ah    mAhir    mAhi    mAho      Wi      Wo  SOC  BSOC    RUN  Weight   ESR   1ESR   2ESR\n\n\r",
	battheader1[] = " #     LDE    LDC     AD    ACC    FCC    FDC   AHUP  MINBV  MAXBV    AHUR   CEF  CEF_CAL CEF_SAV BS_AHU AHIR  BS_AHI  BS_AHO  CEF_RAW\n\n\r",
	modelheader0[] = " Battery Model Data Display\n\n\r",
	modelheader1[] = " Battery Model Data \n\n\r",
	battbuffer0[] = " Block_buffer overflow >SDBUFFERSIZE \r\n",
	sddump0[] = " #### \x1b[7mNo SDCARD Found, Goodbye\x1b[0m ####\r\n",
	sddump1[] = " #### \x1b[7mWrong Type SDCARD Found, Goodbye\x1b[0m ####\r\n",
	sddump2[] = " Update EEP/SD data.\r\n",
	sddump3[] = " Update EEP/SD data, New SDCARD Installed.\r\n",
	divert0[] = " D0  AC Power diversion is ON\r\n",
	divert1[] = " D1  AC Power diversion is OFF\r\n",
	divert99[] = "";

const rom int8_t \
almcode0[] = " A0  Charger on, Battery Low.\r\n",
	almcode1[] = " A1  Charger on, Low Primary Battery, in irq.\r\n",
	almcode2[] = " A2  Charger off, Battery Fresh.\r\n",
	almcode3[] = " A3  Charger off, Failsafe Mode.\r\n",
	almcode4[] = " A4  Charger on, Low Primary battery, in main.\r\n",
	almcode5[] = " A5  Charger on, Battery value SOC, Morning Help.\r\n",
	almcode6[] = " A6  Charger off, Battery value SOC, Morning Help.\r\n",
	almcode7[] = " A7  Charger off, PV Voltage High while charger is on.\r\n",
	almcode8[] = " A8  Charger, PV below SOLARDOWN value EOD.\r\n",
	almcode9[] = " A9  Charger Alarm on, Possible Dead Battery.\r\n",
	almcode10[] = " A10 Day Clock, PV above SOLARUP value Daily Status reset BOD.\r\n",
	almcode11[] = " A11 Charger off, Command or Logic.\r\n",
	almcode12[] = " A12 Charger on,  Command or Logic.\r\n",
	almcode13[] = " A13 Possible Utility AC power glitching.\r\n",
	almcode14[] = " A14 AC power is off.\r\n",
	almcode15[] = " A15 AC power is on.\r\n",
	almcode16[] = " A16 Possible Inverter AC power glitching.\r\n",
	almcode17[] = " A17 Program Error detected.\r\n",
	almcode99[] = "";

const rom int8_t \
chrgcode0[] = " C0  Charging stopped, Battery overvolt limit.\r\n",
	chrgcode1[] = " C1  Charging stopped, PV voltage too low (normal).\r\n",
	chrgcode2[] = " C2  Charging stopped, Terminal FORCEOUT.\r\n",
	chrgcode3[] = " C3  Charging stopped, Battery in FLOAT cycle.\r\n",
	chrgcode4[] = " C4  Charging stopped, PV voltage too low (critical).\r\n",
	chrgcode5[] = " C5  Charging, .\r\n",
	chrgcode6[] = " C6  Charging, OVERRIDE MODE.\r\n",
	chrgcode7[] = " C7  Charging, Battery load test complete.\r\n",
	chrgcode8[] = " C8  Charging, Started, Begin routines.\r\n",
	chrgcode9[] = " C9  Charging, Done, Exit routine.\r\n",
	chrgcode10[] = " C10 Charging, Battery load test starting.\r\n",
	chrgcode11[] = " C11 Charging, Battery load test return.\r\n",
	chrgcode12[] = " C12 Charging, In main charge loop routine.\r\n",
	chrgcode13[] = " C13 Charging, Charger relay switched off.\r\n",
	chrgcode14[] = " C14 Charging, POWER UNSTABLE exit from charging.\r\n",
	chrgcode15[] = " C15 Charging, Skipped battery testing.\r\n",
	chrgcode16[] = " C16 Charging, Critical Battery Tested.\r\n",
	charger0[] = "\n\r C17 Battery in float, charger relay switched off.\n\r",
	charger1[] = "\n\r C18 Inverter battery low, charger relay switched on.\n\r",
	charger2[] = " C19 No charger power detected, charger relay switched off.\n\r",
	charger3[] = " C20  AC Power Charger is ON\r\n",
	charger4[] = " C21  AC Power Charger is OFF\r\n",
	charger99[] = "";

#endif /* MANDM_MSG_H_INCLUDED */

