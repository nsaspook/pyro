/* High and Low ISR codes, some glue routines */
#include "pyro_vector.h"
#include "mfc.h"

#pragma interrupt tick_handler

void tick_handler(void) // This is the high priority ISR routine
{
    static uint8_t adc_trigger = FALSE, HID_IDLE_FLAG = TRUE, b_read = 0;
    union Timers timer;
    static union adc_buf_type adc_buf;
    static union lcd_buf_type lcd_buf;
    static union spi_buf_type spi_buf;
    static int8_t data, slow = 0;

    DLED_0 = HIGH; // high ISR period flag
    V.highint_count++; // high int counter

    if (PIE1bits.ADIE && PIR1bits.ADIF) { // ADC conversion complete flag
        DLED_5 = LOW;
        V.adc_count++; // just keep count
        PIR1bits.ADIF = LOW;
        adc_buf.buf = ADRES;
        adc_buf.map.index = L.adc_chan; // add channel data to the 16 bit variable
        ringBufS_put(L.rx1b, adc_buf.buf);
        if (!(L.adc_chan & ADC_CHAN_MASK)) {
            DLED_3 = HIGH; // pulse high on ADC channel zero
        } else {
            DLED_3 = LOW;
        }
        L.adc_chan++; // next ADC channel
        ADCON0bits.CHS = L.adc_chan & ADC_CHAN_MASK; // set the current channel
        adc_trigger = FALSE; // reset the skip flag
    }

    /*
     * several khz state machine sequencer timer
     */
    if (PIE3bits.TMR4IE && PIR3bits.TMR4IF) {

        PIR3bits.TMR4IF = LOW;
        PR4 = TIMER4_NORM;
        V.t4_prev = TIMER4_NORM;
        V.pwm4int_count++;

        /*
         *  scan ADC channels
         */
        if (!ADCON0bits.GO) {
            if (adc_trigger++) { // trigger the conversion on the next timer int
                ADCON0bits.GO = HIGH; // and begin A/D conv, will set adc int flag when done.
                DLED_5 = HIGH;
            }
        }

        /*
         * send SPI data
         */
        if (!ringBufS_empty(spi_link.tx1b)) { // SPI send 
            SSP1CON1bits.SSPM = 0;
            spi_buf.buf = ringBufS_get(spi_link.tx1b);
            SPI_LOAD = spi_buf.map.load;
            switch (spi_buf.map.select) { // set device select options
                case 0:
                    DAC_0_CS = LOW;
                    DAC_1_CS = HIGH;
                    SHF_2_CS = HIGH;
                    SHF_3_CS = HIGH;
                    break;
                case 1:
                    DAC_0_CS = HIGH;
                    DAC_1_CS = LOW;
                    SHF_2_CS = HIGH;
                    SHF_3_CS = HIGH;
                    break;
                case 2:
                    DAC_0_CS = HIGH;
                    DAC_1_CS = HIGH;
                    SHF_2_CS = LOW;
                    SHF_3_CS = HIGH;
                    break;
                case 3:
                    DAC_0_CS = HIGH;
                    DAC_1_CS = HIGH;
                    SHF_2_CS = HIGH;
                    SHF_3_CS = LOW;
                    break;
                default:
                    DAC_0_CS = HIGH;
                    DAC_1_CS = HIGH;
                    SHF_2_CS = HIGH;
                    SHF_3_CS = HIGH;
                    break;
            }
            Nop();
            SSP1BUF = spi_buf.map.buf; // transfer the 8 bit data buffer
            if (spi_buf.map.cs) { // dselect device after current transfer ?
                Nop(); // a bit of extra delay
                switch (spi_buf.map.select) { // set device deselect options
                    case 0:
                        DAC_0_CS = HIGH;
                        break;
                    case 1:
                        DAC_1_CS = HIGH;
                        break;
                    case 2:
                        SHF_2_CS = HIGH;
                        break;
                    case 3:
                        SHF_3_CS = HIGH;
                        break;
                    default:
                        DAC_0_CS = HIGH;
                        DAC_1_CS = HIGH;
                        SHF_2_CS = HIGH;
                        SHF_3_CS = HIGH;
                        break;
                }
            }
        }

        /*
         * LCD data handler
         */
        if (!ringBufS_empty(L.tx1b) || lcd_buf.map.state) { // LCD send, 4bit , upper nibble
            switch (lcd_buf.map.state) {
                case 0:
                    lcd_buf.buf = ringBufS_get(L.tx1b);
                    data = lcd_buf.buf;
                    lcd_buf.map.state = 0;
                    lcd_buf.map.skip = 0;
                    TRIS_DATA_PORT &= 0x0f;
                    DATA_PORT &= 0x0f;
                    DATA_PORT |= data & 0xf0;
                    if (lcd_buf.map.cmd) {
                        RS_PIN = 0; // Set control signals for command
                    } else {
                        RS_PIN = 1; // Set control bits for data
                    }
                    RW_PIN = 0;
                    lcd_buf.map.state++;
                    break;
                case 1:
                    E_PIN = 1; // Clock nibble into LCD
                    lcd_buf.map.state++;
                    break;
                case 2:
                    E_PIN = 0;
                    DATA_PORT &= 0x0f;
                    DATA_PORT |= ((data << 4)&0xf0);
                    lcd_buf.map.state++;
                    break;
                case 3:
                    E_PIN = 1; // Clock nibble into LCD
                    lcd_buf.map.state++;
                    break;
                case 4:
                    if (!lcd_buf.map.skip) { // don't repeat if we're in slow time
                        E_PIN = 0;
                        TRIS_DATA_PORT |= 0xf0;
                        V.lcd_count++;
                    }
                    /* fall-through to default people */
                default:
                    if (lcd_buf.map.slow) { // stay in this state until the slow flag clears below
                        lcd_buf.map.skip = 1;
                        if (slow++ >= LCD_SLOW) { // for home and clear commands >3ms delay
                            lcd_buf.map.slow = 0;
                            slow = 0;
                        }
                    } else {
                        lcd_buf.map.state = 0;
                    }
                    break;
            }
            if (!lcd_buf.map.slow)
                PR4 = TIMER4_FAST; // pump the LCD faster than normal
        } else {
            lcd_buf.map.state = 0;
        }
    }

    if (PIE1bits.SSP1IE && PIR1bits.SSP1IF) { // get data from SPI bus 1
        spi_link.count++;
        PIR1bits.SSP1IF = LOW;
        ringBufS_put(spi_link.rx1b, SSP1BUF);
    }

    if (INTCON3bits.INT3IF) { // motor QEI input
        INTCON3bits.INT3IF = LOW;
        V.b3++;
    }

    if (INTCONbits.RBIF) { // PORT B int handler for QEIencoder inputs A/B
        INTCONbits.RBIF = LOW;
        b_read = EXTIO;
        V.buttonint_count++;
        D_UPDATE = TRUE;

        // modified version from www.piclist.com qenc-dk.htm
        if (OldEncoder.OldPortB[0] != (OldEncoder.byTemp[0] = (b_read & QENC1BITS))) { // read encoder 1
            knob1.ticks = 0;
            OldEncoder.OldPortB[0] = OldEncoder.byTemp[0];
            if (ENC_A1 == OldEncoder.A1) {
                if (ENC_B1 == ENC_A1) {
                    ++knob1.c;
                    knob1.cw = TRUE;
                    knob1.ccw = FALSE;
                    knob1.movement = CW;
                } else {
                    --knob1.c;
                    knob1.cw = FALSE;
                    knob1.ccw = TRUE;
                    knob1.movement = CCW;
                }
            } else {
                OldEncoder.A1 = ENC_A1;
            }
        }

        if (OldEncoder.OldPortB[1] != (OldEncoder.byTemp[1] = (b_read & QENC2BITS))) { // read encoder 2
            knob2.ticks = 0;
            OldEncoder.OldPortB[1] = OldEncoder.byTemp[1];
            if (ENC_A2 == OldEncoder.A2) {
                if (ENC_B2 == ENC_A2) {
                    ++knob2.c;
                    knob2.cw = TRUE;
                    knob2.ccw = FALSE;
                    knob2.movement = CW;
                } else {
                    --knob2.c;
                    knob2.cw = FALSE;
                    knob2.ccw = TRUE;
                    knob2.movement = CCW;
                }
            } else {
                OldEncoder.A2 = ENC_A2;
            }
        }
        HID_IDLE_FLAG = FALSE;
    }

    if (PIR2bits.EEIF) { // EEPROM write complete flag
        V.eeprom_count++; // just keep count
        MPULED = !MPULED; //  flash led
        PIR2bits.EEIF = LOW;
    }

    if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer int handler
        INTCONbits.TMR0IF = LOW; //clear interrupt flag
        //check for TMR0 overflow

        timer.lt = TIMEROFFSET; // Copy timer value into union
        TMR0H = timer.bt[HIGH]; // Write high byte to Timer0
        TMR0L = timer.bt[LOW]; // Write low byte to Timer0

        TIMERFLAG = TRUE;

        MPULED = !MPULED; //  flash led
        V.timerint_count++; // set 1 second clock counter.
        DLED_6 = OFF;
        DLED_7 = OFF;
    }

    /* User terminal comm routines */
    if (PIR3bits.RC2IF) { // is data from user command/dump terminal port
        /* clear com2 interrupt flag */
        // a read clears the flag
        V.c2_int++;
        if (RCSTA2bits.OERR) {
            RCSTA2bits.CREN = LOW; //      clear overrun
            RCSTA2bits.CREN = HIGH; // re-enable
        }

        ringBufS_put(L.rx2b, RCREG2); // read from host port2 and clear PIR3bits.RC2IF
    }

    /* Control button routines */
    if (INTCONbits.INT0IF) {
        INTCONbits.INT0IF = LOW;
        V.b0++;
        if (SYSTEM_STABLE && (PB0 == 0u)) {
            button.B0 = 1;
        }
        HID_IDLE_FLAG = FALSE;
    }

    if (INTCON3bits.INT1IF) {
        INTCON3bits.INT1IF = LOW;
        V.b1++;
        if (SYSTEM_STABLE && (PB1 == 0u)) {
            button.B1 = 1;
        }
        HID_IDLE_FLAG = FALSE;
    }

    if (INTCON3bits.INT2IF) {
        INTCON3bits.INT2IF = LOW;
        V.b2++;
        if (SYSTEM_STABLE && (PB2 == 0u)) {
            button.B2 = 1;
        }
        HID_IDLE_FLAG = FALSE;
    }

    V.t4_now = V.t4_prev;
    DLED_0 = LOW;
}

#pragma interruptlow work_handler

void work_handler(void) // This is the low priority ISR routine, the high ISR routine will be called during this code section
{ // lamp scan converter (future), mfc totals
    union Timers timerl;
    uint8_t i;
    union mcp4822_adr_type mfc_dac_select;

    DLED_1 = HIGH; //low ISR period flag
    V.lowint_count++; // low int trigger entropy counter

    if (PIR2bits.TMR3IF) { //      Timer3 int handler
        PIR2bits.TMR3IF = LOW; // clear int flag
        timerl.lt = TIMER3REG; // Save the 16-bit value in local
        TMR3H = timerl.bt[HIGH]; // Write high byte to Timer3 High byte
        TMR3L = timerl.bt[LOW]; // Write low byte to Timer3 Low byte
        V.clock20++;

        for (i = AIR_MFC; i <= COLOR2_MFC; i++) {
            mfc[i].mfc_integ_total_mass += L.adc_val[i];
            mfc[i].mfc_integ_current_mass += L.adc_val[i];

            switch (mfc[i].gas_t) {
                case MASS: /* turn off the MFC at the total mass setpoint target */
                    if (mfc[i].mfc_integ_current_mass >= mfc[i].mfc_integ_target_mass || (mfc[i].flow_time++ >mfc[i].flow_time_max)) {
                        if (mfc[i].flow_time++ >mfc[i].flow_time_max) {
                            mfc->timeout = TRUE;
                        }

                        mfc[i].done = HIGH;
                        mfc_dac_select.buf = i;
                        mfc[i].mfc_set = 0;
                        SPI_Daq_Update(mfc[i].mfc_set, mfc_dac_select.map.cs, mfc_dac_select.map.device);
                    }
                    break;
                case SHUT:
                    mfc[i].mfc_integ_current_mass = 0;
                    break;
                case FLOW:
                    break;
                default:
                    break;
            }
        }
    }
    DLED_1 = LOW;
}

void Clear_All_Buttons(void) {
    button.B0 = 0;
    button.B1 = 0;
    button.B2 = 0;
}
