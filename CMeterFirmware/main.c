#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- ���������

#define DISPLAY_UPDATE_DELAY 2000 /* �������� ��� ���������� �������. ��� ������ ��� �����, ��� ������ ����� ������� ������� � ������ ����� ���������� ������. */
#define PRE_COEFF 38.461 /* ��������������� ����������� ��� �������������� ��������� ��� � ������. ����: 510 �� */
#define CURRENT_CHANGE_SENSITIVITY 0.1 /* ���������������� �� �������� ���� */
#define MINIMUM_CURRENT 0.5 /* ���������� ���������� ������� ��� ���������� ����������, ����� */
#define MINIMUM_VOLTAGE 4.5 /* ���������� ���������� ���������� ������� ���������� */

// ---------------------------------------------------------------- ����������� ���������

#define F_CPU 8000000

#define ADC_BITS 10 /* ���������� ��� */
#define ADC_MAX_VAL ((1 << ADC_BITS) - 1) /* ����������� ��������� ��������� ��� */
#define ADC_HALF_VAL ((1 << (ADC_BITS - 1)) - 1) /* �������� ������� ��� */
#define REFERENCE_VOLTAGE 2.495 /* ������� ���������� */
#define VOLTAGE_DIVIDER 0.5 /* �������� ��� ��������� ���������� */
#define CONTROL_DIVIDER 0.5 /* �������� ��� �������� ��� */

#define WAIT_FOR_ADC {adc_timeout_counter = 0; while (((ADCSRA & (1 << ADSC)) != 0) && (adc_timeout_counter++ < 0xfff0)) asm("nop");} /* ��� ��������� �������������� ��� */
#define ADC_START_CONVERSION ADCSRA |= (1 << ADSC); /* ������ �������������� ��� */

#define MEASUREMENTS_COUNT 9 /* ������� ������ ��������� ���� ��� ��������� �������. �������� ����� ������ 4 � ������ 255 */

#define DOT_POINT 0b10000000

// ---------------------------------------------------------------- ���������-������

const uint8_t chars[] PROGMEM = {
	0b00111111, // 0
	0b00000110, // 1
	0b01011011, // 2
	0b01001111, // 3
	0b01100110, // 4
	0b01101101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01101111, // 9
	0b00000000, // ������
	0b01111001, // E
	0b01010000, // R
	0b01000000  // -
};

// ---------------------------------------------------------------- ���������

// ��������� �������
typedef enum DISPLAY_STATE
{
	DISPLAY_STATE_CURRENT = 0, /* ���������� ��� */
	DISPLAY_STATE_PERCENT = 1, /* ���������� �������� */
	DISPLAY_STATE_ERROR = 2, /* ���������� "Err" */
	DISPLAY_STATE_MEASURING = 3, /* ���������� "---" */
} DISPLAY_STATE_t;

// ----------------------------------------------------------------

// ��������� �������
typedef enum DEVICE_STATE
{
	DEVICE_STATE_STANDBY = 0, /* ���������� �� ���������� ���� � ���������� */
	DEVICE_STATE_PREPARE = 1, /* ���������� � ��������� ���� */
	DEVICE_STATE_MEASURE = 2, /* ��������� ���� */
} DEVICE_STATE_t;

// ---------------------------------------------------------------- ����������

volatile uint16_t brightness_delay; // �������� ��� ����������� �������

volatile uint32_t display_number; // ������-������� �����, ��� ����� ����� �������
volatile uint8_t display_chars[3]; // ������� ��� ������ �� �������
volatile DISPLAY_STATE_t display_state; // ��� ���������� �� �������

volatile uint8_t display_update_state; // ��������� ��������, ������������ �������
uint16_t adc_timeout_counter;

volatile uint16_t adc_result; // ��������� ��������� ���
uint16_t adc_readings_count; // ���������� ����������
uint32_t adc_readings_sum; // ����� ����������

uint8_t measuring_higher_current; /* ����� �� �� �������� ������� ��� */
DEVICE_STATE_t device_state; /* ��������� ������� */
uint8_t standby_measuring_current; /* �������� �� �� ��� � ������ STANDBY */
float current; // ���, �����
float voltage; // ����������, �����
float percent; // ��������
float control; // �������� ���
float measured_currents[2];
uint8_t measured_currents_count;
float prev_current;
float diff_current;
float measurements[MEASUREMENTS_COUNT];
uint8_t measurements_count;

// ---------------------------------------------------------------- eeprom

// ---------------------------------------------------------------- �������������

// ������������� ����������� �������
void init_wdt(void)
{
    asm("wdr");
    WDTCR |= (1 << WDCE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
}

// ----------------------------------------------------------------

// ������������� �����
void init_gpio(void)
{
    DDRB = (1 << PINB0) | (1 << PINB1) | (1 << PINB2); // DIG1 - DIG3

    DDRC = (1 << PINC3); // Percent LEDs
	
    DDRD = 0xff; // All Outputs: SEG_A-SEG_DP
    // TODO: disable ADC at PC3, PC4, PC5
	// TODO: disable unnecessary peripherals
	
    // ��������� PULL-UP ���������
    PORTB = 0;
    PORTC = (1 << PINC4) | (1 << PINC5); // BTN1, BTN2
    PORTD = 0;
}

// ----------------------------------------------------------------

// ������������� �������, ����������� �� ���������� �������
void init_tc1(void)
{
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (1 << FOC1A) | (0 << FOC1B) | (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
	TCNT1 = 0;
	OCR1A = DISPLAY_UPDATE_DELAY;
	OCR1B = 0;
	ICR1 = 0;
	TIMSK = (0 << TICIE1) | (1 << OCIE1A) | (0 << OCIE1B) | (0 << TOIE1);
	//TIFR = 0;
}

// ----------------------------------------------------------------

#define ADC_MUX_CONTROL 0
#define ADC_MUX_CURRENT 1
#define ADC_MUX_VOLTAGE 2

// ������������� ���
void init_adc(const uint8_t input)
{
	ADCSRA = 0; // Disable ADC
	adc_readings_count = 0;
	adc_readings_sum = 0;
	ADMUX = (0 << REFS1) | (0 << REFS0) | (0 << ADLAR) | (input & 0x0f); // REF: AREF, MUX: input
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADFR) | (0 << ADIF) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // DIV128, Interrupt, Auto-trigger
}

#define ADC_DIVISOR 128
#define ADC_MEASURINGS_PER_SEC (F_CPU / ADC_DIVISOR / 13)

// ---------------------------------------------------------------- �������

// �������, ��� �������������� ����� � �������, ������������ �� �������
void display_update(void)
{
	if ((display_number > 99900) || (display_state == DISPLAY_STATE_ERROR))
	{
		display_chars[0] = pgm_read_byte(&chars[11]); // E
		display_chars[1] = pgm_read_byte(&chars[12]); // r
		display_chars[2] = pgm_read_byte(&chars[12]); // r
		return;
	}
	
	if (display_state == DISPLAY_STATE_MEASURING)
	{
		display_chars[0] = pgm_read_byte(&chars[13]); // -
		display_chars[1] = pgm_read_byte(&chars[13]); // -
		display_chars[2] = pgm_read_byte(&chars[13]); // -
		return;
	}
	
	uint32_t display_number_temp;
	display_number_temp = display_number;
	// TODO: ��� ����������� �����������-����� ��� ����������� ������ ����������
	uint8_t tmp_digits[5];
	
	for (uint8_t i=0; i<5; i++)
	{
		tmp_digits[i] = display_number_temp % 10;
		display_number_temp /= 10;
	}
	
	if (display_state == DISPLAY_STATE_PERCENT) // ��������
	{
		if (tmp_digits[4] != 0)
		{
			display_chars[0] = pgm_read_byte(&chars[tmp_digits[4]]);
		} else {
			display_chars[0] = pgm_read_byte(&chars[10]); // ������
		}
		
		if (tmp_digits[3] != 0)
		{
			display_chars[1] = pgm_read_byte(&chars[tmp_digits[3]]);
		} else {
			display_chars[1] = pgm_read_byte(&chars[10]); // ������
		}
		
		display_chars[2] = pgm_read_byte(&chars[tmp_digits[2]])/* | DOT_POINT*/;
		return;
	}
	
	if (tmp_digits[4] != 0) // ����� � ����� ������� �� �������
	{
		display_chars[0] = pgm_read_byte(&chars[tmp_digits[4]]);
		display_chars[1] = pgm_read_byte(&chars[tmp_digits[3]]);
		display_chars[2] = pgm_read_byte(&chars[tmp_digits[2]]) | DOT_POINT;
		return;
	}
	
	if (tmp_digits[3] != 0) // ����� � ����� ������� �� �������
	{
		display_chars[0] = pgm_read_byte(&chars[tmp_digits[3]]);
		display_chars[1] = pgm_read_byte(&chars[tmp_digits[2]]) | DOT_POINT;
		display_chars[2] = pgm_read_byte(&chars[tmp_digits[1]]);
		return;
	}
	
	display_chars[0] = pgm_read_byte(&chars[tmp_digits[2]]) | DOT_POINT;
	display_chars[1] = pgm_read_byte(&chars[tmp_digits[1]]);
	display_chars[2] = pgm_read_byte(&chars[tmp_digits[0]]);
}

// ----------------------------------------------------------------

// ������������� �����
void init_all(void)
{
	init_wdt(); // ������������� ����������� �������
	init_gpio(); // ������������� �����
	init_tc1(); // ������������� �������, ������������ �������
	init_adc(ADC_MUX_CONTROL); // ������� �������� ����������� �����
	sei();
	
	brightness_delay = 0xffff;
	measured_currents_count = 0;
	measuring_higher_current = 0;
	standby_measuring_current = 0;
	brightness_delay = 0xffff;
	prev_current = 0;
	device_state = DEVICE_STATE_PREPARE; // ������� �������� ����������� �����
	display_state = DISPLAY_STATE_MEASURING;
	display_update();
}

// ----------------------------------------------------------------

// ---------------------------------------------------------------- ����������

// ���������� ���, ������ ��� ������� ����
ISR (ADC_vect)
{
	// TODO: ���-���� ����������� ������ ���������, �������� ����������
	adc_readings_sum += ADCW;
	adc_readings_count++;
}

// ----------------------------------------------------------------

// ���������� �������, ������������ �����
ISR (TIMER1_COMPA_vect)
{
	if (display_update_state < 3)
	{
		// � ��������� 0, 1, 2 ���������� �������
		PORTB = 0;
		PORTD = display_chars[display_update_state];
		PORTB = (1 << display_update_state);
		
		if (display_update_state == 0)
		{
			OCR1A = DISPLAY_UPDATE_DELAY;
			if ((display_state == DISPLAY_STATE_PERCENT) && ((adc_readings_count & (1 << 8)) || (display_number == 0))) PORTC |= (1 << PINC3); // ��� ������������� ������ � ������ �������� �������� ���������� "�������"
		} else {
			PORTC &= ~(1 << PINC3); // ��������� ���������� "�������"
		}
	} else {
		// ����� ����������� ���������� ������� ��������� ����� � ���
		PORTB = 0;
		PORTD = 0;
		OCR1A = brightness_delay;
	}
	display_update_state++;
	if (display_update_state > 3)
	{
		display_update_state = 0;
	}
	// TODO: �������� ��� � ����������
}

// ---------------------------------------------------------------- main

int main(void)
{
	init_all();
	
    while (1)
    {
		switch (device_state)
		{
			case DEVICE_STATE_STANDBY: { // ���������� �� ����������� ������� � �����
				if (standby_measuring_current)
				{
					// ���������� �� �����
					if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 3)) // 0.33 ���
					{
						adc_readings_count -= 1; // ������ ��������� ���� ���������
						current = ((float)(adc_readings_sum)) / ((float)(adc_readings_count)) / ((float)(PRE_COEFF)) * control;
						
						init_adc(ADC_MUX_VOLTAGE);
						standby_measuring_current = 0;
					}
				} else {
					// ���������� �� �����������
					if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 3)) // 0.33 ���
					{
						adc_readings_count -= 1; // ������ ��������� ���� ���������
						voltage = (((float)(adc_readings_sum)) / ((float)(adc_readings_count))) / ((float)(ADC_MAX_VAL)) * ((float)(REFERENCE_VOLTAGE)) / ((float)(VOLTAGE_DIVIDER)) * control;
						
						init_adc(ADC_MUX_CURRENT);
						standby_measuring_current = 1;
					}
				}
				
				diff_current = current - prev_current;
				prev_current = current;
				
				if ((current < MINIMUM_CURRENT) || (voltage < MINIMUM_VOLTAGE)) // ���� ��� ������, �� ���� ���������� ��������
				{
					if (display_state != DISPLAY_STATE_PERCENT)
					{
						if (measured_currents_count == 2)
						{
							if (measured_currents[0] > measured_currents[1])
							{
								percent = (measured_currents[0] / measured_currents[1] - 1.0) * 100.0;
							} else {
								percent = (measured_currents[1] / measured_currents[0] - 1.0) * 100.0;
							}
							
							display_number = ((uint32_t)(percent * 100.0 + 0.5));
						} else {
							display_number = 0;
						}
						measured_currents_count = 0;
						
						device_state = DEVICE_STATE_STANDBY;
						display_state = DISPLAY_STATE_PERCENT;
						display_update();
							
					}
					
					// TODO: �������� ����������� �������
					/*
					float brightness = 5.0 / voltage;
					if (brightness > 1.0) brightness = 1.0;
					brightness_delay = ((uint16_t)( ((float)(0xfffe)) * (1.0 / (brightness * brightness)) )); // ������� ��������� �������. ����������� ������� �� 2.0, � 2.4
					if (brightness_delay < DISPLAY_UPDATE_DELAY) brightness_delay = DISPLAY_UPDATE_DELAY;
					if (brightness_delay == 0xffff) brightness_delay = 0xfffe;*/
					brightness_delay = 0xfffe;
				} else {
					brightness_delay = 0xfffe;
				
					if ((diff_current < -CURRENT_CHANGE_SENSITIVITY) || (diff_current > CURRENT_CHANGE_SENSITIVITY)) // ��������� ������ ����
					{
						device_state = DEVICE_STATE_PREPARE; // ���������� � ���������
						init_adc(ADC_MUX_CONTROL);
						display_state = DISPLAY_STATE_MEASURING;
						display_update();
					}
				}
				
			}; break;
			
			case DEVICE_STATE_PREPARE: { // ���������� � ��������� ���� (0.5 �������)
				if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 2)) // 0.5 ���
				{
					adc_readings_count -= 1; // ������ ��������� ���� ���������
					control = ((float)(ADC_HALF_VAL)) / (((float)(adc_readings_sum)) / ((float)(adc_readings_count)));
					
					device_state = DEVICE_STATE_MEASURE;
					display_state = DISPLAY_STATE_MEASURING;
					measurements_count = 0;
					display_update();
					init_adc(ADC_MUX_CURRENT);
				}
			}; break;
			
			case DEVICE_STATE_MEASURE: { // ��������� ���� (1 �������)
				if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 16)) // 0.05 ���
				{
					adc_readings_count -= 1; // ������ ��������� ���� ���������
					current = ((float)(adc_readings_sum)) / ((float)(adc_readings_count)) / ((float)(PRE_COEFF));
					init_adc(ADC_MUX_CURRENT);
					
					if (measurements_count < MEASUREMENTS_COUNT)
					{
						measurements[measurements_count++] = current;
					}
					
					if (measurements_count >= MEASUREMENTS_COUNT)
					{
						// ����������
						for (uint8_t i=0; i<MEASUREMENTS_COUNT - 1; i++)
							for (uint8_t j=i+1; j<MEASUREMENTS_COUNT; j++)
								if (measurements[i] > measurements[j])
								{
									current = measurements[j];
									measurements[j] = measurements[i];
									measurements[i] = current;
									asm("wdr");
								}
						
						// ��������� �������� ���������
						if ((measurements[MEASUREMENTS_COUNT / 2 + 1] / measurements[MEASUREMENTS_COUNT / 2 - 1]) < 1.1) // 5 ��������� ���������� �� ����� ��� �� 10%
						{
							// ���������� �������
							current = measurements[MEASUREMENTS_COUNT / 2];
							
							if (current < MINIMUM_CURRENT)
							{
								device_state = DEVICE_STATE_STANDBY;
								prev_current = current; // ����� ����� �� ������� ������������ ���������
								break;
							}
							
							measured_currents[0] = measured_currents[1];
							measured_currents[1] = current;
							if (measured_currents_count < 2) measured_currents_count++;
							
							display_state = DISPLAY_STATE_CURRENT;
							display_number = ((uint32_t)(current * 100.0 + 0.5));
						} else {
							current = measurements[MEASUREMENTS_COUNT / 2];
							display_number = 0;
							display_state = DISPLAY_STATE_ERROR;
							measured_currents_count = 0; // ��� ������ �������� ��������� ������������ � ������������� ���� �������
						}
						
						prev_current = current; // ����� ����� �� ������� ������������ ���������
						device_state = DEVICE_STATE_STANDBY;
						standby_measuring_current = 0;
						display_update();
						init_adc(ADC_MUX_VOLTAGE);
						
						measurements_count = 0;
					}
				}
			}; break;
		}
		
		asm("wdr");
    }
}

// TODO: �������� ��������. ���������, ��� �������� ������ � ����
// TODO: �������� ������� ����������
// TODO: �������� "+ 0.5" �� ���������� ����������