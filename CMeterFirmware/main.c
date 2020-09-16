#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------- Настройки

#define DISPLAY_UPDATE_DELAY 2000 /* Задержка при обновлении дисплея. Чем больше это число, тем больше будет яркость дисплея и меньше время автономной работы. */
#define PRE_COEFF 38.461 /* Предварительных коэффициент для преобразования показаний АЦП в Амперы. Шунт: 510 Ом */
#define CURRENT_CHANGE_SENSITIVITY 0.1 /* Чувствительность на изменени тока */
#define MINIMUM_CURRENT 0.5 /* Минимально допустимый рабочий ток измеряемых двигателей, Ампер */
#define MINIMUM_VOLTAGE 4.5 /* Минимально допустимое напряжение питания устройства */

// ---------------------------------------------------------------- Технические константы

#define F_CPU 8000000

#define ADC_BITS 10 /* Разрешение АЦП */
#define ADC_MAX_VAL ((1 << ADC_BITS) - 1) /* Максимально возможные показания АЦП */
#define ADC_HALF_VAL ((1 << (ADC_BITS - 1)) - 1) /* Середина размаха АЦП */
#define REFERENCE_VOLTAGE 2.495 /* Опорное напряжение */
#define VOLTAGE_DIVIDER 0.5 /* Делитель для измерения напряжения */
#define CONTROL_DIVIDER 0.5 /* Делитель для контроля АЦП */

#define WAIT_FOR_ADC {adc_timeout_counter = 0; while (((ADCSRA & (1 << ADSC)) != 0) && (adc_timeout_counter++ < 0xfff0)) asm("nop");} /* Ждём окончания преобразования АЦП */
#define ADC_START_CONVERSION ADCSRA |= (1 << ADSC); /* Начать преобразование АЦП */

#define MEASUREMENTS_COUNT 9 /* Сколько делать измерений тока для получения медианы. Нечётное число больше 4 и меньше 255 */

#define DOT_POINT 0b10000000

// ---------------------------------------------------------------- Константы-строки

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
	0b00000000, // Пробел
	0b01111001, // E
	0b01010000, // R
	0b01000000  // -
};

// ---------------------------------------------------------------- Структуры

// Состояние дисплея
typedef enum DISPLAY_STATE
{
	DISPLAY_STATE_CURRENT = 0, /* Отобразить ток */
	DISPLAY_STATE_PERCENT = 1, /* Отобразить проценты */
	DISPLAY_STATE_ERROR = 2, /* Отобразить "Err" */
	DISPLAY_STATE_MEASURING = 3, /* Отобразить "---" */
} DISPLAY_STATE_t;

// ----------------------------------------------------------------

// Состояние прибора
typedef enum DEVICE_STATE
{
	DEVICE_STATE_STANDBY = 0, /* Наблюдение за изменением тока и напряжения */
	DEVICE_STATE_PREPARE = 1, /* Подготовка к измерению тока */
	DEVICE_STATE_MEASURE = 2, /* Измерение тока */
} DEVICE_STATE_t;

// ---------------------------------------------------------------- Переменные

volatile uint16_t brightness_delay; // Задержка для компенсации яркости

volatile uint32_t display_number; // Псевдо-дробное число, две цифры после запятой
volatile uint8_t display_chars[3]; // Символы для вывода на дисплей
volatile DISPLAY_STATE_t display_state; // Что отобразить на дисплее

volatile uint8_t display_update_state; // Состояние автомата, обновляющего дисплей
uint16_t adc_timeout_counter;

volatile uint16_t adc_result; // Результат измерения АЦП
uint16_t adc_readings_count; // Количество считываний
uint32_t adc_readings_sum; // Сумма считываний

uint8_t measuring_higher_current; /* Будем ли мы измерять больший ток */
DEVICE_STATE_t device_state; /* Состояние прибора */
uint8_t standby_measuring_current; /* Измеряем ли мы ток в режиме STANDBY */
float current; // Ток, ампер
float voltage; // Напряжение, вольт
float percent; // Проценты
float control; // Контроль АЦП
float measured_currents[2];
uint8_t measured_currents_count;
float prev_current;
float diff_current;
float measurements[MEASUREMENTS_COUNT];
uint8_t measurements_count;

// ---------------------------------------------------------------- eeprom

// ---------------------------------------------------------------- Инициализация

// Инициализация сторожевого таймера
void init_wdt(void)
{
    asm("wdr");
    WDTCR |= (1 << WDCE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
}

// ----------------------------------------------------------------

// Инициализация пинов
void init_gpio(void)
{
    DDRB = (1 << PINB0) | (1 << PINB1) | (1 << PINB2); // DIG1 - DIG3

    DDRC = (1 << PINC3); // Percent LEDs
	
    DDRD = 0xff; // All Outputs: SEG_A-SEG_DP
    // TODO: disable ADC at PC3, PC4, PC5
	// TODO: disable unnecessary peripherals
	
    // Выключаем PULL-UP резисторы
    PORTB = 0;
    PORTC = (1 << PINC4) | (1 << PINC5); // BTN1, BTN2
    PORTD = 0;
}

// ----------------------------------------------------------------

// Инициализация таймера, отвечающего за обновление дисплея
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

// Инициализация АЦП
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

// ---------------------------------------------------------------- Функции

// Функция, для преобразования числа в символы, отображаемые на дисплее
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
	// TODO: При копировании переменного-числа для отображения делать округление
	uint8_t tmp_digits[5];
	
	for (uint8_t i=0; i<5; i++)
	{
		tmp_digits[i] = display_number_temp % 10;
		display_number_temp /= 10;
	}
	
	if (display_state == DISPLAY_STATE_PERCENT) // Проценты
	{
		if (tmp_digits[4] != 0)
		{
			display_chars[0] = pgm_read_byte(&chars[tmp_digits[4]]);
		} else {
			display_chars[0] = pgm_read_byte(&chars[10]); // Пробел
		}
		
		if (tmp_digits[3] != 0)
		{
			display_chars[1] = pgm_read_byte(&chars[tmp_digits[3]]);
		} else {
			display_chars[1] = pgm_read_byte(&chars[10]); // Пробел
		}
		
		display_chars[2] = pgm_read_byte(&chars[tmp_digits[2]])/* | DOT_POINT*/;
		return;
	}
	
	if (tmp_digits[4] != 0) // Число с тремя знаками до запятой
	{
		display_chars[0] = pgm_read_byte(&chars[tmp_digits[4]]);
		display_chars[1] = pgm_read_byte(&chars[tmp_digits[3]]);
		display_chars[2] = pgm_read_byte(&chars[tmp_digits[2]]) | DOT_POINT;
		return;
	}
	
	if (tmp_digits[3] != 0) // Число с двумя знаками до запятой
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

// Инициализация всего
void init_all(void)
{
	init_wdt(); // Инициализация сторожевого таймера
	init_gpio(); // Инициализация пинов
	init_tc1(); // Инициализация таймера, обновляющего дисплей
	init_adc(ADC_MUX_CONTROL); // Сначала измеряем контрольную точку
	sei();
	
	brightness_delay = 0xffff;
	measured_currents_count = 0;
	measuring_higher_current = 0;
	standby_measuring_current = 0;
	brightness_delay = 0xffff;
	prev_current = 0;
	device_state = DEVICE_STATE_PREPARE; // Сначала измеряем контрольную точку
	display_state = DISPLAY_STATE_MEASURING;
	display_update();
}

// ----------------------------------------------------------------

// ---------------------------------------------------------------- Прерывания

// Прерывание АЦП, только для датчика тока
ISR (ADC_vect)
{
	// TODO: все-таки выбрасывать первое измерение, точность увеличится
	adc_readings_sum += ADCW;
	adc_readings_count++;
}

// ----------------------------------------------------------------

// Прерывание таймера, обновляющего экран
ISR (TIMER1_COMPA_vect)
{
	if (display_update_state < 3)
	{
		// В состоянии 0, 1, 2 показываем символы
		PORTB = 0;
		PORTD = display_chars[display_update_state];
		PORTB = (1 << display_update_state);
		
		if (display_update_state == 0)
		{
			OCR1A = DISPLAY_UPDATE_DELAY;
			if ((display_state == DISPLAY_STATE_PERCENT) && ((adc_readings_count & (1 << 8)) || (display_number == 0))) PORTC |= (1 << PINC3); // При необходимости вместе с первым символом включаем светодиоды "процент"
		} else {
			PORTC &= ~(1 << PINC3); // Выключаем светодиоды "процент"
		}
	} else {
		// После отображения последнего символа выключаем табло и ждём
		PORTB = 0;
		PORTD = 0;
		OCR1A = brightness_delay;
	}
	display_update_state++;
	if (display_update_state > 3)
	{
		display_update_state = 0;
	}
	// TODO: ускорить код в прерывании
}

// ---------------------------------------------------------------- main

int main(void)
{
	init_all();
	
    while (1)
    {
		switch (device_state)
		{
			case DEVICE_STATE_STANDBY: { // Наблюдение за напряжением питания и током
				if (standby_measuring_current)
				{
					// Наблюдение за током
					if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 3)) // 0.33 сек
					{
						adc_readings_count -= 1; // Первое измерение было выброшено
						current = ((float)(adc_readings_sum)) / ((float)(adc_readings_count)) / ((float)(PRE_COEFF)) * control;
						
						init_adc(ADC_MUX_VOLTAGE);
						standby_measuring_current = 0;
					}
				} else {
					// Наблюдение за напряжением
					if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 3)) // 0.33 сек
					{
						adc_readings_count -= 1; // Первое измерение было выброшено
						voltage = (((float)(adc_readings_sum)) / ((float)(adc_readings_count))) / ((float)(ADC_MAX_VAL)) * ((float)(REFERENCE_VOLTAGE)) / ((float)(VOLTAGE_DIVIDER)) * control;
						
						init_adc(ADC_MUX_CURRENT);
						standby_measuring_current = 1;
					}
				}
				
				diff_current = current - prev_current;
				prev_current = current;
				
				if ((current < MINIMUM_CURRENT) || (voltage < MINIMUM_VOLTAGE)) // Если ток пропал, то пора показывать проценты
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
					
					// TODO: починить компенсацию яркости
					/*
					float brightness = 5.0 / voltage;
					if (brightness > 1.0) brightness = 1.0;
					brightness_delay = ((uint16_t)( ((float)(0xfffe)) * (1.0 / (brightness * brightness)) )); // Простая коррекция яркости. Оптимальная степень не 2.0, а 2.4
					if (brightness_delay < DISPLAY_UPDATE_DELAY) brightness_delay = DISPLAY_UPDATE_DELAY;
					if (brightness_delay == 0xffff) brightness_delay = 0xfffe;*/
					brightness_delay = 0xfffe;
				} else {
					brightness_delay = 0xfffe;
				
					if ((diff_current < -CURRENT_CHANGE_SENSITIVITY) || (diff_current > CURRENT_CHANGE_SENSITIVITY)) // Произошел скачок тока
					{
						device_state = DEVICE_STATE_PREPARE; // Подготовка к измерению
						init_adc(ADC_MUX_CONTROL);
						display_state = DISPLAY_STATE_MEASURING;
						display_update();
					}
				}
				
			}; break;
			
			case DEVICE_STATE_PREPARE: { // Подготовка к измерению тока (0.5 секунды)
				if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 2)) // 0.5 сек
				{
					adc_readings_count -= 1; // Первое измерение было выброшено
					control = ((float)(ADC_HALF_VAL)) / (((float)(adc_readings_sum)) / ((float)(adc_readings_count)));
					
					device_state = DEVICE_STATE_MEASURE;
					display_state = DISPLAY_STATE_MEASURING;
					measurements_count = 0;
					display_update();
					init_adc(ADC_MUX_CURRENT);
				}
			}; break;
			
			case DEVICE_STATE_MEASURE: { // Измерение тока (1 секунда)
				if (adc_readings_count > (ADC_MEASURINGS_PER_SEC / 16)) // 0.05 сек
				{
					adc_readings_count -= 1; // Первое измерение было выброшено
					current = ((float)(adc_readings_sum)) / ((float)(adc_readings_count)) / ((float)(PRE_COEFF));
					init_adc(ADC_MUX_CURRENT);
					
					if (measurements_count < MEASUREMENTS_COUNT)
					{
						measurements[measurements_count++] = current;
					}
					
					if (measurements_count >= MEASUREMENTS_COUNT)
					{
						// Сортировка
						for (uint8_t i=0; i<MEASUREMENTS_COUNT - 1; i++)
							for (uint8_t j=i+1; j<MEASUREMENTS_COUNT; j++)
								if (measurements[i] > measurements[j])
								{
									current = measurements[j];
									measurements[j] = measurements[i];
									measurements[i] = current;
									asm("wdr");
								}
						
						// Оцениваем качество измерений
						if ((measurements[MEASUREMENTS_COUNT / 2 + 1] / measurements[MEASUREMENTS_COUNT / 2 - 1]) < 1.1) // 5 измерений отличаются не более чем на 10%
						{
							// Извлечение медианы
							current = measurements[MEASUREMENTS_COUNT / 2];
							
							if (current < MINIMUM_CURRENT)
							{
								device_state = DEVICE_STATE_STANDBY;
								prev_current = current; // Чтобы сразу не вызвать срабатывание измерения
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
							measured_currents_count = 0; // При ошибке начинаем измерение минимального и максимального тока сначала
						}
						
						prev_current = current; // Чтобы сразу не вызвать срабатывание измерения
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

// TODO: точность страдает. убедиться, что виноваты помехи в сети
// TODO: добавить функцию калибровки
// TODO: заменить "+ 0.5" на нормальное округление