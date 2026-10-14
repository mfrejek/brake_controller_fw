/*
 * brake_controller.c
 *
 * Created: 31/08/2026 14:08:35
 * Author : user
 */ 
#define F_CPU	10000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <stdio.h>
#include "stdbool.h"

// avrdude -c usbasp -p m8 -v -V -U flash:w:D:\dev\analog\brake_controller_fw\brake_controller\brake_controller\Release\brake_controller.hex
//COM7

#define LED1_PORT	PORTD
#define LED1_DDR		DDRD
#define LED1_IO		PD2
#define LED2_PORT	PORTD
#define LED2_DDR		DDRD
#define LED2_IO		PD3
#define LED3_PORT	PORTD
#define LED3_DDR		DDRD
#define LED3_IO		PD4
#define LED4_PORT	PORTD
#define LED4_DDR		DDRD
#define LED4_IO		PD7
#define LED5_PORT	PORTB
#define LED5_DDR		DDRB
#define LED5_IO		PB0
#define LED6_PORT	PORTB
#define LED6_DDR		DDRB
#define LED6_IO		PB4

#define PWM1_PORT	PORTD
#define PWM1_DDR		DDRD
#define PWM1_IO		PD6
#define PWM2_PORT	PORTD
#define PWM2_DDR		DDRD
#define PWM2_IO		PD5
#define PWM3_PORT	PORTB
#define PWM3_DDR		DDRB
#define PWM3_IO		PB1
#define PWM4_PORT	PORTB
#define PWM4_DDR		DDRB
#define PWM4_IO		PB2

#define PWM_CNT_TOP	10000U

#define PWM_START_CH1	0
#define PWM_START_CH2	(PWM_CNT_TOP / 4)
#define PWM_START_CH3	PWM_START_CH2 * 2
#define PWM_START_CH4	PWM_START_CH2 * 3 

#define LED_1_MIN_PW (uint16_t)1
#define LED_2_MIN_PW (uint16_t)(PWM_CNT_TOP / 5)
#define LED_3_MIN_PW (uint16_t)((PWM_CNT_TOP / 5) * 2)
#define LED_4_MIN_PW (uint16_t)((PWM_CNT_TOP / 5) * 3)
#define LED_5_MIN_PW (uint16_t)((PWM_CNT_TOP / 5) * 4)
#define LED_6_MIN_PW (uint16_t)((PWM_CNT_TOP / 10) * 9)

#define BRAKE_VOLTAGE_START_2MV	(20000 / 2)	//Start braking at this DC voltage
#define BRAKE_VOLTAGE_END_2MV	(30000 / 2)	//Start braking at this DC voltage
#define VSENSE_DIV_RATIO			45		//Voltage divider ratio; 660k over 15k
#define VBUS_MV_PER TICK			(2.56f * VSENSE_DIV_RATIO) / 1024.0f;

#define BAUD 125000UL      // Baudrate

// Berechnungen
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   // clever runden
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD) // Fehler in Promille, 1000 = kein Fehler.

#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
#error Systematischer Fehler der Baudrate grösser 1% und damit zu hoch!
#endif

void set_brake_pw(uint16_t pulse_width);
void led_show_percentage(uint16_t pw);
volatile uint32_t print_val = 42;
unsigned char outstr[80];
volatile uint16_t dbg_var[10];
volatile uint16_t last_timer_cnt = 0;


typedef enum {
	FET_1_ON,
	FET_2_ON,
	FET_3_ON,
	FET_4_ON,
	FET_1_OFF,
	FET_2_OFF,
	FET_3_OFF,
	FET_4_OFF
} PwmAction_t;

typedef struct
{
	uint16_t on_ticks;
	uint16_t off_ticks;
} PwmChannel_t;
volatile PwmChannel_t pwm_channels[4];


volatile uint16_t adc_res = 0;
volatile uint16_t next_compare = 0;
volatile uint16_t new_pulse_width = 0;

void tmp_delay(uint32_t ticks)
{
	volatile uint32_t i = 0;
	while(i < 1000)
	{
		wdt_reset();
		i++;
	}
}

/* ATmega16 */
int uart_putc(unsigned char c)
{
	while (!(UCSRA & (1<<UDRE)))  /* warten bis Senden moeglich */
	{
		wdt_reset();
	}

	UDR = c;                      /* sende Zeichen */
	return 0;
}


/* puts ist unabhaengig vom Controllertyp */
void uart_puts (char *s)
{
	while (*s)
	{   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
		uart_putc(*s);
		s++;
	}
}

void uart_init(void)
{
	UBRRH = UBRR_VAL >> 8;
	UBRRL = UBRR_VAL & 0xFF;

	UCSRB |= (1<<TXEN);  // UART TX einschalten
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // Asynchron 8N1
}

inline void adc_set_channel(uint8_t channel)
{
	ADMUX = channel | (1<<REFS1) | (1<<REFS0);
}

void adc_schedule_next_measurement(void)
{
	static uint8_t chan_cnt = 1;
	if(chan_cnt & 1)
	{
		//Odd count - sample DC Bus voltage
		adc_set_channel(0); //Ch0 = DC voltage
	}
	else
	{
		//Even count - sample a channel current or 48V Input voltage
		adc_set_channel(chan_cnt >> 1);
	}
	chan_cnt++;
	if(chan_cnt > 8)
	{
		chan_cnt = 1; //chan_cnt goes from 1...8
	}
}

int main(void)
{
    /* Replace with your application code */
	uint16_t brake_pw = 1;
	LED1_DDR |= (1<<LED1_IO);
	LED2_DDR |= (1<<LED2_IO);
	LED3_DDR |= (1<<LED3_IO);
	LED4_DDR |= (1<<LED4_IO);
	LED5_DDR |= (1<<LED5_IO);
	LED6_DDR |= (1<<LED6_IO);
	PWM1_DDR |= (1<<PWM1_IO);
	PWM2_DDR |= (1<<PWM2_IO);
	PWM3_DDR |= (1<<PWM3_IO);
	PWM4_DDR |= (1<<PWM4_IO);
	//Preset 
	set_brake_pw(1);
	//Enable TIM0
	TCNT1 = 0;
	OCR1A = 0;
	OCR1B = 0;
	//No pulse-free interval
	//Suspicion: Only 1 PWM pulse per timer cycle. OCR1A/1B resetting may not work?
	ICR1 = PWM_CNT_TOP;	//Set timer top
	TCCR1A = 0;// | (1<<WGM11);
	TCCR1B = (1<<WGM12) | (1<<WGM13);
	TIMSK = (1<<OCIE1A) | (1<<OCIE1B); //| (1<<TOIE1); ; no overflow in CTC mode
	//Enable ADC
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); //Prescaler 64
	adc_res = ADCW;
	//Enable UART
	uart_init();
	sei();
	uart_puts("Hello World!\r\n");
	
	ADCSRA |= (1<<ADSC); //Start Conversion
	led_show_percentage(3000);
	uint32_t loop_cnt = 0;
    while (1) 
    {
		//new_pulse_width = brake_pw;
		set_brake_pw(brake_pw);
		TCCR1B |= (1<<CS11); //This starts the timer
		led_show_percentage(brake_pw);
		brake_pw += 10;
		if(brake_pw >=PWM_CNT_TOP)
		{
			brake_pw = 1;
			loop_cnt++;
			//sprintf(outstr, "Cnt: %i\r\n", loop_cnt);
			uart_puts(outstr);
		}
		tmp_delay(40);
/*
		LED1_PORT ^= (1<<LED1_IO);
		tmp_delay(20);
		LED2_PORT ^= (1<<LED2_IO);
		tmp_delay(20);
		LED3_PORT ^= (1<<LED3_IO);
		tmp_delay(20);
		LED4_PORT ^= (1<<LED4_IO);
		tmp_delay(20);
		LED5_PORT ^= (1<<LED5_IO);
		tmp_delay(20);
		LED6_PORT ^= (1<<LED6_IO);
		tmp_delay(20);
		*/
    }
}

void led_show_percentage(uint16_t pw)
{
	if(pw >= LED_1_MIN_PW)
	{
		LED1_PORT |= (1<<LED1_IO);
	}
	else if(pw < LED_1_MIN_PW)
	{
		LED1_PORT &= ~(1<<LED1_IO);
	}
	if(pw >= LED_2_MIN_PW)
	{
		LED2_PORT |= (1<<LED2_IO);
	}
	else if(pw < LED_2_MIN_PW)
	{
		LED2_PORT &= ~(1<<LED2_IO);
	}
	if(pw >= LED_3_MIN_PW)
	{
		LED3_PORT |= (1<<LED3_IO);
	}
	else if(pw < LED_3_MIN_PW)
	{
		LED3_PORT &= ~(1<<LED3_IO);
	}
	if(pw >= LED_4_MIN_PW)
	{
		LED4_PORT |= (1<<LED4_IO);
	}
	else if(pw < LED_4_MIN_PW)
	{
		LED4_PORT &= ~(1<<LED4_IO);
	}
	if(pw >= LED_5_MIN_PW)
	{
		LED5_PORT |= (1<<LED5_IO);
	}
	else if(pw < LED_5_MIN_PW)
	{
		LED5_PORT &= ~(1<<LED5_IO);
	}
	//Test only. LED6 should be handled seperately
	if(pw >= LED_6_MIN_PW)
	{
		LED6_PORT |= (1<<LED6_IO);
	}
	else if(pw < LED_6_MIN_PW)
	{
		LED6_PORT &= ~(1<<LED6_IO);
	}
}
/*
ISR (TIMER1_OVF_vect)
{
	
}
*/

void set_brake_pw(uint16_t pulse_width)
{
	static uint16_t val_cntx = 0;
	//led_show_percentage(pulse_width);
	if(pulse_width == 0)
	{
		//Edge case: If PW = 0, set on_ticks to UINT16_MAX to surely never reach it, for the PWM to stay always off
		for(uint8_t i=0;i<4;i++)
		{
			pwm_channels[i].on_ticks = 0;
			pwm_channels[i].off_ticks = 0;
		}
	}
	else 
	{
		/*
		uint16_t pwm_chan_start = 0;
		for(uint16_t i=0;i<4;i++)
		{
			pwm_channels[i].on_ticks = pwm_chan_start;
			pwm_channels[i].off_ticks = pwm_chan_start + pulse_width;
			pwm_chan_start += PWM_START_CH2;
			//Wraparound for counter top
			if(pwm_channels[i].off_ticks >= PWM_CNT_TOP)
			{
				pwm_channels[i].off_ticks -= PWM_CNT_TOP;
			}
		}
		*/
		/*
		2000 = 2200 = 2300 = OK
		2499 = 2490 = 2400 = Bad after 2 LED cycles
		2350 = Bad in first cycle 4H overrflows to 50
		2333 = Lim to stay below 10000, but keeps working at e.g. 2334 as well
		Doesn't like on_ticks = 0
		Overflow doesn't work. Pulse ends when counter at max.
		Freq 200Hz below 5000 + val_add, 1000Hz above?? 944Hz at 4999
		246Hz at 4900
		PROBLEM: Too low PW (below about 600) skips turn off. Turn off only happens on next channel turn on!
		*/
		val_cntx += 10;
		val_cntx %= PWM_CNT_TOP;
		uint16_t val_add = val_cntx;
		for(uint16_t i=0;i<4;i++)
		{
			val_add += 2500;
			uint16_t tmp_on_ticks = 1000; //0 is not OK
			uint16_t tmp_off_ticks = val_add;	 //2500 = perfect, 3500 also, 4999 also, 5000 always High
			//if(i==1) val_add += 500; //With val_add=0, all switch at same time as expected. val_add >0 makes only first channel switch, others stay Low
			//Wraparound for counter top
			tmp_on_ticks %= PWM_CNT_TOP;
			tmp_off_ticks %= PWM_CNT_TOP;
			//Write produces lots of glitching, tbd
			//pwm_channels[i].on_ticks = tmp_on_ticks;
			//pwm_channels[i].off_ticks = tmp_off_ticks;
		}
	}
	//TEST
	pwm_channels[0].on_ticks = 1000;
	pwm_channels[0].off_ticks = 2000;
	pwm_channels[1].on_ticks = 1500;
	pwm_channels[1].off_ticks = 2500;
	pwm_channels[2].on_ticks = 2000;
	pwm_channels[2].off_ticks = 3000;
	pwm_channels[3].on_ticks = 2500;
	pwm_channels[3].off_ticks = 3500;
	//OCR1A somehow set to 10500?
	print_val++;
	//sprintf(outstr, "0D%i,1D%i,2D%i,3D%i,4D%i,5D%i,6D%i,7D%i,8D%i,9D%i\r\n", dbg_var[0],dbg_var[1],dbg_var[2],dbg_var[3],dbg_var[4],dbg_var[5],dbg_var[6],dbg_var[7],dbg_var[8],dbg_var[9]);
	//sprintf(outstr, "1L%i,1H%i,2L%i,2H%i,3L%i,3H%i,4L%i,4H%i,1A%i,TC%i,N%i\r\n", pwm_channels[0].on_ticks,pwm_channels[0].off_ticks,pwm_channels[1].on_ticks,pwm_channels[1].off_ticks,pwm_channels[2].on_ticks,pwm_channels[2].off_ticks,pwm_channels[3].on_ticks,pwm_channels[3].off_ticks,OCR1A,TCNT1,print_val);
}

/*
Expected with if(i==1) val_add += 2500;
Ch0 and Ch1: H at 1, L at 5000
Ch2 and Ch3: H at 2501, L at 7500
Matches UART Dbg!
Actual Scope pins:
All go H at the same moment
0 and 1 H for ~15% PW, then L for ~25% PW, then H for ~40% PW, then L for ~20% PW
2 and 3 H for ~80% PW, then L for ~20% PW. L synchronous with last L of 0 and 1
*/

void set_next_compare_old(uint16_t current_compare)
{
	//FIXME - this seems to set invalid compares and ISR doesn't trigger anymore
	//BUG: on_ticks > off_ticks gives only minimal pulse
	/*Example for 50% PW and 10000 Top:
		pwm_channels[0].on_ticks = 0
		pwm_channels[0].off_ticks = 5000
		pwm_channels[1].on_ticks = 2500
		pwm_channels[1].off_ticks = 7500
		pwm_channels[2].on_ticks = 5000
		pwm_channels[2].off_ticks = 0
		pwm_channels[3].on_ticks = 7500
		pwm_channels[3].off_ticks = 2500
		current_compare e.g. 5000
	*/
	uint16_t compare_dist = UINT16_MAX;
	int8_t next_on_idx = -1;
	int8_t next_off_idx = -1;
	//Find next switching action in on/off arrays
	for(uint8_t i=0;i<4;i++)
	{
		//Check On ticks
		int32_t current_dist = pwm_channels[i].on_ticks - current_compare;
		if(current_dist <= 0)
		{
			current_dist += PWM_CNT_TOP;
		}
		if(current_dist < compare_dist)
		{
			compare_dist = current_dist;
			next_on_idx = i;
			next_off_idx = -1;
		}
		//Check Off ticks
		current_dist = pwm_channels[i].off_ticks - current_compare;
		if(current_dist <= 0)
		{
			current_dist += PWM_CNT_TOP;
		}
		if(current_dist < compare_dist)
		{
			compare_dist = current_dist;
			next_off_idx = i;
			next_on_idx = -1;
		}
	}
	if(next_on_idx >=0)
	{
		OCR1A = pwm_channels[next_on_idx].on_ticks;
	}
	if(next_off_idx >=0)
	{
		OCR1B = pwm_channels[next_off_idx].off_ticks;
	}
	//led_show_percentage(7999);
}
//UNUSED
void dddset_next_compare_dbg(uint16_t current_compare)
{
	//FIXME - this seems to set invalid compares and ISR doesn't trigger anymore
	/*Example for 50% PW and 10000 Top:
		pwm_channels[0].on_ticks = 0
		pwm_channels[0].off_ticks = 5000
		pwm_channels[1].on_ticks = 2500
		pwm_channels[1].off_ticks = 7500
		pwm_channels[2].on_ticks = 5000
		pwm_channels[2].off_ticks = 0
		pwm_channels[3].on_ticks = 7500
		pwm_channels[3].off_ticks = 2500
		current_compare e.g. 5000
	*/
	uint16_t compare_dist = UINT16_MAX;
	int8_t next_on_idx = -1;
	int8_t next_off_idx = -1;
	static volatile uint16_t last_pulse_width = 0;
	if(new_pulse_width != last_pulse_width)
	{
		last_pulse_width = new_pulse_width;
		//set_brake_pw(new_pulse_width);
	}
	//Find next switching action in on/off arrays
	for(uint8_t i=0;i<4;i++)
	{
		//Check On ticks
		int32_t current_dist = (int32_t)pwm_channels[i].on_ticks - (int32_t)current_compare;
		if(current_dist < 0)
		{
			current_dist += PWM_CNT_TOP;
		}
		if(current_dist < compare_dist)
		{
			compare_dist = current_dist;
			next_on_idx = i;
		}
		//Check Off ticks
		current_dist = (int32_t)pwm_channels[i].off_ticks - (int32_t)current_compare;
		if(current_dist < 0)
		{
			current_dist += PWM_CNT_TOP;
		}
		if(current_dist < compare_dist)
		{
			compare_dist = current_dist;
			next_off_idx = i;
		}
	}
	if(next_off_idx >=0)
	{
		OCR1B = pwm_channels[next_off_idx].off_ticks;
	}
	if(next_on_idx >=0)
	{
		OCR1A = pwm_channels[next_on_idx].on_ticks;
	}
	//led_show_percentage(7999);
}

void set_next_compare_on(uint16_t current_compare)
{
	//FIXME - this seems to set invalid compares and ISR doesn't trigger anymore
	/*Example for 50% PW and 10000 Top:
		pwm_channels[0].on_ticks = 0
		pwm_channels[0].off_ticks = 5000
		pwm_channels[1].on_ticks = 2500
		pwm_channels[1].off_ticks = 7500
		pwm_channels[2].on_ticks = 5000
		pwm_channels[2].off_ticks = 0
		pwm_channels[3].on_ticks = 7500
		pwm_channels[3].off_ticks = 2500
		current_compare e.g. 5000
	*/
	uint16_t compare_dist = UINT16_MAX;
	uint8_t next_on_idx = 0;
	//Find next switching action in on/off arrays
	for(uint8_t i=0;i<4;i++)
	{
		//Check On ticks
		int32_t current_dist = (int32_t)pwm_channels[i].on_ticks - (int32_t)current_compare;
		while(current_dist < 0)
		{
			current_dist += PWM_CNT_TOP;
		}
		if(current_dist < compare_dist)
		{
			compare_dist = current_dist;
			next_on_idx = i;
		}
	}
	static uint8_t tmp_next_on = 0;
	OCR1A = pwm_channels[tmp_next_on].on_ticks; //next_on_idx
	tmp_next_on++;
	if(tmp_next_on == 4)
	{
		tmp_next_on = 0;
	}
	dbg_var[1] = next_on_idx;
	dbg_var[2] = pwm_channels[next_on_idx].on_ticks;
}

void set_next_compare_off(uint16_t current_compare)
{
	//FIXME - this seems to set invalid compares and ISR doesn't trigger anymore
	/*Example for 50% PW and 10000 Top:
		pwm_channels[0].on_ticks = 0
		pwm_channels[0].off_ticks = 5000
		pwm_channels[1].on_ticks = 2500
		pwm_channels[1].off_ticks = 7500
		pwm_channels[2].on_ticks = 5000
		pwm_channels[2].off_ticks = 0
		pwm_channels[3].on_ticks = 7500
		pwm_channels[3].off_ticks = 2500
		current_compare e.g. 5000
	*/
	uint16_t compare_dist = UINT16_MAX;
	uint8_t next_off_idx = 0;
	//Find next switching action in on/off arrays
	for(uint8_t i=0;i<4;i++)
	{
		//Check Off ticks
		int32_t current_dist = (int32_t)pwm_channels[i].off_ticks - (int32_t)current_compare;
		while(current_dist < 0)
		{
			current_dist += PWM_CNT_TOP;
		}
		if(current_dist < compare_dist)
		{
			compare_dist = current_dist;
			next_off_idx = i;
		}
	}
	static uint8_t tmp_next_off = 0;
	OCR1B = pwm_channels[tmp_next_off].off_ticks; //next_off_idx
	tmp_next_off++;
	if(tmp_next_off == 4)
	{
		tmp_next_off = 0;
	}
	//OCR1B = pwm_channels[next_off_idx].off_ticks;
	dbg_var[3] = next_off_idx;
	dbg_var[4] = pwm_channels[next_off_idx].off_ticks;
	//led_show_percentage(7999);
}

volatile uint16_t ovf_cnt = 0;

/*
bool switch_has_passed(uint16_t switch_ticks, uint16_t new_ticks, uint16_t last_ticks)
{
	uint32_t new_ticks_comp = new_ticks;
	if(last_ticks > new_ticks)
	{
		//Overflow has happened
		//E.g. last_ticks=65000, new_ticks=500, switch_ticks=65500
		//Add UINT16_MAX to new_ticks_comp
		new_ticks_comp += PWM_CNT_TOP;
	}
	if(new_ticks >= switch_ticks)
	{
		return true;
	}
	//Check if switch_ticks is inbetween both
	if((last_ticks < switch_ticks) && (new_ticks_comp > switch_ticks))
	{
		return true;
	}
	return false;
}
*/

//Reason for spurious on/off switching is likely here
/*
bool get_onoff_state(uint16_t on_ticks, uint16_t off_ticks, uint16_t new_ticks, uint16_t last_ticks)
{
	uint16_t ticks_since_on = 0;
	uint16_t ticks_since_off = 0;
	//Check if On or Off is shorter ago
	ticks_since_on = new_ticks - on_ticks;
	if(on_ticks > new_ticks)
	{
		ticks_since_on += PWM_CNT_TOP;
	}
	ticks_since_off = new_ticks - off_ticks;
	if(off_ticks > new_ticks)
	{
		ticks_since_off += PWM_CNT_TOP;
	}
	return (ticks_since_on < ticks_since_off);
}*/

/*
Example for overflow:
new_ticks = 300
on_ticks = 500
off_ticks = 8000
Expectation: Be off
ticks_since_on = 300 - 500 = -200 = 65336
+= PWM_CNT_TOP = 75336 = 9800
ticks_since_off = 8000 - 500 = 7500 = 65336
+= PWM_CNT_TOP = 75336 = 9800
*/
/*
bool get_onoff_state(uint16_t on_ticks, uint16_t off_ticks, uint16_t new_ticks, uint16_t last_ticks)
{
	uint16_t ticks_since_on = 0;
	uint16_t ticks_since_off = 0;
	//Check if On or Off is shorter ago
	ticks_since_on = new_ticks - on_ticks;
	if(on_ticks > new_ticks)
	{
		ticks_since_on += PWM_CNT_TOP;
	}
	ticks_since_off = new_ticks - off_ticks;
	if(off_ticks > new_ticks)
	{
		ticks_since_off += PWM_CNT_TOP;
	}
	return (ticks_since_on < ticks_since_off);
}
*/

//This also gives 2 switching at same moment
//Likely issue: Timer ISR cfg
bool get_onoff_state(uint8_t dbg_chan, uint16_t on_ticks, uint16_t off_ticks, uint16_t new_ticks, uint16_t last_ticks)
{
	switch(dbg_chan)
	{
		case 0:
			if((new_ticks >= 1000) && (new_ticks < 2000))
			{
				return true;
			}
			break;
		case 1:
			if((new_ticks >= 1500) && (new_ticks < 2500))
			{
				return true;
			}
			break;
		case 2:
			if((new_ticks >= 2000) && (new_ticks < 3000))
			{
				return true;
			}
			break;
		case 3:
			if((new_ticks >= 2500) && (new_ticks < 3500))
			{
				return true;
			}
		break;
	}
	return false;
}

ISR (TIMER1_COMPA_vect)
{
	//sei();
	uint16_t timer_cnt = TCNT1; //For consistency during ISR
	//led_show_percentage(TCNT1);
	/*
	led_show_percentage(ovf_cnt);
	ovf_cnt++;
	if(ovf_cnt >=PWM_CNT_TOP)
	{
		ovf_cnt = 0;
	}
	uint8_t pwm_stg = ovf_cnt % 4;
	switch(pwm_stg)
	{
		case 0:
			PWM1_PORT |= (1<<PWM1_IO);
			PWM2_PORT &= ~(1<<PWM2_IO);
			PWM3_PORT &= ~(1<<PWM3_IO);
			PWM4_PORT &= ~(1<<PWM4_IO);
			break;
		case 1:
			PWM1_PORT &= ~(1<<PWM1_IO);
			PWM2_PORT |= (1<<PWM2_IO);
			PWM3_PORT &= ~(1<<PWM3_IO);
			PWM4_PORT &= ~(1<<PWM4_IO);
			break;
		case 2:
			PWM1_PORT &= ~(1<<PWM1_IO);
			PWM2_PORT &= ~(1<<PWM2_IO);
			PWM3_PORT |= (1<<PWM3_IO);
			PWM4_PORT &= ~(1<<PWM4_IO);
			break;
		case 3:
			PWM1_PORT &= ~(1<<PWM1_IO);
			PWM2_PORT &= ~(1<<PWM2_IO);
			PWM3_PORT &= ~(1<<PWM3_IO);
			PWM4_PORT |= (1<<PWM4_IO);
			break;
	}
	*/
	
	if(get_onoff_state(0, pwm_channels[0].on_ticks, pwm_channels[0].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM1_PORT |= (1<<PWM1_IO);
	}
	if(get_onoff_state(1, pwm_channels[1].on_ticks, pwm_channels[1].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM2_PORT |= (1<<PWM2_IO);
	}
	if(get_onoff_state(2, pwm_channels[2].on_ticks, pwm_channels[2].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM3_PORT |= (1<<PWM3_IO);
	}
	if(get_onoff_state(3, pwm_channels[3].on_ticks, pwm_channels[3].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM4_PORT |= (1<<PWM4_IO);
	}
	/*
	if(timer_cnt == pwm_channels[0].on_ticks)
	{
		PWM1_PORT |= (1<<PWM1_IO);
	}
	if(timer_cnt == pwm_channels[0].off_ticks)
	{
		PWM1_PORT &= ~(1<<PWM1_IO);
	}
	if(timer_cnt == pwm_channels[1].on_ticks)
	{
		PWM2_PORT |= (1<<PWM2_IO);
	}
	if(timer_cnt == pwm_channels[1].off_ticks)
	{
		PWM2_PORT &= ~(1<<PWM2_IO);
	}
	if(timer_cnt == pwm_channels[2].on_ticks)
	{
		PWM3_PORT |= (1<<PWM3_IO);
	}
	if(timer_cnt == pwm_channels[2].off_ticks)
	{
		PWM3_PORT &= ~(1<<PWM3_IO);
	}
	if(timer_cnt == pwm_channels[3].on_ticks)
	{
		PWM4_PORT |= (1<<PWM4_IO);
	}
	if(timer_cnt == pwm_channels[3].off_ticks)
	{
		PWM4_PORT &= ~(1<<PWM4_IO);
	}
	*/
	//This seems to mess up the next compare
	set_next_compare_on(timer_cnt);
	last_timer_cnt = timer_cnt;
}

ISR (TIMER1_COMPB_vect)
{
	uint16_t timer_cnt = TCNT1; //For consistency during ISR
	if(!get_onoff_state(0, pwm_channels[0].on_ticks, pwm_channels[0].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM1_PORT &= ~(1<<PWM1_IO);
	}
	if(!get_onoff_state(1, pwm_channels[1].on_ticks, pwm_channels[1].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM2_PORT &= ~(1<<PWM2_IO);
	}
	if(!get_onoff_state(2, pwm_channels[2].on_ticks, pwm_channels[2].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM3_PORT &= ~(1<<PWM3_IO);
	}
	if(!get_onoff_state(3, pwm_channels[3].on_ticks, pwm_channels[3].off_ticks, timer_cnt, last_timer_cnt))
	{
		PWM4_PORT &= ~(1<<PWM4_IO);
	}
	set_next_compare_off(timer_cnt);
	last_timer_cnt = timer_cnt;
}