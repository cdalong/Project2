#include "os.h"
//
// LAB - TEST 4
//	Noah Spriggs, Murray Dunne, Daniel McIlvaney
//
// EXPECTED RUNNING ORDER: P0,P1,P2,P3,TIMER,P0,P1,P2
//
// P0     wait 3            signal 2, signal 1
// P1     wait 1
// P2     wait 2
// P3            timer->signal 3

MUTEX mut;
EVENT evt1;
EVENT evt2;
EVENT evt3;

void Task_P0(int parameter)
{
	for(;;)
	{
		Event_Wait(evt3);
		Event_Signal(evt2);
		Event_Signal(evt1);		
	}
}

void Task_P1(int parameter)
{
	Event_Wait(evt1);
    Task_Terminate();
    for(;;);
}

void Task_P2(int parameter)
{
	Event_Wait(evt2);
    Task_Terminate();
    for(;;);
}

void Task_P3(int parameter)
{
    for(;;);
}

void configure_timer()
{
    //Clear timer config.
    TCCR3A = 0;
    TCCR3B = 0;  
    //Set to CTC (mode 4)
    TCCR3B |= (1<<WGM32);

    //Set prescaller to 256
    TCCR3B |= (1<<CS32);

    //Set TOP value (0.1 seconds)
    OCR3A = 6250;

    //Set timer to 0 (optional here).
    TCNT3 = 0;
    
    //Enable interupt A for timer 3.
    TIMSK3 |= (1<<OCIE3A);
}

void timer_handler()
{
    Event_Signal(evt3);
}

ISR(TIMER3_COMPA_vect)
{
    TIMSK3 &= ~(1<<OCIE3A);
    timer_handler();
}

void a_main(int parameter)
{
	/*
	//Place these as necessary to display output if not already doing so inside the RTOS
	//initialize pins
	DDRB |= (1<<PB1);	//pin 52
	DDRB |= (1<<PB2);	//pin 51	
	DDRB |= (1<<PB3);	//pin 50
	
	
	PORTB |= (1<<PB1);	//pin 52 on
	PORTB |= (1<<PB2);	//pin 51 on
	PORTB |= (1<<PB3);	//pin 50 on


	PORTB &= ~(1<<PB1);	//pin 52 off
	PORTB &= ~(1<<PB2);	//pin 51 off
	PORTB &= ~(1<<PB3);	//pin 50 off

	*/
	evt1 = Event_Init();
    evt2 = Event_Init();
    evt3 = Event_Init();

	Task_Create(Task_P1, 1, 0);
	Task_Create(Task_P2, 2, 0);
	Task_Create(Task_P0, 0, 0);
	Task_Create(Task_P3, 3, 0);
	
    configure_timer();
}