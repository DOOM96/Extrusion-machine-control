/*
 * TT2_6.c
 *
 * Created: 10/12/2019 11:14:23 p. m.
 *  Author: danieloscar
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

#define F_CPU 8000000UL
#define SAMPLE_TIME 8333
#define EN PORTB2
#define DIR PORTB3
#define PULSE PORTB1
#define ZERO PORTD2
#define LED PORTB0


//variables de recepción de datos de temperatura
int rec[6] = {48,48,48,48,48,48}; //recepción del set point en ASCII
int rec1[6] = {0,0,0,0,0,0}; //recepción del nible bajo del set point
//variable de recepción de ADC de lectura de los termopares
int temp[2][5] = {{0,0,0,0,0},{0,0,0,0,0}};
int tempcomp = 0;
int tempT = 0;
//arreglo de secuencias para el motor a pasos
uint8_t sec[4] = {0x0A,0x09,0x05,0x06};
//variables de indices de arreglos
int i = 0,j=0,n=0;
int k = 0;
//variable indicadora de que termopar es leído
int pos = 4;
//variable para guardar el set point de cada temperatura
int tempc[4] = {0,0,0,0};
//acumulador de temperatura cuando es mandada por serial
int res=0;

//variables para setpoints
int sp1 = 0, sp2 = 0, sp3 = 0, sp4 = 0;

//Errores
int E1[2] = {0,0}, E2[2] = {0,0}, E3[2] = {0,0} , E4[2]={0,0};
int dE1 = 0, dE2=0, dE3=0;
int IE1 = 0, IE2 = 0, IE3 = 0;

//salidas
int y1 = 0, y2 = 0, y3 = 0, y4 = 0;

////diferenciales de tiempo
//float dt = 0.00005;
//
////acumuladores de integrador
//int ac1 = 0, ac2 = 0, ac3 = 0;
//int p1 = 0, p2 = 0, p3 = 0;

//ganancias de controladores
int kp = 1.5, kd = 0.0001, ki =0.5;

int aux = 0, aux1 = 0;
int cont = 0, cont1 = 0, cont2 = 0, cont3 = 0;

int state = 1, state1 = 1, state2 = 1, state3 = 1;
int t = 150, t1 = 150;


//variables pwm control de velocidad de motor
int fp[2] = {100,100};
int p = -1;
int TOP = 9999;
int h = -1;



ISR(USART_RX_vect)
{
	rec[k] = (int)UDR0;
	rec1[k] = rec[k] & 0x0F;
	if (k>3)
	{
		//se revisan que todos los datos estén dentro de los valores esperados
		//primer valor sea una "t"
		//el segundo valor este comprendido entre 1 y 3
		//y los tres valores posteriores sean números
		if ((rec[0] == 116) & (rec[1]>48 & rec[1]<53) & (rec[2] >= 48 & rec[2]<=57) & (rec[3] >= 48 & rec[3]<=57) & (rec[4] >= 48 & rec[4]<=57))
		{
			k=0;
			tempc[rec1[1]-1] = rec1[2]*100+rec1[3]*10+rec1[4];
			rec[0] = 0;
			rec[1] = 0;
			rec[2] = 0;
			rec[3] = 0;
			rec[4] = 0;
		}else if (rec[0] == 105) //empezar todo el proceso
		{
			TIMSK0 |= (1<<OCIE0A);//se activan las interrupciones que controlan la temperatura y la velocidad del motor
			TIMSK2 |= (1<<TOIE2);
			PORTB &= ~(1<<LED);
			k=0;
			rec[0] = 0;
			rec[1] = 0;
			rec[2] = 0;
			rec[3] = 0;
			rec[4] = 0;
		
		}else if (rec[0] == 101) //cancelar todo el proceso
		{
			TIMSK0 &= ~(1<<OCIE0A); //se desactivan las interrupciones que controlan los procesos de control de velocidad y de temperatura
			TIMSK2 &= ~(1<<TOIE2);
			k=0;
			rec[0] = 0;
			rec[1] = 0;
			rec[2] = 0;
			rec[3] = 0;
			rec[4] = 0;
		}else if (rec[0] == 99) //cambio de velocidad
		{
			int vel = rec1[2]*100+rec1[3]*10+rec1[4];
			fp[0] = vel*512;
			p=1; //se llama a la rutina de cambio de velocidad
			PORTB &= ~(1<<LED);
			k=0;
			rec[0] = 0;
			rec[1] = 0;
			rec[2] = 0;
			rec[3] = 0;
			rec[4] = 0;
			
		}else if (rec[0] == 97) //activar motor
		{
			int vel = rec1[2]*100+rec1[3]*10+rec1[4];
			fp[0] = vel*512; //se realiza el ajuste de la frecuencia requerida de acuerdo a la velocidad que el usuario ingresa
			h=1;//se enciende el proceso de rranque de motor y de cambio de velocidad
			p=1;//se llama a la rutina de cambio de velocidad del motor
			k=0;
			DDRB |= (1<<PULSE); //se activa la salida del pulso
		    PORTB &= ~(1<<LED);
			rec[0] = 0;
			rec[1] = 0;
			rec[2] = 0;
			rec[3] = 0;
			rec[4] = 0;
		}
		else if (rec[0] == 100) //desactivar motor
		{
			h=0; //se manda a la rutina para el frenado
			k=0;
			PORTB &= ~(1<<LED);
		}
		else
		{
			k=0;
			rec[0] = 0;
			rec[1] = 0;
			rec[2] = 0;
			rec[3] = 0;
			rec[4] = 0;
		}			
		
	}else
	{
		k++;
	}		
		
}

int val1 = 0, val2 = 0;
int n1 = 0;
ISR(ADC_vect)
{
	
	val1 = ADCL;
	val2 = ADCH;
	
	temp[1][n] = pos; //se les asigna un lugar a los datos leídos
	temp[2][n] = ((val2<<8|val1)+87.434)/4.5505;
		
	n++;
	if (n>3)
	{
		n=0;
	}
	ADMUX &= 0xE0; //se realiza la multiplexión de los puertos de ADC
	ADMUX |= n;
}

ISR(TIMER2_OVF_vect)
{
	
	TCNT2 = -234; //tiempo de 30ms
	
	switch(i)
	{
		case 0:
		UDR0 = temp[1][n]; //se envía el identificador del sensor de temperatura (1,2,3 o 4)
		i=1;
		break;
		case 1:
		UDR0 =  temp[2][n] / 2; //se envía el valor que se lee divido entre dos por cuestiones de comunicación, la interfaz realizará el ajuste
		i=2;
		break;
		case 2:
		UDR0 = 44;
		i=3;
		break;
		case 3:
		UDR0  = temp[1][n] + 4; // se envía el dato que el micro recibe de la interfaz, para corroborar que el dato se haya guardado, la posición 
		i=4;
		break;
		case 4:
		UDR0 = tempc[n] / 2; // y el dato de temperatura que se desea
		i=5;
		break;
		case 5:
		UDR0 = 44;
		i=0;
		pos++;
		if (pos>4)
		{
			pos = 1; 
		}
		ADCSRA |= (1<<ADSC); //se pide la siguiente lectura de los ADC
		break;
		default:
		UDR0 = 0X2C;
		i = 0;
		break;
	}	
}

int c = 0;
ISR(TIMER0_COMPA_vect)
{
	//PORTD ^= (1<<PORTD7);
	c++;
	/////////////////////////////////////control de temperatura 1
	if (tempc[0] > temp[2][0])//sólo si el error es distinto de cero se activa el pulso para el disparo de angulo de fase
	{
		PORTB |= (1<<PORTB4);
	}else
	{
		PORTB &= ~(1<<PORTB4);
	}
	//
	///////////////////////////////////////control de temperatura 2
	//
	if (tempc[1]>temp[2][1])//sólo si el error es distointo de cero se activa el pulso para el disparo de angulo de fase
	{
		PORTB |= (1<<PORTB5);	
	}else
	{
		PORTB &= ~(1<<PORTB5);
	}
	//
	/////////////////////////////////////////control de temperatura 3
	//
	if (tempc[2]>temp[2][2])//sólo si el error es distointo de cero se activa el pulso para el disparo de angulo de fase
	{
		PORTB |= (1<<PORTB6);
	}else
	{
		PORTB &= ~(1<<PORTB6);
	}
	//
	//
	///////////////////////////////////////////control de temperatura 4/////////////////////////////
	//
	if (tempc[3]>temp[2][3]) //sólo si el error es distinto de cero se activa el pulso para el disparo de angulo de fase
	{
		PORTB |= (1<<PORTB7);
	}else
	{
		PORTB &= ~(1<<PORTB7);
	}
		
	/////////////////////////////////////////////CONTROL DE MOTOR////////////////////////////////////////////////////
	//
	if(c>22)
	{
		////
		if (h == 1)
		{
			PORTB &= ~(1<<EN);
			switch(p)
			{
				case 1:
				if (fp[0]>fp[1])
				{
					p = 2;
				}else if (fp[0]<fp[1])
				{
					p = 3;
				}else
				{
					p=1;
				}
					
				break;
					
				case 2: //frecuencia deseada es mayor que la frecuencia actual
				fp[1] = fp[1]+1;
				if (fp[1] >= fp[0])
				{
					p=-1;
					fp[1] = fp[0];
					PORTB |= (1<<LED);
						
				}
				break;
					
				case 3: //frecuencia deseada es menor que la frecuencia actual
				fp[1] = fp[1]-1;
				if (fp[1] <= fp[0])
				{
					p=-1;
					fp[1] = fp[0];
					PORTB |= (1<<LED);
				}
				break;
					
				default:
				fp[0] = fp[0];
				break;
					
			}
			OCR1A = TOP;
		}else if (h == 0) //rutina de paro del stepper
		{
			if (fp[1] > 0) //si la frecuencia actual es mayor que cero, comenzará a disminuir una unidad cada vez
			{
				fp[1] = fp[1] - 1;
				OCR1A = TOP; //se asigna el valor calculado del TOP al registro OCR1A para cambiar la frecuencia del pulso
			}else
			{
				PORTB |= (1<<LED);
				PORTB |= (1<<EN);
			}
		}
		c=0;
	}
		
}

ISR(INT0_vect)
{
	///////////////////////cruce por cero 
	aux = 1;
}

int main(void)
{
	cli();
	//configuracion puerto serial
	UCSR0B |= (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);//CONFIGURACION DE INTERRUPCIONES Y HABILITACION DE RECEPCION Y TRANSMISION
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ02) ; //configuracion de 8 bits de trama, 1 bit de parada, asincrona
	UBRR0L = 52; //BAUDRATE DE 9600 a 8MHz
	
	//configuracion Timer0
	TCCR0A |= (1<<WGM01); //CTC Mode
	TCCR0B |= (1<<CS01) ; //preescaler de 8
	OCR0A = 45; //100us
	
	//////configuracion de timer 1
	TCCR1A |= (1<<COM1A0) | (1<<WGM10) | (1<<WGM11) ; //Fast-PWM, OCR1A TOP, toggle
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS11); //prescaler de 8
	OCR1A = 100;
	
	//configuracion de ADC
	ADMUX |= (1<<REFS0) | (1<<REFS1); //se configura referencia de voltaje interna 1.1V y alineacion a la izquierda de los datos convertidos
	ADCSRA |= (1<<ADEN) | (1<<ADIE); //se habilita el ADC, se inicia la primer conversion, se habilita la interrupcion
	
	//configuración Timer 2
	TCCR2B |= (1<<CS20) | (1<<CS21) | (1<<CS22); //Preescaler de 1024
	TCNT2 = -234; //tiempo de 30ms
	
	//configuracion interrupción externa
	EICRA |= (1<<ISC01); // activacion por flanco de bajada
	EIMSK |= (1<<INT0); // enable de interrupcion externa en INT0
	//
	sei();
	
	DDRB = 0XFD;//declaracion de puerto B como salida
	DDRD = 0XC2;//pines de TX y RX declarados como salida //CAMBIAR A 0Xc3
	PORTB |= (1<<EN);
	PORTD |= (1<<ZERO);
	
    while(1)
    {	
		PORTB |= (1<<DIR);
		TOP = round(F_CPU/(8*fp[1]) - 1);
		
		//E1[0] = abs(tempc[0]-temp[2][0]);
		//E2[0] = abs(tempc[1]-temp[2][1]);
		//E3[0] = abs(tempc[2]-temp[2][2]);
		//E4[0] = abs(tempc[3]-temp[2][3]);
		//
		//y1 = round(t - ((kp*E1[0])*t1/255));
		//y2 = round(t - ((kp*E2[0])*t1/255));
		//y3 = round(t - ((kp*E3[0])*t1/255));
		//y4 = round(t - ((kp*E4[0])*t1/255));
		
    }
}