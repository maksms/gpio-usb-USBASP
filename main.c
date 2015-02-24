/* Name: main.c
GPIO-USB:DHT,rcremote
Очень сырой код. Это первые опыты на Си с м/к AVR.
 */


//unsigned int
unsigned char datadht[5];

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for delayMicroseconds() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "requests.h"       /* The custom request numbers we use */
#include <avr/eeprom.h>

int hi,lo;
unsigned int delay_counter=60*1000-100;

//------------------------------------------------
//------------------Подпрограммы------------------
//------------------------------------------------
#if HIDMODE
PROGMEM const char usbHidReportDescriptor[22] = {   /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
#endif
int dhtread ()
{
#define DHT_PORT        PORTB
#define DHT_DDR         DDRB
#define DHT_PIN         PINB
#define DHT_BIT         4


uint8_t  j = 0, i = 0;


DHT_DDR|=(1<<DHT_BIT); //pin as output
DHT_PORT&=~(1<<DHT_BIT); //0
_delay_ms(18);
DHT_PORT|=(1<<DHT_BIT); //1
DHT_DDR&=~(1<<DHT_BIT); //pin as input
//=============check DHT11 response
_delay_us(50);
if (DHT_PIN&(1<<DHT_BIT)) return 0;
_delay_us(80);
if (!(DHT_PIN&(1<<DHT_BIT))) return 0;

//===============receive 40 data bits
while (DHT_PIN&(1<<DHT_BIT));
for (j=0; j<5; j++)
    {
    datadht[j]=0;
    for(i=0; i<8; i++)
        {
        while (!(DHT_PIN&(1<<DHT_BIT)));
        _delay_us (30);
        if (DHT_PIN&(1<<DHT_BIT))
            datadht[j]|=1<<(7-i);
        while (DHT_PIN&(1<<DHT_BIT));
        }
    }




	return 1;
}


int mode_pin (unsigned int pin,unsigned int pinmode)
{


if(pinmode==1) { // ввод

if(pin==1)  DDRC &= ~_BV(0); //1
if(pin==2)  DDRC &= ~_BV(1); //2
if(pin==3)  DDRB &= ~_BV(2); //3
if(pin==4)  DDRB &= ~_BV(3); //4
if(pin==5)  DDRB &= ~_BV(4); //5
if(pin==6)  DDRB &= ~_BV(5); //6

if(pin==7)  DDRD &= ~_BV(0); //7
if(pin==8)  DDRD &= ~ _BV(1); //8

} else {
   // --- вывод
if(pin==1)  DDRC |= _BV(0); //1
if(pin==2)  DDRC |= _BV(1); //2
if(pin==3)  DDRB |= _BV(2); //3
if(pin==4)  DDRB |= _BV(3); //4
if(pin==5)  DDRB |= _BV(4); //5
if(pin==6)  DDRB |= _BV(5); //6

if(pin==7)  DDRD |= _BV(0); //7
if(pin==8)  DDRD |= _BV(1); //8

  //  PORTC &= ~_BV(0); // 0
  //  PORTC &= ~_BV(1); // 0
if(pin==1)   PORTC |= _BV(0); // 1
if(pin==2)   PORTC |= _BV(1); // 1


}

return 0;
}


void delayMicroseconds(unsigned int us)
{
 int i;
for(i=0; i<us; i++) _delay_us(100);
}


void sendRC(unsigned long data,unsigned int periodusec)
{ // , unsigned short pin 531 440 max

    DDRB |= _BV(5); //6 режим на вывод

    data |= 3L << 20;

	unsigned short repeats = 1 << (((unsigned long)data >> 20) & 7);
	data = data & 0xfffff; //truncate to 20 bit

	unsigned long dataBase4 = 0;
	unsigned short i;
	 cli();                              //Disable interrupts
	for (i=0; i<12; i++) {
		dataBase4<<=2;
		dataBase4|=(data%3);
		data/=3;
	}
	unsigned short int j;
	for (j=0;j<repeats;j++) {

		data=dataBase4;

		unsigned short i;
		for (i=0; i<12; i++) {
			switch (data & 3) {
				case 0:
					PORTB |= _BV(5);
					delayMicroseconds(periodusec);
					PORTB &= ~_BV(5);
					delayMicroseconds(periodusec*3);
					PORTB |= _BV(5);
					delayMicroseconds(periodusec);
					PORTB &= ~_BV(5);
					delayMicroseconds(periodusec*3);
					break;
				case 1:
					PORTB |= _BV(5);
					delayMicroseconds(periodusec*3);
					PORTB &= ~_BV(5);
					delayMicroseconds(periodusec);
					PORTB |= _BV(5);
					delayMicroseconds(periodusec*3);
					PORTB &= ~_BV(5);
					delayMicroseconds(periodusec);
					break;
				case 2:
					PORTB |= _BV(5);
					delayMicroseconds(periodusec);
					PORTB &= ~_BV(5);
					delayMicroseconds(periodusec*3);
					PORTB |= _BV(5);
					delayMicroseconds(periodusec*3);
					PORTB &= ~_BV(5);
					delayMicroseconds(periodusec);
					break;
			}

			data>>=2;
		}

		PORTB |= _BV(5);
		delayMicroseconds(periodusec);
		PORTB &= ~_BV(5);
		delayMicroseconds(periodusec*31);
	}
sei();                          //Enable interrupts
DDRB &= ~_BV(5); //6 режим на ввод,чтобы не блокировать цепь
}


/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;
static uchar    dataBuffer[8];  /* buffer must stay valid when usbFunctionSetup returns */

    if(rq->bRequest == 0){ /* echo -- used for reliability tests  CUSTOM_RQ_ECHO */
        dataBuffer[0] = rq->wValue.bytes[0];
        dataBuffer[1] = rq->wValue.bytes[1];
        dataBuffer[2] = rq->wIndex.bytes[0];
        dataBuffer[3] = rq->wIndex.bytes[1];
        usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
        return 4;
    }else if(rq->bRequest == 1){

         if(rq->wValue.bytes[0] == 3){ // B2
        if(rq->wValue.bytes[1] & 1) PORTB |= _BV(2);
        else			    PORTB &= ~_BV(2);

         }else if(rq->wValue.bytes[0] == 4){ // B3
        if(rq->wValue.bytes[1] & 1) PORTB |= _BV(3);
        else			    PORTB &= ~_BV(3);

         }else if(rq->wValue.bytes[0] == 5){ // B4
        if(rq->wValue.bytes[1] & 1) PORTB |= _BV(4);
        else			    PORTB &= ~_BV(4);

         }else if(rq->wValue.bytes[0] == 6){ // B5
        if(rq->wValue.bytes[1] & 1)PORTB |= _BV(5);
        else	  		   PORTB &= ~_BV(5);

        }else if(rq->wValue.bytes[0] == 7){ // D0
        if(rq->wValue.bytes[1] & 1) PORTD |= _BV(0);
        else 			    PORTD &= ~_BV(0);


        }else if(rq->wValue.bytes[0] == 8){ // D1
        if(rq->wValue.bytes[1] & 1) PORTD |= _BV(1);
	else 			    PORTD &= ~_BV(1);


        }else if(rq->wValue.bytes[0] == 1){ //
        if(rq->wValue.bytes[1] & 1) PORTC &= ~_BV(0);
        else                        PORTC |= _BV(0);

            }else if(rq->wValue.bytes[0] == 2){
        if(rq->wValue.bytes[1] & 1) PORTC &= ~_BV(1);
        else 			    PORTC |= _BV(1);
return 0;
	    }
        ////////////////////////
        }else if(rq->bRequest == 2){ // переключение режима пина.
	dataBuffer[0]=rq->wValue.bytes[0];

	mode_pin((rq->wValue.bytes[0]),(rq->wValue.bytes[1]));
        usbMsgPtr = dataBuffer;
        return 1;

      }else if(rq->bRequest == 3){ // переключение режима пина и запись режима в еером


        eeprom_write_byte((uint8_t *)((rq->wValue.bytes[0])+255),(int)(rq->wValue.bytes[1]));

	dataBuffer[0]=rq->wValue.bytes[0];

	mode_pin((rq->wValue.bytes[0]),(rq->wValue.bytes[1]));
        usbMsgPtr = dataBuffer;
        return 1;

	}else if(rq->bRequest == 4){ // RC remote

	sendRC(rq->wValue.bytes[0]+(unsigned int)(256*(rq->wValue.bytes[1]))+(unsigned long)(65536*(rq->wIndex.bytes[0])),(rq->wIndex.bytes[1])); // +(unsigned int)(65536*(rq->wIndex.bytes[0]))

	return 0;

	}else if(rq->bRequest == 5){ // dhtsetup read

	dataBuffer[0]=(uint8_t *)eeprom_read_byte(500);
        usbMsgPtr = dataBuffer;
	return 1;

	}else if(rq->bRequest == 6){ // dhtsetup write

	 eeprom_write_byte(500,(uint8_t *)(rq->wValue.bytes[0]));

delay_counter=60*1000-100; // обнуление счетчика

	  return 0;

	}else if(rq->bRequest == 17){ // analoginput ---------------------------------------------------------------


	// общ
	dataBuffer[0]  = lo;
	dataBuffer[1] = hi;
//	dataBuffer[0]  = 1;
//	dataBuffer[1] = 1;
	// или ADCW
	        usbMsgPtr = dataBuffer;
        return 2;


	//------------------------------------------------------------------------------------------------------------

	 }else if(rq->bRequest == 22){
	   // -----------dht--------
	 dataBuffer[0]=  datadht[0];
	 dataBuffer[1]=  datadht[1];
	 dataBuffer[2]=  datadht[2];
	 dataBuffer[3]=  datadht[3];
	 dataBuffer[4]=  datadht[4];
	   //--------dht------------
	 usbMsgPtr = dataBuffer;
        return 5;

	 }else if(rq->bRequest == 24){ //  PWM https://sites.google.com/site/qeewiki/books/avr-guide/pwm-atmega8
    DDRB |= (1 << DDB3);
    // PB3 is now an output
    OCR2 = rq->wValue.bytes[0];

    TCCR2 |= (1 << COM21);
    // set none-inverting mode

    TCCR2 |= (1 << WGM21) | (1 << WGM20);
    // set fast PWM Mode

    TCCR2 |= (1 << CS21);
    // set prescaler to 8 and starts PWM
      //  usbMsgPtr = dataBuffer;

    return 0;

    }else if(rq->bRequest == 23){

    DDRB |= (1 << DDB2);
    OCR1B = rq->wValue.bytes[0];
    TCCR1A |= (1 << COM1B1);
    TCCR1A|= (1 << WGM11) | (1 << WGM10);
    TCCR1B |= (1 << CS11);

        return 0;


	//////////////////////////
     }else if(rq->bRequest == 21){ //    чтение режимов портов из ЕЕРОМ
       uint8_t i;
       for (i=0;i<8;i++)
       {
	dataBuffer[i]= eeprom_read_byte((const uint8_t *)(i+256));


}
usbMsgPtr = dataBuffer;
return i;

     }else if(rq->bRequest == 24){ // чтение состояния 1 порта на ввод

 int pin=(rq->wValue.bytes[0]);

if(pin==1)  dataBuffer[0] = ((PINC & _BV(0)) != 0); //1
if(pin==2)  dataBuffer[0] = ((PINC & _BV(1)) != 0); //2
if(pin==3)  dataBuffer[0] = ((PINB & _BV(2)) != 0); //3
if(pin==4)  dataBuffer[0] = ((PINB & _BV(3)) != 0); //4
if(pin==5)  dataBuffer[0] = ((PINB & _BV(4)) != 0); //5
if(pin==6)  dataBuffer[0] = ((PINB & _BV(5)) != 0); //6

if(pin==7)  dataBuffer[0] = ((PIND & _BV(0)) != 0); //7
if(pin==8)  dataBuffer[0] = ((PIND & _BV(1)) != 0); //8


      usbMsgPtr = dataBuffer;
       return 1;
      }else if(rq->bRequest == 25){ // чтение состояния портов на ввод

dataBuffer[0] = ((PINC & _BV(0)) != 0); //1
dataBuffer[1] = ((PINC & _BV(1)) != 0); //2
dataBuffer[2] = ((PINB & _BV(2)) != 0); //3
dataBuffer[3] = ((PINB & _BV(3)) != 0); //4
dataBuffer[4] = ((PINB & _BV(4)) != 0); //5
dataBuffer[5] = ((PINB & _BV(5)) != 0); //6

dataBuffer[6] = ((PIND & _BV(0)) != 0); //7
dataBuffer[7] = ((PIND & _BV(1)) != 0); //8


      usbMsgPtr = dataBuffer;
       return 8;

    }else if(rq->bRequest == CUSTOM_RQ_GET_STATUS){ // статусы портов на вывод
        dataBuffer[0] = ((PORTC & _BV(0)) == 0);
	dataBuffer[1] = ((PORTC & _BV(1)) == 0);

	dataBuffer[2] = ((PORTB & _BV(2)) != 0);
	dataBuffer[3] = ((PORTB & _BV(3)) != 0);
	dataBuffer[4] = ((PORTB & _BV(4)) != 0);
	dataBuffer[5] = ((PORTB & _BV(5)) != 0);

	dataBuffer[6] = ((PORTD & _BV(0)) != 0);
	dataBuffer[7] = ((PORTD & _BV(1)) != 0);

        usbMsgPtr = dataBuffer;         /* tell the driver which data to return */
        return 8;                       /* количество отправляемых байт !! */
    }


    return 0;   /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */

int __attribute__((noreturn)) main(void)
{
uchar   i;

    wdt_enable(WDTO_1S);


    odDebugInit();
    DBG1(0x00, 0, 0);       /* debug output: main starts */
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    DDRC |= _BV(0); //1  /* make the LED bit an output */
    DDRC |= _BV(1); //2

    DDRB |= _BV(2); //3
    DDRB |= _BV(3); //4
    DDRB |= _BV(4); //5
    DDRB |= _BV(5); //6

    DDRD |= _BV(0); //7
    DDRD |= _BV(1); //8
  //  PORTC &= ~_BV(0); // 0
  //  PORTC &= ~_BV(1); // 0
    PORTC |= _BV(0); // 1
    PORTC |= _BV(1); // 1

 // -----

    // -----
    sei();
  //  DBG1(0x01, 0, 0);       /* debug output: main loop starts */
    for(;;){                /* main event loop */
   //     DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        wdt_reset();


//	-------------------------------------------------------------------------------------------------------------
	_delay_ms(1);

	if (delay_counter==60*1000) {
	  delay_counter=0;


         datadht[0] = datadht[1] = datadht[2] = datadht[3] = datadht[4] = 0;
	 if ((uint8_t *)eeprom_read_byte(500)==1) {


	   cli();
	   dhtread ();
	 sei();


	}



	}
	delay_counter++;
        //	-------------------------------------------------------------------------------------------------------------
	//
	usbPoll();
    } //
}

/* ------------------------------------------------------------------------- */


