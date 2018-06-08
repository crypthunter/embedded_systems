#include <c8051f020.h> // SFR definitions
//bottone e led
sbit button = P3^7;
sbit led = P0^6;
//contatore per il timer1
unsigned char t1overFlow = 0;
// 0 = bottone non premuto, 1 = bottone premuto
unsigned char premuto = 0;
//se è passato più di un secondo da quando il bottone è stato premuto
unsigned char unSec = 0;
//variabile per decidere se il led deve lampeggiare ( molto velocemente così da sembrare ad una luminosità differente)
//oppure restare spento
unsigned char acceso = 1;
//variabile per controllare la luminosità
unsigned char lumi;
void init (void) {
	//abilita iinterrupt globali
	EA = 1;
	//disabilita watchdog timer
	WDTCN = 0xde;
	WDTCN = 0xad; 
	//oscillatore
	OSCICN &= 0x14;
	//clock interno
	XBR0=0x00;
	//crossbar, gestisce i pin
	XBR1 = 0x00;
	XBR2=0x40;
	//push-pull
	P1MDOUT |= 0x40;
	//push pull display
	P0MDOUT |= 0x40;
	//per l'interrupt del bottone
	EIE2 |= 0x20;
	//iinterrupt del smbus
	EIE1 |= 0x02;
	//abilita smbus
	ENSMB = 1;
}

//resetta il timer 2 senza fermarlo
void resetTimer2()
{
	TF2 = 0;
	TH2 = 0xf9;
	TL2 = 0x7e;	
}

void setLumi()
{
	resetTimer2();
	TR2 = 1;
}

//quando il timer va in iinterrupt sono passati 200 ms, incrementa di poco la luminosità del led
void timer2() interrupt 5
{
	char lumiStep = 1;
	lumi = lumi + lumiStep;
	resetTimer2();
}

//resetta il timer per contare 200ms, e t1overFlow a 0 per contare 1s
void resetTimer1()
{
		t1overFlow = 0;
		TF1 = 0;
		TH1 = 0x7d;
		TL1 = 0xcb;	
		TR1 = 0;
}

//imposta il necessario per il programma
void pwm_setup()
{
	//timer 0 a 8 bbit, timer 1 a 16 bbit
	TMOD = 0x10;
	//clock intero per il timer0
	CKCON = 0x8;
	//luminosità iniziale del led, 0 = luminosità massima
	lumi = 100;
	//abilita gli iinterrupt di timer0, timer1 e timer2
	ET0 = 1;
	ET1 = 1;
	ET2 = 1;
	//imposta il valore iniziale del timer 1 (conta 200 ms)
	TH1 = 0x7d;
	TL1 = 0xcb;
	//fa partire il timer 0 che gestisce pwm
	TR0 = 1;
}
 
//iinterrupt del timer0
void timer0() interrupt 1
{
	//il led lampeggia solo se la variabile è a 1
	if (acceso == 1)
	{
		if (!led) {	
			led = 1;		// cambia stato led
			TH0 = lumi;	
			TF0 = 0;		//pulisce flag interrput
		}
		else {			
			led = 0;
			TH0 = 255 - lumi;	
			TF0 = 0;
		}
	}
}

void buttonInt() interrupt 19
{
	//se non stavo premendo il bottone e lo premo
	if(premuto == 0)
	{
		//fa partire il timer che deve contare 1 secondo
		TR1 = 1;
		//resetta l'interrupt
		EIE2 |= 0x20;
		P3IF &= 0x7f;
		//cambia in rising edge per avere un interruptt al rilascio del bottone
		P3IF |= 0x08;
		premuto = 1;
	}
	//se sto premendo il bottone e quando rilascio non è passato un secondo, deve cambiare lo stato del led
	else if (premuto == 1 && unSec == 0)
	{
		//resetto l'interrupt
		EIE2 |= 0x20;
		P3IF &= 0x7f;
		//rimette in falling edge il bottone
		P3IF &= 0x77;
		//cambio stato led
		acceso = !acceso;
		led = 0;
		premuto = 0;
		unSec = 0;
		//resetto e disabilito il timer
		resetTimer1();
	}
	//se lascio il bottone ma è passato più di un secondo
	else if (premuto == 1 && unSec == 1)
	{
		//devo fermare il timer2 usato per regolare la luminosità
		TR2 = 0;
		resetTimer2();
		premuto = 0;
		unSec = 0;
		//resetta e disabilito il timer
		resetTimer1();
		//resetta l'interrupt
		EIE2 |= 0x20;
		P3IF &= 0x7f;
		//rimette in falling edge il bottone
		P3IF &= 0x77;
	}
}

void timer1() interrupt 3
{
	t1overFlow++;
	//se il contatore è a 5 significa che è passato 1 secondo
	if(t1overFlow == 5)
	{
		//fermo il timer
		resetTimer1();
		unSec = 1;
		//imposto la luminosità solo se il bottone è acceso
		if (acceso == 1)
		{
			setLumi();
		}
	}
	else
	{
		TF1 = 0;
		TH1 = 0x7d;
		TL1 = 0xcb;	
	}
}

void main()
{
	init();
	pwm_setup();
	while(1);
}