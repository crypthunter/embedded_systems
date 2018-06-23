#include <c8051f020.h> // SFR definitions

//---------------------------------------------------VARIABILI----------------------------------------------------------------------
// 0 accelerometro, 1 temperatura, 2 display
unsigned char select_interrupt;
//quante volte va in overflow
unsigned char t3_overflow_display = 0;
unsigned char t3_overflow_temp = 0;
unsigned char flag_mma = 0;
unsigned char flag_temp = 0;
unsigned char flag_display = 0;
unsigned char smBusy = 0;
//tipo di azione che deve fare smbus
// 0 = mma, 1 = display, 2 = temperatura
unsigned char interrupt_type = 0;
//variabili per la media
unsigned char avg_cont = 0;
int avg_x = 0;
int avg_y = 0;
int avg_z = 0;
//------------------------------------------------------PWM--------------------------------------------------------------------------
//bottone e backlight
sbit button = P3^7;
sbit backlight = P0^6;
//contatore per il timer1
unsigned char t1overFlow = 0;
// 0 = bottone non premuto, 1 = bottone premuto
unsigned char premuto = 0;
//se è passato più di un secondo da quando il bottone è stato premuto
unsigned char unSec = 0;
//variabile per decidere se il backlight deve lampeggiare ( molto velocemente così da sembrare ad una luminosità differente)
//oppure restare spento
unsigned char acceso = 1;
//variabile per controllare la luminosità
unsigned char lumi;
char lumiStep = 1;
//--------------------------------------------COSTANTI PERIFERICHE-----------------------------------------------------------------
#define SMB_START 0x08 //ricevuto start
#define SMB_RESTART 0x010 //ricevuto restart

#define SMB_FIRSTWRITE 0x18 //scrivo il primo valore
#define SMB_WRITE 0x28 // scrivo gli altri valori

#define SMB_FIRSTREAD 0x40  //primo read
#define SMB_READ 0x50 //altri read
#define SMB_READ_NACK 0x58  //dopo che ho dato AA = 0

#define MMA_WRITE 0x98 //indirizzo per scrivere sull'accelerometro
#define MMA_READ 0x99  //indirizzo per leggere dall'accelerometro

#define TEMP_READ 0x91  //indirizzo per leggere dal termometro

#define DISPLAY_WRITE 0x7c  //indirizzo per leggere dal display

//--------------------------------------------------------ACCELEROMETRO-------------------------------------------------------------
//definizione registri accelerometro
#define 	XOUT				0x00
#define 	YOUT				0x01
#define 	ZOUT				0x02
#define 	MODE				0x07

unsigned char mma_init [] = {MODE, 0x01};
unsigned char mma_pos = 0;
unsigned char mma_init_finished = 0;
unsigned char mma_read_ready = 0;

char buffer_x[8] = {99, 99, 99, 99, 99, 99, 99, 99};
char buffer_y[8] = {99, 99, 99, 99, 99, 99, 99, 99};
char buffer_z[8] = {99, 99, 99, 99, 99, 99, 99, 99};
unsigned char buffer_pos = 0;

int mma_value_read = 0;
int i = 0;
float xyz[3];
unsigned char xyz_mma_pos = 0;
code float TILT_XY[64] = {0, 2.69, 5.38, 8.08, 10.81, 13.55, 16.33, 19.16, 22.02, 24.95, 27.95, 31.04, 34.23, 37.54, 41.01, 44.68, 48.59, 52.83, 57.54, 62.95, 69.64, 79.86, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, -79.86, -69.64, -62.95, -57.54, -52.83, -48.59, -44.68, -41.01, -37.54, -34.23, -31.04, -27.95, -24.95, -22.02, -19.16, -16.33, -13.55, -10.81, -8.08, -5.38, -2.69}; 
code float TILT_Z[64] = {90.00, 87.31, 84.62, 81.92, 79.19, 76.45, 73.67, 70.84, 67.98, 65.05, 62.05, 58.96, 55.77, 52.46, 48.99, 45.32, 41.41, 37.17, 32.46, 27.05, 20.36, 10.14, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10.14, -20.36, -27.05, -32.46, -37.17, -41.41, -45.32, -48.99, -52.46, -55.77, -58.96, -62.05, -65.05, -67.98, -70.84, -73.67, -76.45, -79.19, -81.92, -84.62};

//---------------------------------------DISPLAY---------------------------------------------
unsigned char display_init_values[] = {0x38, 0x39, 0x14, 0x74, 0x54, 0x6F, 0x0C, 0x0F, 0x01};
unsigned char display_values[] = {0x80, 0x01, 0x40, 'T',':', '2', '0', 0x80, 0xC0, 0x40, 'X', ':', '2' , '0', 'Y', ':', '5', '0', 'Z', ':', '4', '0'};
//variabile che indica se l'init è finito
unsigned char display_init = 0;
unsigned char display_init_pos = 0;
unsigned char cont = 0;
unsigned char cont1 = 0;
unsigned char write_finished = 0;
//---------------------------------------TEMPERATURA-----------------------------------------
int tempH = 0;
int tempL = 0;
//variabile che indica se ho letto la parte alta della temperatura
unsigned char readH = 0;
float temp_float = 0;
int temp_int = 0;
int decine = 0;
int unita = 0;
//---------------------------------------PROGRAMMA-------------------------------------------
void init (void) {
	//abilita iinterrupt globali
	EA = 1;
	//disabilita watchdog timer
	WDTCN = 0xde;
	WDTCN = 0xad; 
	//oscillatore
	OSCICN &= 0x14;
	//clock interno
	XBR0 = 0x00;
	//abilita smbus e uart ( per spostare i pin)
	XBR0 |= 0x05;
	//crossbar, gestisce i pin
	XBR1 = 0x00;
	XBR2 = 0x40;
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
	AA = 1;
}

void timer3_init()
{
	TMR3H = 0xbe;
	TMR3L = 0xe6;
	//abilita iinterrupt timer3
	EIE2 |= 0x01;
	//fa partire il timer 3
	TMR3CN |= 0x04;
}

void mma_initialize()
{
	//altrimenti non entra nell'interrupt
	flag_mma = 1;
	STA = 1;
	while(!mma_init_finished);
	mma_init_finished = 2;
	flag_mma = 0;
	smBusy = 0;
	//corrisponde a nessuna azione da fare su smbus
	interrupt_type = 99;
}

void timer3() interrupt 14
{
	t3_overflow_display ++;
	t3_overflow_temp ++;
	//ogni volta che va in overflow (100ms)
	flag_mma = 1;
	if(t3_overflow_display == 3)
	{
		flag_display = 1;
		t3_overflow_display = 0;
	}
	
	if(t3_overflow_temp == 10)
	{
		flag_temp = 1;
		t3_overflow_temp = 0;
	}
	
	TMR3H = 0xbe;
	TMR3L = 0xe6;
	//resetta flag overflow
	TMR3CN &= 0x7f;
}
/*
																	**********************************************************
																	***********************GESTIONE PWM***********************
																	**********************************************************
*/

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

//quando il timer va in iinterrupt sono passati 200 ms, incrementa di poco la luminosità del backlight
void timer2() interrupt 5
{
	
	lumi = lumi + lumiStep;
	if(lumi == 255)
		lumiStep = -1;
	else if (lumi == 0)
		lumiStep = 1;
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

//immma_posta il necessario per il programma
void pwm_setup()
{
	//timer 0 a 8 bbit, timer 1 a 16 bbit
	TMOD = 0x10;
	//clock intero per il timer0
	CKCON = 0x8;
	//luminosità iniziale del backlight, 0 = luminosità massima
	lumi = 0;
	//abilita gli iinterrupt di timer0, timer1 e timer2
	ET0 = 1;
	ET1 = 1;
	ET2 = 1;
	//immma_posta il valore iniziale del timer 1 (conta 200 ms)
	TH1 = 0x7d;
	TL1 = 0xcb;
	//fa partire il timer 0 che gestisce pwm
	TR0 = 1;
}
 
//iinterrupt del timer0
void timer0() interrupt 1
{
	//il backlight lampeggia solo se la variabile è a 1
	if (acceso == 1)
	{
		if (!backlight) {	
			backlight = 1;		// cambia stato backlight
			TH0 = lumi;	
			TF0 = 0;		//pulisce flag interrput
		}
		else {			
			backlight = 0;
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
	//se sto premendo il bottone e quando rilascio non è passato un secondo, deve cambiare lo stato del backlight
	else if (premuto == 1 && unSec == 0)
	{
		//resetto l'interrupt
		EIE2 |= 0x20;
		P3IF &= 0x7f;
		//rimette in falling edge il bottone
		P3IF &= 0x77;
		//cambio stato backlight
		acceso = !acceso;
		backlight = 0;
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
		//immma_posto la luminosità solo se il bottone è acceso
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

/*
																	**********************************************************
																	***********************ACCELEROMETRO**********************
																	**********************************************************
*/

void accelerometer_interrupt()
{
	
	if(mma_pos == sizeof(mma_init))
			{
				mma_pos = 0;
				mma_init_finished = 1;
				STO = 1;
				//STA = 1;
			}
			switch(SMB0STA)
			{
				case SMB_START:
					SMB0DAT = MMA_WRITE;
					STA = 0;
					//smBusy = 1;
					break;
				
				//gli devo dare indirizzo di lettura
				case SMB_RESTART:
					SMB0DAT = MMA_READ;
					STA = 0;
					break;

				case SMB_FIRSTWRITE:
				case SMB_WRITE:
					if(mma_init_finished == 0)
					{
						SMB0DAT = mma_init[mma_pos];
						mma_pos++;
					}
					else if(mma_init_finished == 2)
					{
						if(!mma_read_ready)
						{
							SMB0DAT = XOUT;
							//dice che mma è pronto a leggere
							mma_read_ready = 1;
						}
						else
							STA = 1;
					}
					break;
					
				case SMB_FIRSTREAD:
					STA = 0;
					break;
				
				case SMB_READ:
					mma_value_read = SMB0DAT;
					mma_value_read &= 00111111;
					xyz[xyz_mma_pos] = TILT_XY[mma_value_read];
					if(xyz_mma_pos == 1)
						AA = 0;
				
					xyz_mma_pos++;
					break;
				
				case SMB_READ_NACK:
					mma_value_read &= 00111111;
					xyz[xyz_mma_pos] = TILT_Z[mma_value_read];
					STO = 1;
					AA = 1;
					//reset flag e variabili
					smBusy = 0;
					flag_mma = 0;
					mma_read_ready = 0;
					xyz_mma_pos = 0;
				
					buffer_x[buffer_pos] = xyz[0];
					buffer_y[buffer_pos] = xyz[1];
					buffer_z[buffer_pos] = xyz[2];
				
					if(buffer_pos == 7)
						buffer_pos = 0;
					else
						buffer_pos ++;
					
			}
			SI = 0;
}

void display_interrupt()
{
	
	switch(SMB0STA)
	{
		//primo start
		case SMB_START:
			//smBusy = 1;
			SMB0DAT = DISPLAY_WRITE; // carica indirizzo slave display
			STA = 0;
			break;
		
		case SMB_FIRSTWRITE:
		case SMB_WRITE:
			if(display_init == 0)
			{
				SMB0DAT = display_init_values[display_init_pos];
				display_init_pos++;
			}
			//scritture successive all'init
		 else if (display_init == 2)
			{
				SMB0DAT = display_values[cont];
				cont++;
				if(cont == sizeof(display_values))
				{
					STO = 1;
					smBusy = 0;
					flag_display = 0;
					cont = 0;
				}
					
			}
			break;
	}
	
	SI = 0;

	if (display_init == 1)
	{
		display_init = 2;
		smBusy = 0;
	}
		//gli serve un altro giro per fermarsi
	else if (display_init_pos == sizeof(display_init_values))
	{
		display_init_pos = 0;
		display_init = 1;
		STO = 1;
		flag_display = 0;
	}
	
}

void temp_interrupt()
{
	switch(SMB0STA)
	{
		case SMB_START:
			SMB0DAT = TEMP_READ;
			STA = 0;
			break;
		case SMB_FIRSTREAD:
			STA = 0;
			break;
		case SMB_READ:
			if(readH == 0)
			{
				tempH = SMB0DAT;
				readH = 1;
			}
			else
			{
				//calcola la parte bassa della temperatura
				tempL = SMB0DAT;
				temp_int = (tempH << 8 | tempL);
				//calcola la temperatura reale
				temp_float = (float)( temp_int >> 3 ) / 16;
				//per scrivere sul display estraggo decine e unità
				decine = (int)temp_float / 10 + 48;
				unita = (int)temp_float % 10 + 48;
				
				display_values[5] = (char)decine;
				display_values[6] = (char)unita;
				
				STO = 1;
				smBusy = 0;
				readH = 0;
				flag_temp = 0;
			}
			break;
	}
	
	SI = 0;
}

void smBus() interrupt 7
{
	if(interrupt_type == 0)
		accelerometer_interrupt();
	else if (interrupt_type == 1)
		display_interrupt();
	else if (interrupt_type == 2)
		temp_interrupt();
}

void average_xyz()
{
	for(avg_cont = 0; avg_cont < sizeof(buffer_x); avg_cont++)
	{
		avg_x += buffer_x[avg_cont];
		avg_y += buffer_y[avg_cont];
		avg_z += buffer_z[avg_cont];
	}
	avg_x /= sizeof(buffer_x);
	avg_y /= sizeof(buffer_y);
	avg_z /= sizeof(buffer_z);
	
	display_values[12] = (char)(avg_x / 10 + 48);
	display_values[13] = (char)(avg_x % 10 + 48);
	
	display_values[16] = (char)(avg_y / 10 + 48);
	display_values[17] = (char)(avg_y % 10 + 48);
	
	display_values[20] = (char)(avg_z / 10 + 48);
	display_values[21] = (char)(avg_z % 10 + 48);

}

void main()
{
	init();
	pwm_setup();
	mma_initialize();
	timer3_init();
	while(1)
	{
		if(flag_mma == 1){
			interrupt_type = 0;
			STA = 1;
			smBusy = 1;
			while(smBusy);
		}
		if (flag_display == 1){
			interrupt_type = 1;
			STA = 1;
			smBusy = 1;
			while(smBusy);
		}
		average_xyz();
		
		if (flag_temp == 1){
			interrupt_type = 2;
			STA = 1;
			smBusy = 1;
			while(smBusy);
		}
		
	}
}