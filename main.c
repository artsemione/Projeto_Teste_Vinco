
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

#define CMD0 		(0x40+0) 	// GO_IDLE_STATE
#define CMD1 		(0x40+1) 	// SEND_OP_COND (MMC)
#define ACMD41 	(0xC0+41) // SEND_OP_COND (SDC)
#define CMD8 		(0x40+8) 	// SEND_IF_COND
#define CMD9 		(0x40+9) 	// SEND_CSD
#define CMD16 	(0x40+16) // SET_BLOCKLEN
#define CMD17 	(0x40+17) // READ_SINGLE_BLOCK
#define CMD24 	(0x40+24) // WRITE_BLOCK
#define CMD55 	(0x40+55) // APP_CMD
#define CMD58 	(0x40+58) // READ_OCR

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "SD.h"
#include "ff_gen_drv.h"
#include "user_diskio.h"
#include "stdio.h"
#include "math.h"
#include "stdbool.h"


#define CS_SD_LOW() 			HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_RESET)
#define CS_SD_HIGH() 			HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_SET)


#define HX_SCK_LOW() 			HAL_GPIO_WritePin(HX_SCK_GPIO_Port, HX_SCK_Pin, GPIO_PIN_RESET)
#define HX_SCK_HIGH()			HAL_GPIO_WritePin(HX_SCK_GPIO_Port, HX_SCK_Pin, GPIO_PIN_SET)


#define HX_DT_LOW() 			HAL_GPIO_WritePin(HX_DT_GPIO_Port, HX_DT_Pin, GPIO_PIN_RESET)
#define HX_DT_HIGH() 			HAL_GPIO_WritePin(HX_DT_GPIO_Port, HX_DT_Pin, GPIO_PIN_SET)
#define HX_DT_Read() 			HAL_GPIO_ReadPin(HX_DT_GPIO_Port, HX_DT_Pin)

#define RELE1_LOW() 			HAL_GPIO_WritePin(Rele_Motor_1_GPIO_Port, Rele_Motor_1_Pin, GPIO_PIN_RESET)
#define RELE1_HIGH() 			HAL_GPIO_WritePin(Rele_Motor_1_GPIO_Port, Rele_Motor_1_Pin, GPIO_PIN_SET)

#define RELE2_LOW() 			HAL_GPIO_WritePin(Rele_Motor_2_GPIO_Port, Rele_Motor_2_Pin, GPIO_PIN_RESET)
#define RELE2_HIGH() 			HAL_GPIO_WritePin(Rele_Motor_2_GPIO_Port, Rele_Motor_2_Pin, GPIO_PIN_SET)

#define FimDeCurso_Up_Read() 			HAL_GPIO_ReadPin(FC_IT_0_GPIO_Port, FC_IT_0_Pin)
#define FimDeCurso_Down_Read() 		HAL_GPIO_ReadPin(FC_DOWN_IT_GPIO_Port, FC_DOWN_IT_Pin)

//#define MEA_READY_HIGH() 	HAL_GPIO_WritePin(MEA_READY_GPIO_Port, MEA_READY_Pin, GPIO_PIN_SET)
//#define MEA_READY_LOW() 	HAL_GPIO_WritePin(MEA_READY_GPIO_Port, MEA_READY_Pin, GPIO_PIN_RESET)



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

RTC_DateTypeDef sDate;
RTC_TimeTypeDef sTime;

extern int 	i;
bool 				DoneGRF = false;
bool 				DoNotIT = false;

uint8_t				estado = 0;
uint8_t				teste = 0;
uint8_t 			result;
char 					taux1[8];
char 					taux[3];
unsigned long	sensor[20] = {0};

// ---------------------------------- UART_RX_TX -----------------------------------

bool 			UART_RX_FLAG = false;
int				n = 0;
int 			Recebendo = 0;
uint8_t 	UART_H1[5];
uint8_t 	UART_RX[1];
uint8_t		UART_LAST_RX[1];
uint8_t		UART_LOTE[6];

uint8_t 	UART3_Buffer[7] = {0};		// Recebe os 7 bytes da IHM

char 			UART_MEA_SEND[8] = {0};
char 			UART_RTC_SEND[15] = {0};
char 			UART_RTC_SEND2[17] = {0};

// ----------------------------------- MEDICAO -------------------------------------

double 	Vetor_MED[100] 					= {0};
double 	Vetor_Ultima_MED[100]		= {0};
double 	Tara 										= 0;
double 	HX_Data									= 0;
double 	Force_Gramas 								= 0;

double	Force_GramasTESTE						= 0;

double 	lastForce_Gramas 						= 0;
double	Maior_Valor						= 0;
int    	Y_Axis     						= 0;
double 	Vetor_MED_Raw[100]		= {0};

bool		MEA_Stop								= false;

bool		Enviando_Vetor_UART			= true;
bool		Start_MEA								= true;
bool		Lote_Recebido_UART			= false;
bool		FimDeCursoDown					= false;
bool		FimDeCursoUP						= false;


bool		MEA_Abort 							= false;
bool		TaraObtida							= false;
bool		EnviarMedicao						= false;
// ----------------------------------- uSD -------------------------------------

FATFS 			SDFatFs;
FATFS 			*fs;
FIL 				MyFile;
FRESULT     res;
char 				USER_Path[4]; /* logical drive path */

// ----------------------------------- RTC -------------------------------------

signed int	RTC_Hora 			= 12;
signed int	RTC_Min 			= 30;
signed int	segundos 			= 50;
signed int	RTC_Dia 			= 15;
signed int	RTC_Mes 			= 06;
signed int  RTC_Ano 			= 18;

uint8_t 		RTC_Data[5];

bool 				updateRTC;
// ------------------------------- Kalman`s FILTER ------------------------------

float KG = 0 ;
float MEA = 0;

float ERROR_E0 = 20;
float ERROR_E1 = 0;

int		ERROR_MEA = 5;

double E_E0 = 0;					//Estimativa Futura - Primeira previsao = Tara
double E_E1 = 0;					//Estimativa Passada

// ---------------------------------- FIR FILTER --------------------------------

// Hamming Window (11)
//double			FIR_C[] = {0.0145489741947061	,0.0305712499252980	,0.0725451577608892	,0.124486576686153	,0.166541934360414	,0.182612214145080,	0.166541934360414,	0.124486576686153,	0.0725451577608892,	0.0305712499252980	,0.0145489741947061	};

// Triangular (16)
double			FIR_C[] = {0.0610597081690606,	0.0616322647111381	,0.0621255648055849	,0.0625384434486763,	0.0628699247468551	,0.0631192246649620,	0.0632857532350082	,0.0633691162187148,	0.0633691162187148	,0.0632857532350082,	0.0631192246649620	,0.0628699247468551,	0.0625384434486763,	0.0621255648055849	,0.0616322647111381	,0.0610597081690606};



// ----------------------------------- IIR FILTER: ------------------------------
double 			sumA = 0;
double 			sumB = 0;
double 			IIR_B[3] ={0.000238	,0.000476,	0.000238};
double 			IIR_A[3] ={1, -1.9650,	0.9661};	
double 			y[10] ={0};


// ----------------------------------- Enviar Dados LCD ------------------------------


uint8_t Printf_Aux[17] ;
int count1 = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

#ifdef _GNUC_
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* _GNUC_ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE  
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */

HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
			
    return ch;
}


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* FRESULT open_append ----------------------------------------------------------*/

void					FIR_vector(int M, double vector[],double vector_F[],int N){
	
	double				ss[100] = {0};
	unsigned long sum = 0;
	int						count =0;
	int						it    =0;

	for(i=0;i<N;i++){										
		
		ss[i] = vector[i];
		
	}
	
	for(it=0;it<N;it++){
		
			sum = 0;
			
			for(i=0;i<M;i++){
				
				sum = sum + ss[i]*FIR_C[i];	  // 16 valores 
			}
			
			vector_F[count] = sum;
			count++;
			
			for(i=0;i<(N-1);i++){										
				ss[i] = ss[i+1];         
			}	
	}
}

FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
	)
	{
    FRESULT fr;
		FRESULT aux;

    /* Opens an existing file. If not exist, creates a new file. */
		
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        aux = f_lseek(fp, f_size(fp));
        if (aux != FR_OK)
            f_close(fp);
    }
    return fr;
}
	

void 					Update_RTC(void){
	
	HAL_RTC_GetTime(&hrtc,&sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
	RTC_Min = sTime.Minutes;	
	RTC_Hora = sTime.Hours;
	RTC_Dia = sDate.Date;
	RTC_Mes = sDate.Month;
	RTC_Ano =	sDate.Year;
}

void 					Motor_Stop(void){
	
	RELE1_LOW();
	RELE2_LOW();
		
}
	

void 					SD_Backup(unsigned long datHX_Data){
	
	//  Inicializacao uSD	
			
	disk_initialize(SDFatFs.drv);
					
				if(SD_Init() != 0){
					
//					LCD_LIMPA();
//					ENVIA_STRING_LCD("NO SD");
					
				}
				else{					
					if(f_mount(&SDFatFs,(const char*)USER_Path,0)!= FR_OK){
//								LCD_LIMPA();
//								ENVIA_STRING_LCD("OPEN ERROR 1");
					}
					else{
								
						if(open_append(&MyFile,"MEDICAO.txt") != FR_OK){						
//								LCD_LIMPA();
//								ENVIA_STRING_LCD("OPEN ERROR 2");																
					}
							else{														
								Update_RTC();					
								f_printf(&MyFile, "Data: %02d:%02d  %02d/%02d/%02d\n" ,RTC_Hora,RTC_Min,RTC_Dia,RTC_Mes,RTC_Ano);
								f_printf(&MyFile, "LOTE: %c", UART_LOTE[0]);
								f_printf(&MyFile, "%c", UART_LOTE[1]);
								f_printf(&MyFile, "%c", UART_LOTE[2]);
								f_printf(&MyFile, "%c", UART_LOTE[3]);
								f_printf(&MyFile, "%c", UART_LOTE[4]);
								f_printf(&MyFile, "%c \n", UART_LOTE[5]);
								f_printf(&MyFile, "FORCA: %u gramas \n \n",(datHX_Data));			
								f_close(&MyFile);
							 
					}					 
				}
			}							
				FATFS_UnLinkDriver(USER_Path);
	}

unsigned long ReadCount(void){
	
	unsigned long Count;
	unsigned char i;
	
	HX_DT_HIGH(); //ADDO=1;
	HX_SCK_LOW(); //ADSK=0;
	
	Count=0;
  while(HX_DT_Read()); 				//while(ADDO); 
	
	for (i=0;i<24;i++){
		HX_SCK_HIGH(); 						//ADSK=1;
		Count=Count<<1;
		HX_SCK_LOW();							//ADSK=0;
		if(HX_DT_Read()) Count++;		//if(ADDO)
	}
	
	HX_SCK_HIGH();								//ADSK=1;
	Count=Count^0x800000;
	HX_SCK_LOW();								//ADSK=0;
	return(Count);
} 

unsigned long moving_average(int M){
	
	unsigned long sum = 0;

	for(i=M;i>0;i--){
		
		sensor[i] = sensor[i-1];
	}
	
	sensor[0] = ReadCount();
	sum = 0;
	
	for(i=0;i<M;i++){
		
		sum = sum + sensor[i];	
	}
	
	return sum/M;

}


double 				FIR(int M){
	
	unsigned long sum = 0;

	for(i=M;i>0;i--){
		
		sensor[i] = sensor[i-1];
	}
	
	sensor[0] = ReadCount();
	sum = 0;
	
	for(i=0;i<M;i++){
		
		sum = sum + sensor[i]*FIR_C[i];	
	}
	
	return sum;

}

double 				IIR(int M){
	
	
	for(i=M;i>0;i--){
		
		sensor[i] = ReadCount();
	}
	
	sensor[0] = ReadCount();
	
	sumA = 0;
	sumB = 0;
	
	for(i=0;i<M;i++){
		
		sumB = sumB + sensor[i]*IIR_B[i];	
		
	}
	
	for(i=0;i<(M-1);i++){
	
		sumA = sumA + y[i]*IIR_A[i+1];
	}
			
	for(i=0;i<M;i++){
	
		y[i+1] = y[i];
	}	
	
	y[0] = sumB - sumA;
	
	return (sumB - sumA);
	
}

double 				Kalmans(void){
	
	// Valor Medido
	MEA = ReadCount();
	
	// Kalmans Filter Gain
	KG = ERROR_E0/(ERROR_E0 + ERROR_MEA);
		
	// Updating Estimative
	E_E1 = E_E0;
	E_E0 = E_E1 + KG*(MEA - E_E1);
	
	// Updating Error	
	ERROR_E1 = ERROR_E0;	
	ERROR_E0 = (1-KG)*ERROR_E1;
	
	return E_E0;
	
	
}

double 				Get_Tara(void){
	
	long 	auxTara 	= 0;
	long 	auxTara2 	= 0;
	
	for(i=0;i<10;i++){
		
		auxTara = ReadCount();
		auxTara = 0;
		
	}
	
	for(i=0;i<10;i++){
		
		auxTara = ReadCount();
		auxTara2 += auxTara;
		
	}
					
	Tara = auxTara2/10;
	return Tara;
}
double 				Maior_Valor_Vetor(double Aux_Vector[]){

		
	double Maior_Valor_Function =0;
	
	for(i=0;i<100;i++){
			
			if(Aux_Vector[i] > Maior_Valor_Function){
			
				Maior_Valor_Function = Aux_Vector[i];
			}	
						
		}
	
		return Maior_Valor_Function;
}
void 					Obter_Medida(int Peso_Minimo, double Aux_Vector[]){
	
	int m =0;	
	HX_Data = ReadCount();		
	Force_Gramas = fabs((HX_Data - Tara)*(0.00238)*(0.98));
	
	//fabs((HX_Data - Tara)*(0.00238))
		
	
	while(Force_Gramas < Peso_Minimo){	
		
		HX_Data = ReadCount();			
		//Force_Gramas = fabs((HX_Data - Tara)*(0.00238)*(0.9765));	
		Force_Gramas = fabs((HX_Data - Tara)*(0.00238)*(0.9765));	
		if(MEA_Abort){
			break;
		}		
	}
	
	m =0;
	
	while(!MEA_Abort){	
		
		HX_Data = ReadCount();
		lastForce_Gramas = Force_Gramas;
		Force_Gramas = fabs((HX_Data - Tara)*(0.00238)*(0.9765));		
		Aux_Vector[m] = Force_Gramas;															//////// Aux_Vector[m] = Force_Gramas;
		m++;	
		
		if(Force_Gramas < (20)){
			break;
		}		
		else if(m>99){
			break;
		}
	}	
	
	if(!MEA_Abort){
		
		Maior_Valor = Maior_Valor_Vetor(Vetor_MED);	
		for(i=0;i<100;i++){						
			Vetor_Ultima_MED[i] = Vetor_MED[i];
		}
	}
	
	for(i=0;i<100;i++){
		Vetor_MED[i] = 0;
	}	
	
		UART_RX_FLAG = false;
}
void					UART_Received(void){

	
		if(UART3_Buffer[0] == 0x65){	// Botao Pressionado 0x65
			
			if(UART3_Buffer[1] == 0x00){ // Pag Menu1 00 --------------------------------------------/
				
				if(UART3_Buffer[2] == 0x02)
					{estado = 0x02;} // Botao Iniciar 

				if(UART3_Buffer[2] == 0x03)
					{estado = 0x06;} // Gra
			}	
	
			else if(UART3_Buffer[1] == 0x0A ){	// Pag Init3 02 -------------------------------------/
				
				if(UART3_Buffer[2] == 0x07)
					{estado = 0x03;} // Botao Inicar Init3		
				
			}			
		}
		
		else if(UART3_Buffer[6] == 0x30){
			
			UART_LOTE[0] = UART3_Buffer[0];
			UART_LOTE[1] = UART3_Buffer[1];
			UART_LOTE[2] = UART3_Buffer[2];
			UART_LOTE[3] = UART3_Buffer[3];
			UART_LOTE[4] = UART3_Buffer[4];
			UART_LOTE[5] = UART3_Buffer[5];
			
			Lote_Recebido_UART = true;			
		}		

		else if(UART3_Buffer[0] == 0x48){
			
			RTC_Min = UART3_Buffer[2];
			RTC_Hora = UART3_Buffer[1];
			RTC_Dia = UART3_Buffer[3];
			RTC_Mes = UART3_Buffer[4];
			
			uint16_t RTC_Ano_recebido;
			
			RTC_Ano_recebido = (UART3_Buffer[6]<<8) + UART3_Buffer[5];
			RTC_Ano_recebido = RTC_Ano_recebido - 2000;			
			RTC_Ano = (uint8_t)RTC_Ano_recebido;		
			
			sTime.Minutes = RTC_Min;	
			sTime.Hours = RTC_Hora;
			sDate.Date = RTC_Dia;
			sDate.Month = RTC_Mes;
			sDate.Year = RTC_Ano;
			
			HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
			HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);			
		}		

}
	
	
void					ReceberDados_UART(){
	
		if(UART_RX_FLAG){
			UART_Received();
			UART_RX_FLAG = false;
		}


}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
		
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_Delay(800);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	
	estado = 0x01;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
		
		ReceberDados_UART();
				
		switch(estado){
			
			case 0x01:
				{
					MEA_Abort = false;
					DoneGRF	= false;
					Enviando_Vetor_UART = true;				
					break;				
				}
				
			case 0x02:						// Tarar a celula de carga
				{		

					if(Lote_Recebido_UART){								
						Lote_Recebido_UART = false;
					//	estado = 0x03;
					}					
				break;				
				}
				
			case 0x03:						 
				{					
					Tara = Get_Tara();
					estado = 0x04;										
				break;				
				}
				
			case 0x04:	// Sequencia da medida, caso abortada volta para Menu
				{			
									
					Obter_Medida(100, Vetor_MED);
										
					if(MEA_Abort){
						estado = 0x01;
					}
					else{
						estado = 0x05;						//estado = 0x05;
						EnviarMedicao = true;
					}
					
				break;				
			}
				
			case 0x05:						// Enviar medicao e fazer Backup
				{		
					if(EnviarMedicao){
						
						EnviarMedicao = false;
						sprintf(Printf_Aux,"t3.txt=\"%7.2f\"",Maior_Valor);
						printf(Printf_Aux);
						printf("\xFF\xFF\xFF");
						Update_RTC();
  					SD_Backup(Maior_Valor);
												
					}
					break;				
			}
				
				
			case 0x06:						//  Grafico
				{	
					if(!DoneGRF){
					
						HAL_Delay(20);
						DoneGRF	= true;				
						int mm 	= 0;	
						
							__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
							while(Vetor_Ultima_MED[mm]){
					
								Y_Axis = (251 - 0.0402 * Vetor_Ultima_MED[mm]);
								mm++;		
								sprintf(Printf_Aux,"t0.txt=\"%u\"",Y_Axis);	
								printf(Printf_Aux);
								printf("\xFF\xFF\xFF");
								HAL_Delay(100);				
							
							}	
							HAL_Delay(20);
							__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);							
					}
					
					break;				
				}
				
			case 0x07:						
				{					
					// Tarar a celula de carga
				break;				
			}
				
			default:
			{
				estado = 0x01;			
			}		
			
		}
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }	
	
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Comando_Valv_Pin|Rele_Motor_1_Pin|Rele_Motor_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_SD_Pin|HX_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Comando_Valv_Pin Rele_Motor_1_Pin Rele_Motor_2_Pin */
  GPIO_InitStruct.Pin = Comando_Valv_Pin|Rele_Motor_1_Pin|Rele_Motor_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_SD_Pin HX_SCK_Pin */
  GPIO_InitStruct.Pin = CS_SD_Pin|HX_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HX_DT_Pin */
  GPIO_InitStruct.Pin = HX_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HX_DT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
