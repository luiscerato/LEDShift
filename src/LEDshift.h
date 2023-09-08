#ifndef __LEDMATRIX_H
#define __LEDMATRIX_H


#include "Arduino.h"
#include "stdint.h"
#include "esp_heap_caps.h"
#include "soc/i2s_struct.h"
#include "i2s_par.h"

//Definiciones generales para la clase
#define LEDshift_Buffer_t			uint16_t	//Tipo de datos del buffer (word)
#define LEDshift_W2B				(sizeof(LEDshift_Buffer_t))	//Tamaño en bytes del tipo de datos del buffer


typedef union {
	int32_t PinArray[10];
	struct {
		int32_t CLK;
		int32_t LAT;
		int32_t D0;
		int32_t D1;
		int32_t D2;
		int32_t D3;
		int32_t D4;
		int32_t D5;
		int32_t D6;
		int32_t D7;
	};
}PinDef_t;

enum {
	LS_4bitsDeep = 4,
	LS_5bitsDeep = 5,
	LS_6bitsDeep = 6,
	LS_7bitsDeep = 7,
	LS_8bitsDeep = 8,
	LS_9bitsDeep = 9,
	LS_10bitsDeep = 10,
	LS_11bitsDeep = 11,
	LS_12bitsDeep = 12
} BitDeep_t;


class LEDshift
{

private:
	const PinDef_t dftPins = { 27, 26, 25, 5, -1, -1 , -1 , -1 , -1 , -1 };
	const uint32_t bitSteps[13] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096 };

public:
	bool Ok = false;
	bool fastMode;			//Indica el modo de funcionamiento, en modo fast usa mucha ram y sinconiza rápido, sino usa poca RAM y sincroniza más lento
	int32_t RegCount;		//Cantidad de pines de datos usados (max 8)
	int32_t RegWide;		//Cantidad de bits de salida de los registros de desplazamiento(8 para 74HC595)
	int32_t BitDeep;		//Bits de resolución del PWM (min 4, max 16)
	int32_t OutClock;		//Frecuencia de reloj de salida de bits a registros
	int32_t OutputsCount;	//Cantidad de salidas 	
	int32_t LatPulsePos;	//Bit en el que se emite el pulso de LATCH
	bool ReverseBitsOrder;	//Invierte el orden de salida de los bits
	uint32_t maxWaiting;	//Tiempo máximo de espera (en us) en la sincronización.

	const PinDef_t* PinOut;			//Pines por los que se envían los datos

	uint16_t* OutputData0;	//Buffer con los datos de salida que se escriben por I2S a los 74hc595
	uint16_t* OutputData1;	//Buffer con los datos de salida que se escriben por I2S a los 74hc595
	int32_t* LedsData;		//Buffer con el valor de cada led

	lldesc_t* dmadesc_a;	//Estas son las listas enlazadas que van a brindar la información el DMA de como enviar los datos de salida.
	lldesc_t* dmadesc_b;

	int32_t BufferID;		//Buffer en uso
	i2s_dev_t* I2Sdev;		//Módulo I2S en uso

	uint32_t updateOuts;		//Indica que hay un cambio en las salidas y se debe refrescar

	LEDshift(int32_t outClock, int32_t regCount, int32_t regWide, PinDef_t* pinOut, int32_t latPos, bool reverseBits);

	LEDshift(int32_t outClock, int32_t regCount, int32_t regWide, int32_t latPos, bool reverseBits);

	~LEDshift();

	void Init(int32_t outClock, int32_t regCount, int32_t regWide, const PinDef_t* pinOut, int32_t, bool reverseBits);

	int32_t Begin(int32_t bitResolution);

	void setClock(uint32_t Clock);

	void setOutput(int32_t out, float Value);

	float getOutput(int32_t out);

	uint32_t SyncBuffers();

	uint32_t SyncBuffers2();

	void loop();

	void Sync();

	void PrintBuffer();

};


void Test(LEDshift* src);



#endif // !__LEDMATRIX_H

