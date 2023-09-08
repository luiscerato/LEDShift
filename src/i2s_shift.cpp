#include "i2s_shift.h"


inline uint16_t* reverseAddress(uint16_t* p)
{
	if (((uint32_t)p) & 0x2)
		return p - 1;
	else
		return p + 1;
}

LEDshift::~LEDshift()
{
	if (LedsData) free(LedsData);
	if (OutputData0) heap_caps_free(OutputData0); //heap_caps_malloc
	if (OutputData1) heap_caps_free(OutputData1); //heap_caps_malloc
	if (dmadesc_a) heap_caps_free(dmadesc_a); //heap_caps_malloc
	if (dmadesc_b) heap_caps_free(dmadesc_b); //heap_caps_malloc
}

LEDshift::LEDshift(int32_t outClock, int32_t regCount, int32_t regWide, PinDef_t* pinOut, int32_t latPos, bool reverseBits)
{
	Init(outClock, regCount, regWide, pinOut, latPos, reverseBits);
}

LEDshift::LEDshift(int32_t outClock, int32_t regCount, int32_t regWide, int32_t latPos, bool reverseBits)
{
	Init(outClock, regCount, regWide, &dftPins, latPos, reverseBits);
}

void LEDshift::Init(int32_t outClock, int32_t regCount, int32_t regWide, const PinDef_t* pinOut, int32_t latPos, bool reverseBits)
{
	Ok = false;
	OutClock = constrain(outClock, 100000L, 15000000L);
	RegCount = constrain(regCount, 1, 8);
	RegWide = constrain(regWide, 4, 32);
	PinOut = pinOut ? pinOut : &dftPins;
	LatPulsePos = constrain(latPos, 0, (regWide - 1));
	ReverseBitsOrder = reverseBits;
	I2Sdev = &I2S1;
	fastMode = true;

	//Iniciar todos los buffers que usan memoria dinámica a null
	LedsData = NULL;
	OutputData0 = NULL;
	OutputData1 = NULL;
	dmadesc_a = NULL;
	dmadesc_b = NULL;
}

int32_t LEDshift::Begin(int32_t pwmBitResolution)
{
	i2s_parallel_config_t cfg;
	uint32_t ColorBit, Bit, Max, buffSize, Index;
	uint32_t memUsed = 0;

	BitDeep = constrain(pwmBitResolution, 4, 12);
	/*
		Modo fast:
		En este modo se prioriza la velocidad de sincronizacion entre los valores que se quieren de salida y los datos resultantes a escribir por el bus.
		Para lograrlo se hace un pequeño buffer de salida, en la cual se guardan los datos en formato PWM BCD. Como en BCD los valores se repiten según el
		peso del bit, lo que se hace es indicar al DMA que ese bloque de memoria lo envíe más veces. De esta manera el buffer de datos es chico y se actualiza
		super rápido (50us para 16 salidas) pero se utiliza mucha memoria para almacenar los descriptores de los buffer. Para 10 bits de color va a haber unos
		1024 descriptores por cada buffer (24kb).

		buffer |aaaa|bbbb|cccc|dddd....
		desc   1x aaaa
			   2x bbbb
			   4x cccc
			   8x dddd

		Modo no fast:
		En este modo se aumenta x4 el tiempo de sincronismo, ya que se debe escribir un buffer mucho más grande, pero la memoria total usada es mucho menor
		(10 veces menos). En este caso el buffer se debe describir completamente, ya que la maniobra que hacía el DMA ahora no está más. Como ventaja, solo se
		crean 2 descriptores por buffer.

		buffer |aaaa|bbbb bbbb|cccc cccc cccc cccc|dddd dddd dddd dddd dddd dddd dddd dddd|
		desc   1x aaaa bbbb bbbb cccc cccc cccc cccc dddd dddd dddd dddd dddd dddd dddd dddd
	*/

	int32_t CantLeds = RegCount * RegWide;
	int32_t bufferSize = RegWide * BitDeep * sizeof(*OutputData0);		//Tamaño del buffer en modo rápido (en bytes)
	if (!fastMode) bufferSize = RegWide * bitSteps[BitDeep] * sizeof(*OutputData0);		//en modo lento
	int32_t outFreq = OutClock / RegWide / bitSteps[BitDeep];							//Frecuencia de PWM generado

	Serial.printf("Iniciando configuracion de modulo I2S paralelo en modo %s...\n", fastMode ? "rapido" : "lento");
	Serial.printf(" Cantidad de registros: %i, ancho de registros: %i\n", RegCount, RegWide);
	Serial.printf(" Mapeo de pines -> CLK: %d, LAT: %d, D0: %d, D1: %d, D2: %d, D3: %d, D4: %d, D5: %d, D6: %d, D7: %d\n",
		PinOut->CLK, PinOut->LAT, PinOut->D0, PinOut->D1, PinOut->D2, PinOut->D3, PinOut->D4, PinOut->D5, PinOut->D6, PinOut->D7);
	Serial.printf(" Frecuencia clock: %i hz -> %i hz\n", OutClock, 50000000L / (50000000L / OutClock));	//Ver i2s_par.c linea 217
	Serial.printf(" Cantidad de leds: %i a %i bits de profundidad de color, frecuencia pwm: %i hz\n", CantLeds, BitDeep, outFreq);


	//Crear array para almacenar los valores de las salidas
	maxWaiting = 1000000L / outFreq;	//Calcula el tiempo máximo en us que se debería esperar para sincronziar el buffer
	buffSize = 8 * RegWide * sizeof(*LedsData);
	LedsData = (int32_t*)malloc(buffSize);	//Reservar memoria para los valores de las salidas
	memUsed += buffSize;
	Serial.printf("Asignando %u bytes para buffer de valores de salida... %s\n", buffSize, LedsData ? "Ok" : "Fallo");
	if (LedsData == NULL) return -1;

	/*
		Crea cada uno de los buffers de salida.
		En estos se almacenan los datos tal cual se envían a los registros. Se usan dos para crear un doble buffer
		y así evitar parpadeos mientras se modifica el buffer.
		Estos van a trabajar en modo ping pong en demanda, solo se hace el salto cuando se necesiten refrescar datos.
	*/
	OutputData0 = (uint16_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_DMA);
	OutputData1 = (uint16_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_DMA);
	memset((void*)OutputData0, 0, bufferSize);		//Limpiar buffer
	Serial.printf("Asignando %u bytes para buffer de salida... %s\n", bufferSize, OutputData0 ? "Ok" : "Fallo");
	if (OutputData0 == NULL) return -1;
	memUsed += bufferSize * 2;

	/*
		Se crea un buffer temporal para describir la forma en la que está organizado el buffer de salida. El tamaño va a depender del modo seleccionado.
		Cada descriptor tiene dos valores, uno que indica la direccion de donde se leen los datos, y el otro el tamaño en bytes.
		Este buffer luego se convierte en una lista enlazada. Estas listas son las que le indican al DMA los pasos de como debe enviar los datos.
		Tienen la misma infdrmación, la dirección de memoria, la cantidad de bytes a enviar y donde está la proxima lista.
		Estas listas son las que hacen la magia, ya que repiten cada bloque de datos las veces necesarias desde la misma posición de memoria.
	*/
	Serial.printf("Preparando descriptores de buffers...\n");
	int32_t descSize = sizeof(i2s_parallel_buffer_desc_t) * (bitSteps[BitDeep]);	//Modo rápido
	if (!fastMode) descSize = sizeof(i2s_parallel_buffer_desc_t) * 2;	//Modo lento

	i2s_parallel_buffer_desc_t* buffDesc;
	buffDesc = (i2s_parallel_buffer_desc_t*)malloc(descSize);
	Serial.printf("Asignando %u bytes para %u descriptores... %s\n", descSize, fastMode ? bitSteps[BitDeep] : 2, buffDesc ? "Ok" : "Fallo");
	if (buffDesc == NULL) return -2;
	memUsed += descSize;

	//Llenar el buffer con el detalle de como se deben enviar los datos. Se hace esto por cada buffer.
	int32_t count_desc;
	for (int buffId = 0; buffId < 2; buffId++) {
		uint16_t* pointer = buffId == 0 ? OutputData0 : OutputData1;

		Index = 0;
		if (fastMode) {
			for (ColorBit = 0; ColorBit < BitDeep; ColorBit++) {
				for (Bit = 0; Bit < bitSteps[ColorBit]; Bit++) {
					buffDesc[Index].memory = (void*)pointer;
					buffDesc[Index].size = RegWide * sizeof(*OutputData0);
					//Serial.printf(" Buffer[%u]: Bit: %u pos: %x, Size: %u bytes\n", Index, Bit, Buffers[Index].memory, Buffers[Index].size);
					Index++;
				}
				pointer += RegWide;
			}
		}
		else {
			buffDesc[Index].memory = (void*)pointer;
			buffDesc[Index].size = bufferSize;	//Se calcula tamaño en pasos anteriores
			Index++;
		}
		buffDesc[Index].memory = NULL;	//Fin del descriptor
		buffDesc[Index].size = 0;

		count_desc = calc_needed_dma_descs_for(buffDesc);
		Serial.printf("Cantidad de linked list necesarias: %u\n", count_desc);

		buffSize = sizeof(lldesc_t) * count_desc;	//Calcular tamaño de cada lista
		memUsed += buffSize;
		if (buffId == 0) {
			dmadesc_a = (lldesc_t*)heap_caps_malloc(buffSize, MALLOC_CAP_DMA);	//Asignar memoria
			Serial.printf("Asignando %u bytes a %u linked list... %s\n", buffSize, count_desc, dmadesc_a ? "Ok" : "Fallo");
			if (dmadesc_a == NULL) return -3;
			fill_dma_desc(dmadesc_a, buffDesc);	//Crear la lista
		}
		else {
			dmadesc_b = (lldesc_t*)heap_caps_malloc(buffSize, MALLOC_CAP_DMA);
			Serial.printf("Asignando %u bytes a %u linked list... %s\n", buffSize, count_desc, dmadesc_b ? "Ok" : "Fallo");
			if (dmadesc_b == NULL) return -3;
			fill_dma_desc(dmadesc_b, buffDesc);	//
		}
	}
	free(buffDesc);	//Liberar el buffer temporal
	memUsed -= descSize;

	for (int i = 0; i < 24; i++) cfg.gpio_bus[i] = -1;	//Iniciar todos los pines como sin uso

	cfg.gpio_bus[0] = PinOut->D0;
	cfg.gpio_bus[1] = PinOut->D1;
	cfg.gpio_bus[2] = PinOut->D2;
	cfg.gpio_bus[3] = PinOut->D3;
	cfg.gpio_bus[4] = PinOut->D4;
	cfg.gpio_bus[5] = PinOut->D5;
	cfg.gpio_bus[6] = PinOut->D6;
	cfg.gpio_bus[7] = PinOut->D7;

	cfg.gpio_bus[8] = PinOut->LAT;
	cfg.gpio_clk = PinOut->CLK;

	cfg.clkspeed_hz = OutClock;
	cfg.bits = I2S_PARALLEL_BITS_16;
	cfg.lldesc_a = dmadesc_a;
	cfg.desccount_a = count_desc;
	cfg.lldesc_b = dmadesc_b;
	cfg.desccount_b = count_desc;

	for (int32_t i = 0; i < CantLeds; i++)	//Iniciar salidas a 0
		LedsData[i] = 0;

	Serial.printf("Memoria consumida: %i bytes\n", memUsed);

	BufferID = 1;	//Inicializar a 1 para que comience en el buffer 0
	uint32_t time = SyncBuffers();
	i2s_parallel_flip_to_buffer(I2Sdev, BufferID);
	Serial.printf("Tiempo de sincronizado: %i us\n", time);

	i2s_parallel_setup_without_malloc(I2Sdev, &cfg);
	return 0;
}


void LEDshift::PrintBuffer()
{
	uint16_t X, Y, * Aux, Bit;
	Aux = OutputData0;
	int32_t bufferSize = RegWide * BitDeep, line;


	//El módulo I2S lee primero los 16 bits de más peso (al revez de como se generan los datos) 
	//por lo que se debe invertir el orden en el que se leen en el buffer

	// for (Bit = 0; Bit < BitDeep; Bit++) {
	// 	Serial.printf("Buffer bit: %u\n", Bit);
	// 	Aux = OutputData0 + Bit * RegWide;
	// 	for (X = 0; X < RegWide; X++) {
	// 		Serial.printf("%p %0.4X\n", Aux, *Aux);
	// 		Aux++;
	// 	}
	// }
	// Serial.printf("\n\n");


	//Encabezado
	Serial.printf("            ");
	for (line = 0; line < BitDeep; line++)
		Serial.printf("Bit % 2d   ", line);
	Serial.println();

	//Bits de datos
	char str[256], * c;
	for (line = 0; line < 9; line++) {	//Cada bit de salida
		if (line >= RegCount && line < 8)
			continue;
		Aux = BufferID ? OutputData1 : OutputData0;

		memset(str, 0, sizeof(str));
		c = str;
		for (Bit = 0; Bit < (RegWide * BitDeep); Bit++) { //Cada posisicón del buffer
			int32_t mask = 0x1 << line;
			*c++ = (*reverseAddress(Aux++)) & mask ? '1' : '0';
			if ((Bit & 0x7) == 7) *c++ = ' ';
		}
		if (line < 8)
			Serial.printf("Reg data %d: %s\n", line, str);
		else
			Serial.printf("Lat pulse:  %s\n", str);
	}

	// for (Bit = 0; Bit < BitDeep; Bit++) {
	// 	Serial.printf("Bit de color : %u\n", Bit);
	// 	for (line = 0; line < 9; line++) {
	// 		int32_t mask = 0x1 << line;
	// 		Aux = OutputData0 + Bit * RegWide;
	// 		if (line != 8)
	// 			Serial.printf("Reg bit %d: ", line);
	// 		else
	// 			Serial.printf("Lat pulse: ");
	// 		for (X = 0; X < RegWide; X++) {
	// 			Serial.print((*Aux++) & mask ? '1' : '0');
	// 		}
	// 		Serial.println();
	// 	}
	// }
	// Serial.printf("\n");

}


uint32_t LEDshift::SyncBuffers()
{
	uint32_t Timing = micros();

	//Esperar que se termine de escribir el buffer anterior para evitar flicker (timeout 2500us)
	uint32_t timeout = micros();
	while (!i2s_parallel_is_previous_buffer_free()) {
		if (micros() - timeout > (maxWaiting * 2)) break;
	};

	uint16_t* p = BufferID ? OutputData0 : OutputData1;	//Seleccionar el buffer libre

	//Crear el buffer de salida iterando por los bits de profundidad
	for (int32_t frame = 0; frame < BitDeep; frame++) {
		int32_t bitMask = 0x1 << frame;
		//Serial.printf("Frame: %i\n", frame);
		for (int32_t bit = 0; bit < RegWide; bit++) {		//Iterar por cada bit del ancho del registro de desplazamiento
			int32_t out = 0;

			for (int32_t reg = 0; reg < RegCount; reg++) {
				// Orden inverso: LedsData[(RegWide - bit - 1) + reg * RegWide]; orden normal : LedsData[bit + reg * RegWide];
				int32_t index = (ReverseBitsOrder ? bit : (RegWide - bit - 1)) + reg * RegWide;
				int32_t val = LedsData[index];
				int32_t bitVal = 0x1 << reg;
				if (val & bitMask)
					out |= bitVal;
			}
			// if (bit == (RegWide - 1))
			if (bit == LatPulsePos)
				out |= 0x100;

			//El módulo I2S lee primero los 16 bits de más peso (al revez de como se generan los datos) 
			//por lo que se debe invertir el orden en el que se escriben en el buffer
			*reverseAddress(p++) = out;
			//Serial.printf(" Bit: %i -> out: %0.4X\n", bit, out);
		}
	}
	BufferID = BufferID ? 0 : 1;	//Cambiar de buffer
	i2s_parallel_flip_to_buffer(I2Sdev, BufferID);
	return micros() - Timing;
}

uint32_t LEDshift::SyncBuffers2()
{
	uint32_t Timing = micros();

	//Esperar que se termine de escribir el buffer anterior para evitar flicker (timeout 2500us)
	while (!i2s_parallel_is_previous_buffer_free()) {
		if (micros() - Timing > (maxWaiting * 2)) break;
	};

	uint16_t* p = BufferID ? OutputData0 : OutputData1, * pInit;	//Seleccionar el buffer libre
	//Crear el buffer de salida iterando por los bits de profundidad
	for (int32_t frame = 0; frame < BitDeep; frame++) {
		int32_t bitMask = 0x1 << frame;
		pInit = p; //Guardar direccion de inicio del frame

		Serial.printf("frame %d: p=%p, pInit: %p\n", frame, p, pInit);
		for (int32_t bit = 0; bit < RegWide; bit++) {		//Iterar por cada bit del ancho del registro de desplazamiento
			int32_t out = 0;

			for (int32_t reg = 0; reg < RegCount; reg++) {
				int32_t index = (ReverseBitsOrder ? bit : (RegWide - bit - 1)) + reg * RegWide;
				int32_t val = LedsData[index];
				int32_t bitVal = 0x1 << reg;
				if (val & bitMask)
					out |= bitVal;
			}
			if (bit == LatPulsePos)
				out |= 0x100;
			*reverseAddress(p++) = out;	//Guardar dato en buffer
		}
		//En este modo, se deben copiar el mismo bloque de datos varias veces en el buffer
		if (!fastMode) {
			int_fast32_t max = bitSteps[frame];	//Cantidad de veces a copiar: peso del bit menos 1 que ya copiamos
			int_fast32_t size = RegWide * sizeof(*OutputData0);	//Cantidad de bytes a copiar uint16_t son 2 bytes
			for (uint_fast32_t i = 1; i < max; i++) {
				Serial.printf("frame %d: p=%p, pInit: %p\n", frame, p, pInit);
				memcpy(p, pInit, size);	//Copiar el bloque ya procesado 
				p += RegWide;	//este puntero incrementa de a 2 porque apunta a uint16_t
			}
		}
	}
	BufferID = BufferID ? 0 : 1;	//Cambiar de buffer
	i2s_parallel_flip_to_buffer(I2Sdev, BufferID);
	return micros() - Timing;
}



void LEDshift::loop()
{
	//Actualiza solo cuando el buffer de salida se transfirió completamente.
	if (updateOuts && i2s_parallel_is_previous_buffer_free()) {
		updateOuts = 0;
		SyncBuffers2();
	}
}


void LEDshift::setOutput(int32_t out, float Value)
{
	float max = bitSteps[BitDeep] - 1;
	if (out< 0 || out >((RegCount * RegWide) - 1))
		return;
	Value = constrain(Value, 0, 1.0);
	LedsData[out] = max * Value;
	updateOuts++;
}



float LEDshift::getOutput(int32_t out)
{
	float max = bitSteps[BitDeep];
	if (out< 0 || out >((RegCount * RegWide) - 1))
		return 0;
	return (float)LedsData[out] / (float)max;
}










// void Test(LEDshift* src)
// {
// 	int32_t size = src->bitSteps[src->BitDeep] * src->RegWide * sizeof(uint16_t);
// 	uint16_t* buff = (uint16_t*)malloc(size);
// 	if (buff == nullptr)
// 		return;

// 	uint32_t val = 511, start = 255, index = 0, out = 1, end;
// 	uint32_t offset, inc, mask, max = src->bitSteps[src->BitDeep];

// 	offset = out % src->RegWide;
// 	mask = 0x1 << (out / src->RegWide);

// 	uint16_t* p = &buff[offset];
// 	uint32_t time = micros();
// 	end = start + val;
// 	for (index = 0; index < max; index++) {
// 		if (index >= start && index <= (start + val))
// 			*p = (uint32_t)*p | mask;
// 		else
// 			*p = (uint32_t)*p & (~mask);
// 		p += src->RegWide;
// 	}
// 	time = micros() - time;
// 	free(buff);
// 	Serial.printf("Tiempo de proceso: %u us\n", time);
// }

// uint16_t buff[1024 * 8];
// void LEDshift::Sync()
// {
// 	// int32_t size = 1024 * RegWide * sizeof(uint16_t);
// 	// uint16_t* buff = (uint16_t*)malloc(size);
// 	// if (buff == nullptr)
// 	// 	return;

// 	uint16_t* p = &buff[0];	//Reemplazar por buffer dma
// 	uint32_t index, indexEnd = bitSteps[BitDeep], bit, bitEnd = RegWide, reg, regEnd = RegCount;
// 	uint32_t mask, val, led, time = micros();

// 	for (index = 0; index < 1024; index++) {
// 		for (bit = 0; bit < bitEnd; bit++) {
// 			val = bit == 0 ? 0x100 : 0;	//Bit de latch
// 			led = bit;
// 			mask = 0x1;
// 			for (reg = 0; reg < regEnd; reg++) {
// 				if (index > 0 && index < LedsData[led])
// 					val |= mask;
// 				else
// 					val &= (~mask);
// 				mask <<= 1;
// 				led += bitEnd;
// 			}
// 			*p++ = val;
// 		}
// 	}
// 	time = micros() - time;
// 	Serial.printf("Tiempo de proceso LEDshift::Sync: %u us\n", time);


// 	//Imprimir buffer:
// 	char out[64];
// 	p = &buff[0];

// 	for (index = 0; index < 1024; index++) {
// 		for (bit = 0; bit < bitEnd; bit++) {
// 			led = bit;
// 			mask = 0x1;
// 			val = *p++;
// 			for (reg = 0; reg < regEnd; reg++) {
// 				out[led] = (val & mask) ? '*' : ' ';
// 				mask <<= 1;
// 				led += bitEnd;
// 			}
// 			out[led] = 0x0;
// 		}
// 		Serial.printf("0x%05x: %s\n", index, out);
// 	}

// 	// free(buff);
// }

