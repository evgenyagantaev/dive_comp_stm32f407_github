#include <ILI9163.h>

extern volatile uint32_t MSec;

void Delay(uint32_t MS)
{
	HAL_Delay(MS);
}

//RGB222 to RGB565
uint16_t EToS(uint8_t Col)
{
	uint16_t Temp = 0;

	/* 8 bit
	Temp |= (Col&3)<<3;
	Temp |= ((Col>>2)&7)<<8;
	Temp |= (Col>>5)<<13;
	 */

	Temp |= (Col&3)<<3;
	Temp |= ((Col>>2)&3)<<9;
	Temp |= ((Col>>4)&3)<<14;

	return Temp;
}

uint8_t write_byte_in_tft(uint8_t data)
{

	uint8_t data_out;
    uint8_t read_data;

	// wait for spi transmitter readiness
	while ((SPI3->SR & SPI_SR_TXE) == RESET );
	data_out = data;
    SPI3->DR = data_out;
    // wait while a transmission complete
	while ((SPI3->SR & SPI_SR_RXNE) == RESET );
    read_data = SPI3->DR;
	
	return read_data;

	
}

void SB(uint8_t Data, uint8_t DR)
{
	if(DR == Dat) 
    	a0_tft_GPIO_Port->BSRR = (uint32_t)a0_tft_Pin ;	// set
	else 
    	a0_tft_GPIO_Port->BSRR = (uint32_t)(a0_tft_Pin << 16); 	// reset

	// chipselect low
    spi3_cs_tft_GPIO_Port->BSRR = (uint32_t)(spi3_cs_tft_Pin << 16); 	// reset
	write_byte_in_tft(Data);
	// chipselect high
    spi3_cs_tft_GPIO_Port->BSRR = (uint32_t)spi3_cs_tft_Pin ;	// set
}

void SW(uint16_t Data, uint8_t DR)
{
	if(DR == Dat) 
    	a0_tft_GPIO_Port->BSRR = (uint32_t)a0_tft_Pin ;	// set
	else 
    	a0_tft_GPIO_Port->BSRR = (uint32_t)(a0_tft_Pin << 16); 	// reset

	// chipselect low
    spi3_cs_tft_GPIO_Port->BSRR = (uint32_t)(spi3_cs_tft_Pin << 16); 	// reset
	write_byte_in_tft((uint8_t)(Data>>8));
	write_byte_in_tft((uint8_t)Data);
	// chipselect high
    spi3_cs_tft_GPIO_Port->BSRR = (uint32_t)spi3_cs_tft_Pin ;	// set
}

void SetAddr(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2)
{
	SB(0x2A, Reg);
	SB(0x00, Dat);
	SB(X1, Dat);
	SB(0x00, Dat);
	SB(X2, Dat);
	SB(0x2B, Reg);
	SB(0x00, Dat);
	SB(32+Y1, Dat);
	SB(0x00, Dat);
	SB(32+Y2, Dat);

	SB(0x2C, Reg);
}

void SetScrn(Colours8 Colour)
{
	uint16_t XCnt, YCnt;

	SetAddr(0, 0, XPix-1, YPix-1);

	for(XCnt = 0; XCnt<XPix; XCnt++){
		for(YCnt = 0; YCnt<YPix; YCnt++){
			SW(Colour, Dat);
		}
	}
}

void ClrScrn(void)
{
	SetScrn(BKGCol);
}

void WritePix(uint16_t X, uint16_t Y, Colours8 Colour)
{
	SetAddr(X, Y, X, Y);
	//PCol(Colour);
	SW(Colour, Dat);
}

void PCol(Colours8 Colour)
{
	/*
	switch(Colour){
	case Black:
		SW(0x0000, Dat);
		break;
	case Blue:
		SW(0x0010, Dat);
		break;
	case Red:
		SW(0x8000, Dat);
		break;
	case Magenta:
		SW(0x8010, Dat);
		break;
	case Green:
		SW(0x0400, Dat);
		break;
	case Cyan:
		SW(0x0410, Dat);
		break;
	case Yellow:
		SW(0x8400, Dat);
		break;
	case White:
		SW(0x8410, Dat);
		break;
	}
	*/
}

void SleepMode(uint8_t Mode)
{
	if(Mode == Sleep) SB(0x10, Reg);
	else SB(0x11, Reg);
	Delay(120);
}

void InvMode(uint8_t Mode)
{
	if(Mode==0) SB(0x20, Reg);
	else SB(0x21, Reg);
}


void ILI9163Init(void){

	/*
	S.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	S.SPI_CPHA = SPI_CPHA_1Edge;
	S.SPI_CPOL = SPI_CPOL_Low;
	S.SPI_DataSize = SPI_DataSize_8b;
	S.SPI_FirstBit = SPI_FirstBit_MSB;
	S.SPI_Mode = SPI_Mode_Master;
	S.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1, &S);
	SPI_Cmd(SPI1, ENABLE);
	*/

	/*
	SB(0x11, Reg); //Exit sleep
	Delay(20);

	SB(0x26, Reg); //Set default gamma
	SB(0x04, Dat);

	SB(0xB1, Reg); //Set frame rate
	SB(0x0E, Dat);
	SB(0x10, Dat);

	SB(0xC0, Reg); //Set VRH1[4:0]...
	SB(0x08, Dat);
	SB(0x00, Dat);

	SB(0xC1, Reg);
	SB(0x05, Dat);
	SB(0xC5, Reg);
	SB(0x38, Dat);
	SB(0x40, Dat);

	SB(0x3A, Reg);
	SB(0x05, Dat);
	SB(0x36, Reg);
	SB(0x1C, Dat);

	SB(0x2A, Reg);
	SB(0x00, Dat);
	SB(0x00, Dat);
	SB(0x00, Dat);
	SB(0x7F, Dat);
	SB(0x2B, Reg);
	SB(0x00, Dat);
	SB(32, Dat);
	SB(0x00, Dat);
	SB(127+32, Dat);

	SB(0xB4, Reg);
	SB(0x00, Dat);

	SB(0xF2, Reg);
	SB(0x01, Dat);
	SB(0xE0, Reg);
	SB(0x3F, Dat);
	SB(0x22, Dat);
	SB(0x20, Dat);
	SB(0x30, Dat);
	SB(0x29, Dat);
	SB(0x0C, Dat);
	SB(0x4E, Dat);
	SB(0xB7, Dat);
	SB(0x3C, Dat);
	SB(0x19, Dat);
	SB(0x22, Dat);
	SB(0x1E, Dat);
	SB(0x02, Dat);
	SB(0x01, Dat);
	SB(0x00, Dat);
	SB(0xE1, Reg);
	SB(0x00, Dat);
	SB(0x1B, Dat);
	SB(0x1F, Dat);
	SB(0x0F, Dat);
	SB(0x16, Dat);
	SB(0x13, Dat);
	SB(0x31, Dat);
	SB(0x84, Dat);
	SB(0x43, Dat);
	SB(0x06, Dat);
	SB(0x1D, Dat);
	SB(0x21, Dat);
	SB(0x3D, Dat);
	SB(0x3E, Dat);
	SB(0x3F, Dat);

	SB(0x29, Reg);
	SB(0x2C, Reg);
	 */

	SB(0x01, Reg); //Software reset
	SB(0x11, Reg); //Exit Sleep
	Delay(20);

	SB(0x26, Reg); //Set default gamma
	SB(0x04, Dat);

	SB(0xC0, Reg); //Set Power Control 1
	SB(0x1F, Dat);

	SB(0xC1, Reg); //Set Power Control 2
	SB(0x00, Dat);

	SB(0xC2, Reg); //Set Power Control 3
	SB(0x00, Dat);
	SB(0x07, Dat);

	SB(0xC3, Reg); //Set Power Control 4 (Idle mode)
	SB(0x00, Dat);
	SB(0x07, Dat);

	SB(0xC5, Reg); //Set VCom Control 1
	SB(0x24, Dat); // VComH = 3v
	SB(0xC8, Dat); // VComL = 0v

	SB(0x38, Reg); //Idle mode off
	//SB(0x39, Reg); //Enable idle mode

	SB(0x3A, Reg); //Set pixel mode
	SB(0x05, Dat);

	SB(0x36, Reg); //Set Memory access mode
	SB(0x08, Dat);

	SB(0x29, Reg); //Display on

	InvMode(0);
	ClrScrn();
}
