////////////////////////////////////////////////////////////////////////////////
// FW_VERSION  180701 (yymmdd) - Author: Emilio P.G. Ficara
// Modified to use with STM8TI by TinLethax
// V180701
//  Start of job


////////////////////////////////////////////////////////////////////////////////
// System includes
//
#include <stm8s.h> // DO NOT USE here generic iostm8s.h (register map is different)
#include <delay.h>
#include <stdint.h>
#include <stdbool.h>
////////////////////////////////////////////////////////////////////////////////
// Global constants
//
#define TOP1MS		124 // Tim4 compare top value for 1mS tick

////////////////////////////////////////////////////////////////////////////////
// Global constants
//
volatile uint16_t mil = 0;

////////////////////////////////////////////////////////////////////////////////
// Global variables
//
volatile uint8_t echo = 0; // flag for "RX data is echo of TX"
volatile uint8_t DataFlag = 0;
volatile uint8_t revDat[4] = {0}; //data receive over the UART
////////////////////////////////////////////////////////////////////////////////
// variables for TI stuffs
//
const uint8_t TIP = 5; //PC5 Tip pin
const uint8_t RING = 6; //PC6 Ring pin
const uint8_t TXTIMEOUT = 70; //50ms
const uint8_t RXTIMEOUT = 40; //20ms
uint16_t millis(void);

#define TI_TIP_IS_HIGH   (PC_IDR & 0x10) 
#define TI_RING_IS_HIGH (PC_IDR & 0x20)

#define TI_TIP_IS_LOW   (~PC_IDR & 0x10)
#define TI_RING_IS_LOW  (~PC_IDR & 0x20)

void TiTipLow(){
	PC_ODR &= ~(1 << TIP);   
}

void TiRingLow(){
	PC_ODR &= ~(1 << RING);   
}

void TiTipOutput(){
	PC_DDR |= (1 << TIP);
	PC_CR1 |= (1 << TIP);
}

void TiRingOutput(){
	PC_DDR |= (1 << RING);
	PC_CR1 |= (1 << RING);
}

void TiTipInputHigh(){
	PC_DDR &= ~(1 << TIP);
	PC_CR1 |= (1 << TIP);
}

void TiRingInputHigh(){
	PC_DDR &= ~(1 << RING);
	PC_CR1 |= (1 << RING);
}


//------------------------------------------------------------------------------
// Serial COMM section

void OutCom(unsigned char c)// sending single byte from calc to PC
{
	echo = 1; // set echo flag (TX will be immediatly received on RX)
	while((UART1_SR & 0x80) == 0); // wait for TXE = 1
	UART1_DR = c; // send char
	while(echo); // wait until echo received
}

void prntf(uint8_t *txt){
	while(*txt)
		OutCom(*txt++);
}

//------------------------------------------------------------------------------
// Interrupts section
//
void tim4_millis_irq(void) __interrupt(23)// The millis part, every 1 ms
{
     TIM4_SR = 0x00; // clear irq flag
     mil++;
}

void USART1_RX(void) __interrupt(18) // Deal with the incoming data from PC and soon will transfer to calc
{
uint8_t e;

	e = UART1_SR; // read / clear (and ignore) error flags
	e = 1;
	revDat[0] = UART1_DR; // get the char and clear irq
	if(echo){ // if received data is echo of TX
		echo = 0; // turn off flag and don't save
	}else{
		while(UART1_SR & 0x20)// wait until buffer is empty
		revDat[e++] = UART1_DR;// reading all data from host.
		
		DataFlag = 1;// incoming data is not echo but it's data to send to Ti calc.
	}

}

////////////////////////////////////////////////////////////////////////////////
// Stuff for millis(); and TI stuff
//
uint16_t millis(){
return (uint16_t)mil;
}

void sendByte(uint8_t byte){
    uint16_t currentTime;

    for (uint8_t i = 0; i < 8; ++i){
        bool bit = byte & 0x01;
        byte >>= 1;

        //poll both lines until they are both high, which means we're ready to send a bit
        currentTime = millis();
        while (TI_TIP_IS_LOW || TI_RING_IS_LOW){
            if (millis() - currentTime > TXTIMEOUT){
                return;
            }
        }

        if (bit){
            // send a bit by pulling appropriate line low
            TiRingOutput();
            TiRingLow();

            // wait for opposite line to become low
            currentTime = millis();
            while (TI_TIP_IS_HIGH){
                if (millis() - currentTime > TXTIMEOUT){
                    TiRingInputHigh();// set to input pull up.
                    return;
                }
            }

            // release our line
            TiRingInputHigh();

            // wait for opposite line to become high
            currentTime = millis();
            while (TI_TIP_IS_LOW){
                if (millis() - currentTime > TXTIMEOUT){
                    return;
                }
            }

        }else{
            // send a bit by pulling appropriate line low
            TiTipOutput();
            TiTipLow();

            // wait for opposite line to become low
            currentTime = millis();
            while (TI_RING_IS_HIGH){
                if (millis() - currentTime > TXTIMEOUT){
                    TiTipInputHigh();
                    return;
                }
            }

            // release our line
            TiTipInputHigh();

            // wait for opposite line to become high
            currentTime = millis();
            while (TI_RING_IS_LOW){
                if (millis() - currentTime > TXTIMEOUT){
                    return;
                }
            }

        }
  }
}

bool getByte(uint8_t *byte)
{
   uint16_t currentTime;
    uint8_t result = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {

        currentTime = millis();
        while (TI_RING_IS_HIGH && TI_TIP_IS_HIGH)
        {
            if (millis() - currentTime > RXTIMEOUT)
            {
                return false;
            }
        }
        
        bool bit = TI_RING_IS_LOW;
        result >>= 1;
        
        if (bit)
        {
            //ourLine = TIP;
            //oppositeLine = RING;
            result |= 0x80; //bits are always transmitted LSb first (least significant bit)
            
            // acknowledge a bit by pulling appropriate line low
            TiTipOutput();
            TiTipLow();
            
            //wait for opposite line to become high
            currentTime = millis();
            while (TI_RING_IS_LOW)
            {
                if (millis() - currentTime > RXTIMEOUT)
                {
                    TiTipInputHigh();// Set to input pull-up mode
                    return false;
                }
            }

            // release our line
            TiTipInputHigh();
        }
        else
        {
            //ourLine = RING;
            //oppositeLine = TIP;
            
            // acknowledge a bit by pulling appropriate line low
            TiRingOutput();
            TiRingLow();
            
            //wait for opposite line to become high
            while (TI_TIP_IS_LOW)
            {
                if (millis() - currentTime > RXTIMEOUT)
                {
                    TiRingInputHigh();

                }
            }

            // release our line
            TiRingInputHigh();// set input pull up.
            
        }
    }

    *byte = result;
return true;

}
////////////////////////////////////////////////////////////////////////////////
// Main Program
//


void main(void)
{
	CLK_CKDIVR = 0x00;  // Set the frequency to 16 MHz

	// Check for Download mode pin holding
	while(PB_IDR & (1 << 4));// for holding pin 8 for SWIM
	delay_ms(1000);

	PA_ODR |= 0x08; // 0000.1000 All PA data is 0 except PA3 (UART_TX)
	PA_DDR |= 0x08; // 0000.1000 PA.3 is output
	PA_CR1 |= 0x00; // 0000.0000 PA.3 is open drain

   	 //configure both lines as input and enable pull-up resistors
	//PC_DDR = (0 << TIP) | (0 << RING);
	PC_CR1 |= (1 << TIP) | (1 << RING);
	//PC_CR2 = (0 << TIP) | (0 << RING); 

	// init the uart1 in the single pin half duplex mode on PA3 (pin 5 on SOP8)
	UART1_CR1 = 0x00; // 0000.0000 Enable UART 8bit, no parity
	UART1_CR3 = 0x00; // 0000.0000 1 stop bit
	UART1_CR5 = 0x08; // 0000.1000 no dma, half duplex
	UART1_BRR2 = 0x03; // set 9600 BPS
	UART1_BRR1 = 0x68; //
	UART1_CR2 = 0x2C; // 0010.1100 Enable RX irq, Enable TX and RX

	// Timer4 setup for millis()-likes function.
	TIM4_PSCR = 0x07; // fCK_CNT = fCK_PSC/2**PSCR (2**7=128)
	TIM4_ARR = TOP1MS; // set TOP value (reset cnt) for 1mS
	TIM4_SR = 0x00;
	TIM4_IER |= 0x01;
	TIM4_CR1 |= 0x01;

  	 __asm__("rim");
	//prntf("STM8TILink starting...\n");
	while(1){
		while(DataFlag){
			sendByte(revDat[0]);
			sendByte(revDat[1]);
			sendByte(revDat[2]);
			sendByte(revDat[3]);	
			DataFlag = 0;// clear data receipt flag.
		}

		uint8_t byteFromCalc = 0;
		if (getByte(&byteFromCalc)){
			OutCom(byteFromCalc);
		}
	}
}
