#include "lpc51u68.hpp"
#include <cstdint>
#include <alloca.h>

#define NOP() __asm__("nop" : : :)

/* Red: P0_29
 * Grn: P1_10
 * Blu: P1_9
 * */

/*
 * Port 1 (0:16) can be used for the address line
 * Port 1 (17) can be a strobe of some kind
 * Port 0 will be the data line, although it'll be scrambled.
 * Port 0 can also hold additional strobes.
 *
 * Strobes: OE, CE, PGM
 * */

static constexpr int PinOE { 25 };
static constexpr int PinWR { 26 };
static constexpr int PinCE { 32+17 };

static void SetLed(uint8_t state)
{
	if (state == 0)
		GPIO::CLR0 = 1 << 29;
	else
		GPIO::SET0 = 1 << 29;
}

enum ProgrammingFlags : uint8_t
{
	PF_Verify = 1,
	PF_Retain = 2,
	PF_Dump   = 0xF0
};

void SendToHost(uint8_t value);
void SendState(const char* message);
void SendStringToHost(const char* message);
void SendHexToHost(uint32_t value, int nDigits);
void BeginLatch(void);
void EraseAll(void);
void Wait(void);
uint8_t ReadByte(void);
void Program(void);
void QuickProgram(uint32_t address, uint8_t value, bool preserveSector);
bool Program(uint8_t verificationByte);
void Dump(void);
void BeginProgram(void);

using size_t = std::size_t;

static volatile uint8_t  s_data[256];
static volatile uint32_t s_dataMask[256/32];
static volatile uint8_t  s_rxLog[1024];
static volatile uint16_t s_iRxLog;
static volatile uint32_t s_startAddress;
static volatile uint16_t s_count;
static volatile uint8_t  s_programmingFlags;
static volatile bool     s_startProgramming;
static volatile uint8_t  s_rxState = 0;
static volatile uint16_t s_rxCount = 0;
static volatile uint16_t s_rxIndex = 0;

size_t strlen(const char* str)
{
	const char* p = str;
	while(*p++);
	return p-str;
}

static uint32_t s_address;
void SetAddress(uint32_t address)
{
	s_address = address;
	GPIO::MASK1 = ~0x1FFFF;
	GPIO::MPIN1 = address;
}

void SendState(const char* message)
{
	SendStringToHost(message);
	SendToHost(':');
	SendToHost(' ');
	SendHexToHost(GPIO::PIN0.value(), 8);
	SendToHost(' ');
	SendHexToHost(GPIO::PIN1.value(), 5);
	SendToHost('\n');
}

void LatchByte(uint32_t address, uint8_t value)
{
	/*
	 * tWP:  90ns (WR# held low)
	 * tWPH: 100ns (Minimum time of WR# high after rising edge)
	 * tDS:  50ns (Data valid before rising edge of WR#)
	 * tDH:  0ns (Data valid after rising edge of WR#)
	 * tCH:  0ns (Time between rising edge of WR# and rising edge of CE#)
	 * */
	GPIO::BYTE.Volatile()->PBYTE[PinOE] = 1;
	SetAddress(address);
	GPIO::DIRSET0 = (0x3F << 5) | (3 << 18);
	GPIO::MASK0 = ~((0x3F << 5) | (3 << 18));
	GPIO::MPIN0 = ((value & 0x3F) << 5) | ((value & 0xC0) << (18-6));
	GPIO::BYTE.Volatile()->PBYTE[PinWR] = 0;
	GPIO::BYTE.Volatile()->PBYTE[PinCE] = 0;
	NOP(); NOP(); NOP(); // at 12 MHz, 1 instruction is 83ns
	GPIO::BYTE.Volatile()->PBYTE[PinCE] = 1;
	GPIO::BYTE.Volatile()->PBYTE[PinWR] = 1;
	GPIO::DIRCLR0 = (0x3F << 5) | (3 << 18);
}

void BeginLatch(void)
{
	GPIO::BYTE.Volatile()->PBYTE[PinWR] = 0;
}

void QuickProgram(uint32_t address, uint8_t value, bool preserveSector)
{
	if (preserveSector)
	{
		uint8_t data[256];
		uint32_t tempAddress = address & ~0xFF;
		for (int i = 0; i < 256; ++i)
		{
			SetAddress(tempAddress++);
			data[i] = ReadByte();
		}
		data[address & 0xFF] = value;
		BeginProgram();
		tempAddress = address & ~0xFF;
		for (int i=0; i<256; ++i)
			LatchByte(tempAddress, data[i]);
	}
	else
	{
		BeginProgram();
		LatchByte(address, value);
	}
}

void BeginProgram(void)
{
	LatchByte(0x5555, 0xAA);
	LatchByte(0x2AAA, 0x55);
	LatchByte(0x5555, 0xA0);
	for (volatile uint32_t timeout = 20; timeout != 0; --timeout) ;
}

void Program(void)
{
	// literally just wait 150usec I guess
	for (volatile uint32_t timeout = 500; timeout != 0; --timeout) ;
	Wait();
}

bool Program(uint8_t verificationByte)
{
	for (volatile uint32_t timeout = 1000; timeout != 0; --timeout) ;
	for (int attempts = 50; attempts != 0; --attempts)
	{
		if (ReadByte() == verificationByte)
			return true;
		for (volatile uint32_t timeout = 60; timeout != 0; --timeout) ;
	}
	return false;
}


uint8_t ReadByte(void)
{
	GPIO::BYTE.Volatile()->PBYTE[PinCE] = 0;
	GPIO::BYTE.Volatile()->PBYTE[PinOE] = 0;
	GPIO::DIRCLR0.Volatile() = (0x3F << 5) | (3 << 18);
	uint8_t value = ((GPIO::PIN0.Volatile().value() >> 5) & 0x3F) | ((GPIO::PIN0.Volatile().value() >> (18-6)) & 0xC0);
	GPIO::BYTE.Volatile()->PBYTE[PinOE] = 1;
	GPIO::BYTE.Volatile()->PBYTE[PinCE] = 1;
	return value;
}

void SendToHost(uint8_t value)
{
	while (!USART0::FIFOSTAT.Volatile()->TXNOTFULL) ;
	USART0::FIFOWR.Volatile() = value;
}

void SendStringToHost(const char* message)
{
	size_t len = strlen(message);
	for (size_t i = 0; i < len; ++i)
		SendToHost(message[i]);
}

void SendHexToHost(uint32_t value, int nDigits)
{
	for (int iDigit = 0; iDigit < nDigits; ++iDigit)
	{
		uint8_t digit = (value >> ((nDigits - iDigit - 1) * 4)) & 0xF;
		SendToHost(digit <= 9 ? (digit + '0') : (digit - 0xA + 'A'));
	}
}

int main(void)
{
	//SYSCON::MAINCLKSELA = 3; // fro_hf 96MHz
	//SYSCON::MAINCLKSELB = 0; // MAINCLKSELA
	SYSCON::AHBCLKCTRLSET0 = decltype(*SYSCON::AHBCLKCTRLSET0) {
		.IOCON = 1,
		.GPIO0 = 1,
		.GPIO1 = 1,
		};
	SYSCON::AHBCLKCTRLSET1 = decltype(*SYSCON::AHBCLKCTRLSET1) { .FLEXCOMM0 = 1 };
	SYSCON::FCLKSEL0 = 0; // fro_12m
	//SYSCON::FCLKSEL0 = 1; // fro_hf (96MHz)

	s_iRxLog = 0;
	s_startProgramming = 0;
	s_programmingFlags = 0;
	s_rxState = 0;
	s_count = 0;
	s_rxState = 0;
	s_rxIndex = 0;

	Flexcomm0::PSELID->PERSEL = 1; // USART
	USART0::CFG.Volatile()->ENABLE = 1;
	USART0::CFG.Volatile()->DATALEN = 1;
	// BRGVAL = ((FCLK / (OSRVAL+1)) / baud) - 1
	//      L = ((12e6 / 13) / 38400) - 1 ≈ 24.03846154
	// baud = (FCLK / (OSRVAL+1)) / (BRGVAL+1)
	//      =  (12e6 / 13) / 24 ≈ 38461.53846
	USART0::BRG = 23; //38400 (38462, 0.16% error)
	USART0::OSR = 12;
	//USART0::BRG = 51; //115200 (115384, 0.16% error)
	USART0::FIFOCFG->ENABLETX = 1;
	USART0::FIFOCFG->ENABLERX = 1;

	IOCON::PORT0->Pins[0].FUNC = 1;
	IOCON::PORT0->Pins[1].FUNC = 1;

	// Set all data lines to use a pull-down instead of a pull-up
	IOCON::PORT0->Pins[5].MODE = 1;
	IOCON::PORT0->Pins[6].MODE = 1;
	IOCON::PORT0->Pins[7].MODE = 1;
	IOCON::PORT0->Pins[8].MODE = 1;
	IOCON::PORT0->Pins[9].MODE = 1;
	IOCON::PORT0->Pins[10].MODE = 1;
	IOCON::PORT0->Pins[18].MODE = 1;
	IOCON::PORT0->Pins[19].MODE = 1;

	// Set all address lines to use GPIO
	for (int n = 0; n <= 16; ++n)
	{
		IOCON::PORT1->Pins[n].FUNC = 0; // GPIO function
		IOCON::PORT1->Pins[n].MODE = 1; // pull-down (default LOW)
	}

	GPIO::BYTE->PBYTE[PinOE] = 1;
	GPIO::BYTE->PBYTE[PinCE] = 1;
	GPIO::BYTE->PBYTE[PinWR] = 1;

	(&GPIO::DIRSET0)[PinOE/32] = 1<<(PinOE%32);
	(&GPIO::DIRSET0)[PinCE/32] = 1<<(PinCE%32);
	(&GPIO::DIRSET0)[PinWR/32] = 1<<(PinWR%32);

	GPIO::DIRSET1 = 0x1FFFF;

/* Red: P0_29
 * Grn: P1_10
 * Blu: P1_9
 * */

	GPIO::DIRSET0 = (1<<29);
	GPIO::BYTE->PBYTE[29] = 0;
	GPIO::BYTE->PBYTE[10+32] = 1;
	GPIO::BYTE->PBYTE[32+9] = 0;

	uint32_t testAddress = 1;

	//EraseAll();

#if 1
	// Enter Identification mode
	LatchByte(0x5555, 0xAA);
	LatchByte(0x2AAA, 0x55);
	LatchByte(0x5555, 0x90);
	for (volatile uint32_t timeout = 0x10000; timeout != 0; --timeout) ; 
	// wait 10ms

	SetAddress(0);
	uint8_t manufacturer = ReadByte();
	//SendStringToHost("\nManufacturer ");
	//SendHexToHost(manufacturer, 2);
	//SendToHost('\n');

	SetAddress(1);
	uint8_t device = ReadByte();
	//SendStringToHost("Product ");
	//SendHexToHost(device, 2);
	//SendToHost('\n');

	SetAddress(2);
	uint8_t protection = ReadByte();
	//SendStringToHost("Protection ");
	//SendHexToHost(protection, 2);
	//SendToHost('\n');

	// Leave Identification mode
	LatchByte(0x5555, 0xAA);
	LatchByte(0x2AAA, 0x55);
	LatchByte(0x5555, 0xF0);
	for (volatile uint32_t timeout = 0x10000; timeout != 0; --timeout) ; 
	// wait 10ms

	SetAddress(0);
	uint8_t test0 = ReadByte();
	//SendStringToHost("@ 0: ");
	//SendHexToHost(test0, 2);
	//SendToHost('\n');
	//SendToHost('\n');
#endif

	USART0::FIFOTRIG.Volatile()->RXLVLENA = 1;
	USART0::FIFOTRIG.Volatile()->RXLVL = 0,
	USART0::FIFOINTENSET.Volatile()->RXLVL = 1;
	NVIC_EnableIRQ(Flexcomm0_IRQn);
DoItAllAgain:
	while (!s_startProgramming) ;

	if ((s_programmingFlags & 0xF0) == PF_Dump)
	{
		Dump();
		s_startProgramming = false;
		s_rxState = 0;
		SendToHost(0);
		goto DoItAllAgain;
	}

	if (s_programmingFlags & PF_Retain)
	{
		uint32_t sector = s_startAddress & ~0xFF;
		bool isAlreadyUpToDate = true;
		for (int i=0; i < s_count; ++i)
		{
			if ((s_dataMask[i>>5] & (1 << (i & 31))) == 0)
			{
				SetAddress(sector | i);
				s_data[i] = ReadByte();
			}
			else if (isAlreadyUpToDate)
			{
				SetAddress(sector | i);
				if (ReadByte() != s_data[i])
					isAlreadyUpToDate = false;
			}
		}

		if (isAlreadyUpToDate)
		{
			SendToHost(0);
			s_rxState = 0;
			s_startProgramming = false;
			goto DoItAllAgain;
		}

		s_startAddress = sector;
		s_count = 256;
	}

	uint8_t lastByte = 0;
	for (int attempts = 10; attempts != 0; --attempts)
	{
		BeginProgram();
		const uint32_t sector = 1;
		for (int i=0; i<s_count; ++i)
		{
			uint32_t address = s_startAddress + i;
			LatchByte(address, lastByte = s_data[address & 0xFF]);
		}
		if (Program(lastByte))
			goto Ok;
	}

	s_rxState = 0;
	s_startProgramming = false;

	SendToHost(2);
	SendToHost(lastByte);
	SendToHost(ReadByte());

	goto DoItAllAgain;


Ok:
	if (s_programmingFlags & PF_Verify)
	{
		int nErrors = 0;
		uint8_t* errorOffsets = (uint8_t*) alloca(2 * s_count);
		for (int i=0; i<s_count; ++i)
		{
			uint32_t address = ((s_startAddress + i) & 0xFF) | (s_startAddress & ~0xFF);
			SetAddress(address);
			uint8_t newValue = ReadByte();
			if (newValue != s_data[address & 0xFF])
			{
				errorOffsets[nErrors * 2] = i;
				errorOffsets[nErrors * 2 + 1] = newValue;
				++nErrors;
			}
		}

		if (nErrors > 0)
		{
			SendToHost(1);
			SendToHost(nErrors-1);
			for (int i=0; i < nErrors * 2; ++i)
			{
				SendToHost(errorOffsets[i*2]);
				SendToHost(errorOffsets[i*2+1]);
			}
		}
		else
			SendToHost(0);
	}
	else
		SendToHost(0);

	s_rxState = 0;
	s_startProgramming = false;

	goto DoItAllAgain;

	return 0;
}

void Dump(void)
{
	for (int i=0; i<s_count; ++i)
	{
		SetAddress(s_startAddress + i);
		uint8_t value = ReadByte();
		//SendHexToHost(s_address, 6);
		//SendToHost(' ');
		//SendToHost('=');
		//SendToHost(' ');
		//SendHexToHost(value, 2);
		//SendToHost('\n');
		SendToHost(value);
	}
}

void EraseAll(void)
{
	LatchByte(0x5555, 0xAA);
	LatchByte(0x2AAA, 0x55);
	LatchByte(0x5555, 0x80);
	LatchByte(0x5555, 0xAA);
	LatchByte(0x2AAA, 0x55);
	LatchByte(0x5555, 0x10);

	for (volatile uint32_t timeout = 0x40000; timeout != 0; --timeout) ;
	Wait();
}

void Wait(void)
{
	uint8_t value = ReadByte();
	uint8_t value2 = ReadByte();
	for (;;)
	{
		if ((value & (1<<6)) == (value2 & (1<<6)))
			break;
	}
}


static inline uint8_t ReadUart(void)
{
	uint8_t value = USART0::FIFORD->RXDATA & 0xFF;
	s_rxLog[s_iRxLog++] = value;
	s_iRxLog &= 1023;
	return value;
}

extern "C" void FLEXCOMM0_IRQHandler(void)
{
	//if (USART0::FIFOSTAT->RXNOTEMPTY)
	{
		uint8_t data = ReadUart();
		switch(s_rxState)
		{
			case 0: // initial state, next byte is address
			{
				s_startAddress = data << 24;

				if (s_startAddress == 0xFF000000) // programming flags
					s_rxState = 7;
				else
					++s_rxState;
			} break;
			case 1: // partial reception of address
			case 2:
			case 3:
			{
				s_startAddress |= data << (24 - (8 * s_rxState));
				++s_rxState;
			} break;
			case 4: // count
			{
				s_count = data + 1;
				if ((s_programmingFlags & 0xF0) == PF_Dump)
				{
					s_rxState = 6;
					s_startProgramming = true;
				}
				else
				{
					++s_rxState;
					s_rxIndex = s_startAddress & 0xFF;
					s_rxCount = s_count;
					for (int i=0; i < (256/32); ++i)
						s_dataMask[i] = 0;
				}
			} break;
			case 5: // data
			{
				s_data[s_rxIndex] = data;
				s_dataMask[s_rxIndex >> 5] |= 1 << (s_rxIndex & 31);
				++s_rxIndex;
				s_rxIndex &= 0xFF;
				if ((--s_rxCount) == 0)
				{
					s_startProgramming = true;
					++s_rxState;
				}
			} break;
			case 7: // programming flags
			{
				// b0: Verify
				// b1: Keep unspecified bytes
				s_programmingFlags = data;
				s_rxState = 0;
				SetLed((s_programmingFlags & 0xF0) != 0);
			} break;
			default:
			{
				//USART0::FIFOCFG->EMPTYRX = 1;
				//(void) data; // read and discard
				//s_iRxLog = (s_iRxLog - 1) & 1023;
			} break;
		}

	}
}
