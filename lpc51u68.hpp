#ifndef LPC51U68_HPP
#define LPC51U68_HPP
#include "lpc51u68.h"
#include <assert.h>
#include <type_traits>
namespace std
{
#ifdef EOF
	static constexpr int eof{ EOF };
#undef EOF
#endif
}

#if defined(__GNUC__)
#define DEPRECATE(foo, msg) foo __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
#define DEPRECATE(foo, msg) __declspec(deprecated(msg)) foo
#else
#error This compiler is not supported
#endif

#define PP_CAT(x,y) PP_CAT1(x,y)
#define PP_CAT1(x,y) x##y

namespace detail
{
	struct true_type {};
	struct false_type {};
	template <int test> struct converter : public true_type {};
	template <> struct converter<0> : public false_type {};
}

#define STATIC_WARNING(cond, msg) \
struct PP_CAT(static_warning,__LINE__) { \
  DEPRECATE(void _(::detail::false_type const& ),msg) {}; \
  void _(::detail::true_type const& ) {}; \
  PP_CAT(static_warning,__LINE__)() {_(::detail::converter<(cond)>());} \
}

namespace Internal
{
	template<typename T, int S = sizeof(T)>
	class RegisterW;

	template<typename T, int S = sizeof(T)>
	class RegisterRW;

	template<typename T, int S = sizeof(T)>
	class RegisterR;

	template<typename T, typename U>
	struct alias_cast_t
	{
		union {
			T out;
			U in;
		};
		alias_cast_t(U val) : in(val) {} ;
	};

	template<typename T, typename U>
	static constexpr T alias_cast(U val)
	{
		alias_cast_t<T, U> alias(val);
		return alias.out;
	};

	template<int Byte> struct uintn_t_base {};
	template<> struct uintn_t_base<1> { typedef uint8_t type; };
	template<> struct uintn_t_base<2> { typedef uint16_t type; };
	template<> struct uintn_t_base<4> { typedef uint32_t type; };
	template<int Bytes>
	struct uintn_t
	{
		typedef typename uintn_t_base<((Bytes==1)|(Bytes==2)|(Bytes==4))?Bytes:1>::type type;
		typedef typename uintn_t_base<((Bytes==1)|(Bytes==2)|(Bytes==4))?Bytes:1>::type* pointer;
	};
	template<typename BaseType, bool Condition>
	struct AddVolatileIf
	{ };
	template<typename BaseType>
	struct AddVolatileIf<BaseType, true>
	{
		typedef typename std::add_volatile<BaseType>::type type;
	};
	template<typename BaseType>
	struct AddVolatileIf<BaseType, false>
	{
		typedef BaseType type;
	};
	template<typename T, int S = sizeof(T)>
	class RegisterBase
	{
	public:
		typedef const typename AddVolatileIf<typename uintn_t<S>::type, std::is_volatile<T>::value>::type c;
		typedef typename AddVolatileIf<typename uintn_t<S>::type, std::is_volatile<T>::value>::type t;
		typedef const typename AddVolatileIf<typename uintn_t<S>::pointer, std::is_volatile<T>::value>::type cptr;
		typedef typename AddVolatileIf<typename uintn_t<S>::pointer, std::is_volatile<T>::value>::type ptr;
		typedef const typename AddVolatileIf<typename uintn_t<S>::type, std::is_volatile<T>::value>::type& cref;
		typedef typename AddVolatileIf<typename uintn_t<S>::type, std::is_volatile<T>::value>::type& ref;
	};

	
	template<typename T, int S = sizeof(T), bool P = std::is_fundamental<T>::value>
	struct RegisterAssignment
	{ };
	
	template<typename T, int S>
	struct RegisterAssignment<T, S, true>
	{
		static void Assign(intptr_t addr, const typename uintn_t<S>::type val)
		{
			*(reinterpret_cast<typename uintn_t<S>::type*>(addr)) = val;
		};
	};
	
	template<typename T, int S>
	struct RegisterAssignment<T, S, false>
	{
		static void Assign(intptr_t addr, const typename uintn_t<S>::type val)
		{
			*(reinterpret_cast<typename uintn_t<S>::type*>(addr)) = val;
		};
		static void Assign(intptr_t addr, const T val)
		{
			*(reinterpret_cast<typename uintn_t<S>::type*>(addr)) = alias_cast<const typename uintn_t<S>::type>(val);
		};
	};

	template <typename T, int S>
	class RegisterRW
	{
		intptr_t addr;
		typedef RegisterBase<T, S> Base;
		typedef RegisterRW<typename std::add_volatile<T>::type, S> VolatileRegister;
		typedef RegisterW<T, S> WriteOnlyRegister;
	public:
		typedef T Type;
		constexpr explicit RegisterRW(intptr_t p) : addr{ p } {};
		constexpr operator T* () const { return reinterpret_cast<T*>(addr); };
		constexpr T* operator->() const { return operator T * (); };
		constexpr T* operator&() const { return operator T * (); };
		constexpr VolatileRegister Volatile() const { return VolatileRegister(addr); };
		constexpr uint8_t* pointer() const { return reinterpret_cast<uint8_t*>(addr); };
		constexpr WriteOnlyRegister WriteOnly() const { return WriteOnlyRegister(addr); };
		constexpr typename Base::ref value() const { return *reinterpret_cast<typename Base::ptr>(addr); };
		template<typename U>
		constexpr void operator=(const U val) const
		{
			RegisterAssignment<T, S>::Assign(addr, val);
		};
		constexpr typename Base::t operator|=(typename Base::c value) const { return *reinterpret_cast<typename Base::ptr>(addr) |= value; };
		constexpr typename Base::t operator&=(typename Base::c value) const { return *reinterpret_cast<typename Base::ptr>(addr) &= value; };
		constexpr typename Base::t operator^=(typename Base::c value) const { return *reinterpret_cast<typename Base::ptr>(addr) ^= value; };
		constexpr typename Base::t operator|(typename Base::c val) const { return value() | val; };
		constexpr typename Base::t operator&(typename Base::c val) const { return value() & val; };
		constexpr typename Base::t operator^(typename Base::c val) const { return value() ^ val; };
		constexpr typename Base::t operator~() const { return ~value(); };
	};

	template <typename T, int S>
	class RegisterR
	{
		intptr_t addr;
		typedef RegisterBase<T, S> Base;
		typedef const RegisterR<typename std::add_volatile<T>::type, S> VolatileRegister;
		typedef typename std::add_pointer<typename std::add_const<T>::type>::type Pointer;
	public:
		typedef T Type;
		constexpr explicit RegisterR(intptr_t p) : addr{ p } {}
		constexpr operator Pointer () const { return reinterpret_cast<Pointer>(addr); }
		constexpr Pointer operator->() const { return operator Pointer (); }
		constexpr Pointer operator&() const { return operator Pointer (); }
		constexpr VolatileRegister Volatile() const { return VolatileRegister(addr); };
		constexpr typename Base::cptr pointer() const { return reinterpret_cast<typename Base::cptr>(addr); };
		constexpr typename Base::cref value() const { return *reinterpret_cast<typename Base::cptr>(addr); };
		constexpr void FailWrite() const { assert(("Cannot modify read-only registers.", false)); };
		constexpr void operator=(typename Base::c value) const { FailWrite(); };
		constexpr void operator|=(typename Base::c value) const { FailWrite(); };
		constexpr void operator&=(typename Base::c value) const { FailWrite(); };
		constexpr void operator^=(typename Base::c value) const { FailWrite(); };
		constexpr typename Base::t operator|(const typename Base::c val) const { return value() | val; };
		constexpr typename Base::t operator&(const typename Base::c val) const { return value() & val; };
		constexpr typename Base::t operator^(const typename Base::c val) const { return value() ^ val; };
		constexpr typename Base::t operator~() const { return ~value(); };
	};


	template<typename T, int S>
	class RegisterW
	{
		intptr_t addr;
		typedef RegisterBase<T, S> Base;
		typedef const RegisterW<typename std::add_volatile<T>::type, S> VolatileRegister;
		typedef typename std::add_pointer<T>::type Pointer;
	public:
		typedef T Type;
		constexpr explicit RegisterW(intptr_t p) : addr{ p } {};
		constexpr RegisterW<T, 2> Word(const int offset) const {
			return RegisterW<T, 2>(addr + 2 * offset);
		};
		constexpr RegisterW<T, 1> Byte(const int offset) const {
			return RegisterW<T, 1>(addr + offset);
		};
		constexpr typename Base::t value() const { assert(("Cannot get value of write-only register.", false)); };
		template<typename U>
		constexpr void operator=(const U val) const
		{
			RegisterAssignment<T, S>::Assign(addr, val);
		};
		constexpr void operator|=(typename uintn_t<S>::type val) const { operator=(val); };
		constexpr void operator&=(typename uintn_t<S>::type val) const { operator=(0); };
		constexpr void operator^=(typename uintn_t<S>::type val) const { operator=(val); };
		constexpr T operator*() const { return *reinterpret_cast<Pointer>(addr); };
		constexpr typename Base::t operator|(typename uintn_t<S>::type val) const { return val; };
		constexpr typename Base::t operator&(typename uintn_t<S>::type val) const { return 0; };
		constexpr typename Base::t operator^(typename uintn_t<S>::type val) const { return val; };
		constexpr typename Base::t operator~() const { return ~0; };
		constexpr operator Pointer () const {
			STATIC_WARNING(std::is_volatile<T>::value, "Using pointer method on non-volatile write-only registers is not reccommended. Optimisations (especially -fstore-merging) may cause only the last write to be applied in a sequence of writes. Use register = decltype(*register) { .field=n } notation instead.");
			return reinterpret_cast<Pointer>(addr);
		}
		constexpr Pointer operator->() const { return operator Pointer(); }
		constexpr Pointer operator&() const { return operator Pointer(); }
		constexpr VolatileRegister Volatile() const { return VolatileRegister(addr); };
		constexpr typename Base::ref operator[](const int offset) const { return *reinterpret_cast<typename Base::ptr>(addr + offset); };
	};

	template<typename T, int S = sizeof(T)> using Register = RegisterRW<T, S>;
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#define BIT(Name, Count) unsigned int Name : Count
// Used to ensure that a register is accessed by halfword. Useful for FIFO registers, where the 
// upper halfword is used for control and the lower data halfword initiates the push/pop.
#define BIT16(Name, Count) uint16_t Name : Count
#define RESERVED(Count) unsigned int : Count
#define RESERVED16(Count) uint16_t : Count
#define CONCAT3(first, second, third) first ## second ## third
#define TYPENAME(Name, Prefix) CONCAT3(Name, _, Prefix)
#define REGISTERINTERNAL(Name, Prefix, Base) \
	    static constexpr Internal::Register<Internal::Types::TYPENAME(Prefix, Name)> const Name = \
			Internal::Register<Internal::Types::TYPENAME(Prefix, Name)>((intptr_t)((intptr_t)(Base) + (intptr_t)Internal::Offsets::Prefix::Name))
#define REGISTER(Name) REGISTERINTERNAL(Name, REGISTERPARENT, REGISTERBASE)
#define REGISTERRINTERNAL(Name, Prefix, Base) \
	    static constexpr Internal::RegisterR<Internal::Types::TYPENAME(Prefix, Name)> const Name = \
			Internal::RegisterR<Internal::Types::TYPENAME(Prefix, Name)>((intptr_t)((intptr_t)(Base) + (intptr_t)Internal::Offsets::Prefix::Name))
#define REGISTERR(Name) REGISTERRINTERNAL(Name, REGISTERPARENT, REGISTERBASE)
#define REGISTERWINTERNAL(Name, Prefix, Base) \
	    static constexpr Internal::RegisterW<Internal::Types::TYPENAME(Prefix, Name)> const Name = \
			Internal::RegisterW<Internal::Types::TYPENAME(Prefix, Name)>((intptr_t)((intptr_t)(Base) + (intptr_t)Internal::Offsets::Prefix::Name))
#define REGISTERW(Name) REGISTERWINTERNAL(Name, REGISTERPARENT, REGISTERBASE)
#define REGISTER8INTERNAL(Name, Prefix, Base) \
		static constexpr Internal::Register<uint8_t> const Name = \
			Internal::Register<uint8_t>((intptr_t)((intptr_t)(Base) + (intptr_t)Internal::Offsets::Prefix::Name))
#define REGISTER8(Name) REGISTER8INTERNAL(Name, REGISTERPARENT, REGISTERBASE)
#define REGISTER16INTERNAL(Name, Prefix, Base) \
		static constexpr Internal::Register<uint16_t> const Name = \
			Internal::Register<uint16_t>((intptr_t)((intptr_t)(Base)+(intptr_t)Internal::Offsets::Prefix::Name))
#define REGISTER16(Name) REGISTER16INTERNAL(Name, REGISTERPARENT, REGISTERBASE)
#define REGISTER32INTERNAL(Name, Prefix, Base) \
		static constexpr Internal::Register<uint32_t> const Name = \
			Internal::Register<uint32_t>((intptr_t)((intptr_t)(Base)+(intptr_t)Internal::Offsets::Prefix::Name))
#define REGISTER32(Name) REGISTER32INTERNAL(Name, REGISTERPARENT, REGISTERBASE)
	
namespace Internal
{
	
	enum Bases
	{
		FLASH_BASE = 0x00000000,
		BOOT_BASE = 0x03000000,
		SRAMX_BASE = 0x04000000,
		SRAM0_BASE = 0x20000000,
		APB0_BASE = 0x40000000,
		APB1_BASE = 0x40020000,
		APBA_BASE = 0x40040000,
		AHB_BASE = 0x40080000,
		PPB_BASE = 0xE0000000,

		DMA_BASE = AHB_BASE + 0x1000,
		FSUSB_BASE = AHB_BASE + 0x4000,
		SCT_BASE = AHB_BASE + 0x5000,
		PWM_BASE = AHB_BASE + 0x5000,
		Flexcomm0_BASE = AHB_BASE + 0x6000,
		Flexcomm1_BASE = AHB_BASE + 0x7000,
		Flexcomm2_BASE = AHB_BASE + 0x8000,
		Flexcomm3_BASE = AHB_BASE + 0x9000,
		Flexcomm4_BASE = AHB_BASE + 0xA000,
		GPIO_BASE = AHB_BASE + 0xC000,
		CRC_BASE = AHB_BASE + 0x15000,
		Flexcomm5_BASE = AHB_BASE + 0x16000,
		Flexcomm6_BASE = AHB_BASE + 0x17000,
		Flexcomm7_BASE = AHB_BASE + 0x18000,
		ISP_AP_BASE = AHB_BASE + 0x1C000,
		ADC_BASE = AHB_BASE + 0x21000,

		SYSCON_BASE = APB0_BASE + 0,
		IOCON_BASE = APB0_BASE + 0x1000,
		GINT0_BASE = APB0_BASE + 0x2000,
		GINT1_BASE = APB0_BASE + 0x3000,
		PINT_BASE = APB0_BASE + 0x4000,
		INMUX_BASE = APB0_BASE + 0x5000,
		CTIMER0_BASE = APB0_BASE + 0x8000,
		CTIMER1_BASE = APB0_BASE + 0x9000,
		WDT_BASE = APB0_BASE + 0xC000,
		MRT_BASE = APB0_BASE + 0xD000,
		UTICK_BASE = APB0_BASE + 0xE000,

		RTC_BASE = APB1_BASE + 0xC000,
		FLASHCTL_BASE = APB1_BASE + 0x14000,

		ASYSCON_BASE = APBA_BASE + 0,
		CTIMER3_BASE = APBA_BASE + 0x8000,

		Nvic_BASE = PPB_BASE + 0xE000

	};
	namespace Offsets
	{
		namespace USART
		{
			enum USART
			{
				CFG = 0x000,
				CTL = 0x004,
				STAT = 0x008,
				INTENSET = 0x00C,
				INTENCLR = 0x010,
				BRG = 0x020,
				INTSTAT = 0x024,
				OSR = 0x028,
				ADDR = 0x02C,
				FIFOCFG = 0xE00,
				FIFOSTAT = 0xE04,
				FIFOTRIG = 0xE08,
				FIFOINTENSET = 0xE10,
				FIFOINTENCLR = 0xE14,
				FIFOINTSTAT = 0xE18,
				FIFOWR = 0xE20,
				FIFORD = 0xE30,
				FIFORDNOPOP = 0xE40,
				ID = 0xFFC
			};
		};
		namespace Nvic
		{
			enum Nvic
			{
				ISER0 = 0x100,
				ICER0 = 0x180,
				ISPR0 = 0x200,
				ICPR0 = 0x280,
				IPR0 = 0x400,
				IPR1 = 0x404,
				IPR2 = 0x408,
				IPR3 = 0x40C,
				IPR4 = 0x410,
				IPR5 = 0x414,
				IPR6 = 0x418,
				IPR7 = 0x418,
			};
		};
		namespace SYSCON
		{
			enum SYSCON
			{
				AHBMATPRIO = 0x010,
				SYSTCKCAL = 0x040,
				NMISRC = 0x048,
				ASYNCAPBCTRL = 0x04C,
				PIOPORCAP0 = 0x0C0,
				PIOPORCAP1 = 0x0C4,
				PIORESCAP0 = 0x0D0,
				PIORESCAP1 = 0x0D4,
				PRESETCTRL0 = 0x100,
				PRESETCTRL1 = 0x104,
				PRESETCTRLSET0 = 0x120,
				PRESETCTRLSET1 = 0x124,
				PRESETCTRLCLR0 = 0x140,
				PRESETCTRLCLR1 = 0x144,
				SYSRSTSTAT = 0x1F0,
				AHBCLKCTRL0 = 0x200,
				AHBCLKCTRL1 = 0x204,
				AHBCLKCTRLSET0 = 0x220,
				AHBCLKCTRLSET1 = 0x224,
				AHBCLKCTRLCLR0 = 0x240,
				AHBCLKCTRLCLR1 = 0x244,
				MAINCLKSELA = 0x280,
				MAINCLKSELB = 0x284,
				CLKOUTSELA = 0x288,
				SYSPLLCLKSEL = 0x290,
				ADCCLKSEL = 0x2A4,
				USBCLKSEL = 0x2A8,
				FCLKSEL0 = 0x2B0,
				FCLKSEL1 = 0x2B4,
				FCLKSEL2 = 0x2B8,
				FCLKSEL3 = 0x2BC,
				FCLKSEL4 = 0x2C0,
				FCLKSEL5 = 0x2C4,
				FCLKSEL6 = 0x2C8,
				FCLKSEL7 = 0x2CC,
				MCLKCLKSEL = 0x2E0,
				FRGCLKSEL = 0x2E8,
				SYSTICKCLKDIV = 0x300,
				AHBCLKDIV = 0x380,
				CLKOUTDIV = 0x384,
				ADCCLKDIV = 0x394,
				USBCLKDIV = 0x398,
				FRGCTRL = 0x3A0,
				I2SMCLKDIV = 0x3AC,
				FLASHCFG = 0x400,
				USBCLKCTRL = 0x40C,
				USBCLKSTAT = 0x410,
				FREQMECTRL = 0x418,
				MCLKIO = 0x420,
				FROCTRL = 0x500,
				WDTOSCCTRL = 0x508,
				RTCOSCCTRL = 0x50C,
				SYSPLLCTRL = 0x580,
				SYSPLLSTAT = 0x584,
				SYSPLLNDEC = 0x588,
				SYSPLLPDEC = 0x58C,
				SYSPLLSSCTRL0 = 0x590,
				SYSPLLSSCTRL1 = 0x594,
				PDSLEEPCFG0 = 0x600,
				PDRUNCFG0 = 0x610,
				PDRUNCFGSET0 = 0x620,
				PDRUNCFGCLR0 = 0x630,
				STARTER0 = 0x680,
				STARTERSET0 = 0x6A0,
				STARTERCLR0 = 0x6C0,
				HWWAKE = 0x780,
				JTAGIDCODE = 0xFF4,
				DEVICE_ID0 = 0xFF8,
				DEVICE_ID1 = 0xFFC
			};
		};
		namespace ASYSCON
		{
			enum ASYSCON
			{
				ASYNCPRESETCTRL = 0x000,
				ASYNCPRESETCTRLSET = 0x004,
				ASYNCPRESETCTRLCLR = 0x008,
				ASYNCAPBCLKCTRL = 0x010,
				ASYNCAPBCLKCTRLSET = 0x014,
				ASYNCAPBCLKCTRLCLR = 0x018,
				ASYNCAPBCLKSELA = 0x020
			};
		};
		namespace IOCON
		{
			enum IOCON
			{
				PORT0 = 0x000,
				PORT1 = 0x080
			};
		};
		namespace INMUX
		{
			enum INMUX
			{
				PINTSEL0 = 0x0C0,
				PINTSEL1 = 0x0C4,
				PINTSEL2 = 0x0C8,
				PINTSEL3 = 0x0CC,
				DMA_ITRIG = 0x0E0,
				DMA_OTRIG = 0x160,
				FREQMEAS_REF = 0x180,
				FREQMEAS_TARGET = 0x184

			};
		};
		namespace GPIO
		{
			enum GPIO
			{
				BYTE = 0x000,
				WORD = 0x1000,
				DIR0 = 0x2000,
				DIR1 = 0x2004,
				MASK0 = 0x2080,
				MASK1 = 0x2084,
				PIN0 = 0x2100,
				PIN1 = 0x2104,
				MPIN0 = 0x2180,
				MPIN1 = 0x2184,
				SET0 = 0x2200,
				SET1 = 0x2204,
				CLR0 = 0x2280,
				CLR1 = 0x2284,
				NOT0 = 0x2300,
				NOT1 = 0x2304,
				DIRSET0 = 0x2380,
				DIRSET1 = 0x2384,
				DIRCLR0 = 0x2400,
				DIRCLR1 = 0x2404,
				DIRNOT0 = 0x2480,
				DIRNOT1 = 0x2484
			};
		};
		namespace FLEXCOMM
		{
			enum FLEXCOMM
			{
				PSELID = 0xFF8,
				PID = 0xFFC
			};
		};
		namespace SPI
		{
			enum SPI
			{
				CFG = 0x400,
				DLY = 0x404,
				STAT = 0x408,
				INTENSET = 0x40C,
				INTENCLR = 0x410,
				DIV = 0x424,
				INTSTAT = 0x428,
				FIFOCFG = 0xE00,
				FIFOSTAT = 0xE04,
				FIFOTRIG = 0xE08,
				FIFOINTENSET = 0xE10,
				FIFOINTENCLR = 0xE14,
				FIFOINTSTAT = 0xE18,
				FIFOWR = 0xE20,
				FIFORD = 0xE30,
				FIFORDNOPOP = 0xE40,
				ID = 0xFFC
			};
		};
		namespace USB
		{
			enum USB
			{
				DEVCMDSTAT = 0x000,
				INFO = 0x004,
				EPLISTSTART = 0x008,
				DATABUFSTART = 0x00C,
				LPM = 0x010,
				EPSKIP = 0x014,
				EPINUSE = 0x018,
				EPBUFCFG = 0x01C,
				INTSTAT = 0x020,
				INTEN = 0x024,
				INTSETSTAT = 0x028,
				EPTOGGLE = 0x034
			};
		}
	}

	namespace Types
	{
		struct USART_CFG
		{
			BIT(ENABLE, 1);
			RESERVED(1);
			BIT(DATALEN, 2);
			BIT(PARITYSEL, 2);
			BIT(STOPLEN, 1);
			BIT(MODE32K, 1);
			BIT(LINMODE, 1);
			BIT(CTSEN, 1);
			RESERVED(1);
			BIT(SYNCEN, 1);
			BIT(CLKPOL, 1);
			RESERVED(1);
			BIT(SYNCMST, 1);
			BIT(LOOP, 1);
			RESERVED(2);
			BIT(OETA, 1);
			BIT(AUTOADDR, 1);
			BIT(OESEL, 1);
			BIT(OEPOL, 1);
			BIT(RXPOL, 1);
			BIT(TXPOL, 1);
		};
		struct USART_CTL
		{
			RESERVED(1);
			BIT(TXBRKEN, 1);
			BIT(ADDRDET, 1);
			RESERVED(3);
			BIT(TXDIS, 1);
			RESERVED(1);
			BIT(CC, 1);
			BIT(CLRCCONRX, 1);
			RESERVED(6);
			BIT(AUTOBAUD, 1);
		};
		struct USART_STAT
		{
			RESERVED(1);
			BIT(RXIDLE, 1);
			RESERVED(1);
			BIT(TXIDLE, 1);
			BIT(CTS, 1);
			BIT(DELTACTS, 1);
			BIT(TXDISSTAT, 1);
			RESERVED(3);
			BIT(RXBRK, 1);
			BIT(DELTARXBRK, 1);
			BIT(START, 1);
			BIT(FRAMERRINT, 1);
			BIT(PARITYERRINT, 1);
			BIT(RXNOISEINT, 1);
			BIT(ABERR, 1);
		};
		struct USART_INTENSET
		{
			RESERVED(3);
			BIT(TXIDLEEN, 1);
			RESERVED(1);
			BIT(DELTACTSEN, 1);
			BIT(TXDISEN, 1);
			RESERVED(4);
			BIT(DELTARXBRKEN, 1);
			BIT(STARTEN, 1);
			BIT(FRAMERREN, 1);
			BIT(PARITYERREN, 1);
			BIT(RXNOISEEN, 1);
			BIT(ABERREN, 1);
		};
		struct USART_INTENCLR
		{
			RESERVED(3);
			BIT(TXIDLECLR, 1);
			RESERVED(1);
			BIT(DELTACTSCLR, 1);
			BIT(TXDISCLR, 1);
			RESERVED(4);
			BIT(DELTARXBRKCLR, 1);
			BIT(STARTCLR, 1);
			BIT(FRAMERRCLR, 1);
			BIT(PARITYERRCLR, 1);
			BIT(RXNOISECLR, 1);
			BIT(ABERRCLR, 1);
		};
		struct USART_INTSTAT
		{
			RESERVED(3);
			BIT(TXIDLE, 1);
			RESERVED(1);
			BIT(DELTACTS, 1);
			BIT(TXDIS, 1);
			RESERVED(4);
			BIT(DELTARXBRK, 1);
			BIT(START, 1);
			BIT(FRAMERRINT, 1);
			BIT(PARITYERRINT, 1);
			BIT(RXNOISEINT, 1);
			BIT(ABERRINT, 1);
		};
		struct USART_OSR
		{
			BIT(OSRVAL, 4);
		};
		struct USART_FIFOCFG
		{
			BIT(ENABLETX, 1);
			BIT(ENABLERX, 1);
			RESERVED(2);
			BIT(SIZE, 2);
			RESERVED(6);
			BIT(DMATX, 1);
			BIT(DMARX, 1);
			BIT(WAKETX, 1);
			BIT(WAKERX, 1);
			BIT(EMPTYTX, 1);
			BIT(EMPTYRX, 1);
		};
		struct USART_FIFOSTAT
		{
			BIT(TXERR, 1);
			BIT(RXERR, 1);
			RESERVED(1);
			BIT(PERINT, 1);
			BIT(TXEMPTY, 1);
			BIT(TXNOTFULL, 1);
			BIT(RXNOTEMPTY, 1);
			BIT(RXFULL, 1);
			BIT(TXLVL, 5);
			RESERVED(3);
			BIT(RXLVL, 5);
		};
		struct USART_FIFOTRIG
		{
			BIT(TXLVLENA, 1);
			BIT(RXLVLENA, 1);
			RESERVED(5);
			BIT(TXLVL, 4);
			RESERVED(4);
			BIT(RXLVL, 4);
		};
		struct USART_FIFOINTENSET
		{
			BIT(TXERR, 1);
			BIT(RXERR, 1);
			BIT(TXLVL, 1);
			BIT(RXLVL, 1);
		};
		typedef struct USART_FIFOINTENSET USART_FIFOINTENCLR;
		struct USART_FIFOINTSTAT
		{
			BIT(TXERR, 1);
			BIT(RXERR, 1);
			BIT(TXLVL, 1);
			BIT(RXLVL, 1);
			BIT(PERINT, 1);
		};
		struct USART_FIFOWR
		{
			BIT(TXDATA, 9);
		};
		struct USART_FIFORD
		{
			BIT16(RXDATA, 9);
			//RESERVED(4);
			BIT(FRAMERR, 1);
			BIT(PARITYERR, 1);
			BIT(RXNOISE, 1);
		};
		typedef struct USART_FIFORD USART_FIFORDNOPOP;
		struct USART_ID
		{
			BIT(APERTURE, 8);
			BIT(MINOR_REV, 4);
			BIT(MAJOR_REV, 4);
			BIT(ID, 16);
		};
		struct PIN
		{
			union
			{
				volatile uint32_t w;
				struct
				{
					BIT(FUNC, 3);
					BIT(MODE, 2);
					RESERVED(1);
					BIT(INVERT, 1);
					BIT(DIGIMODE, 1);
					BIT(FILTEROFF, 1);
					// Not available on pins PIO0_23~26, PIO0_29~31, PIO1_0~8
					BIT(SLEW, 1);
					// Not available on pins PIO0_23~26
					BIT(OD, 1);
				};
				struct
				{
					RESERVED(5);
					// Only available on pins PIO0_23~26
					BIT(I2CSLEW, 1);
					RESERVED(3);
					// Only available on pins PIO0_23~26
					BIT(I2CDRIVE, 1);
					// Only available on pins PIO0_23~26
					BIT(I2CFILTER, 1);
				};
			};

		};
		struct PIN_A
		{
			union
			{
				volatile uint32_t w;
				struct
				{
					BIT(FUNC, 3);
					BIT(MODE, 2);
					RESERVED(1);
					BIT(INVERT, 1);
					BIT(DIGIMODE, 1);
					BIT(FILTEROFF, 1);
					RESERVED(1);
					BIT(OD, 1);
				};
			};
		};
		struct PIN_D
		{
			union
			{
				volatile uint32_t w;
				struct
				{
					BIT(FUNC, 3);
					BIT(MODE, 2);
					RESERVED(1);
					BIT(INVERT, 1);
					BIT(DIGIMODE, 1);
					BIT(FILTEROFF, 1);
					BIT(SLEW, 1);
					BIT(OD, 1);
				};
			};
		};
		struct PIN_I
		{
			union
			{
				volatile uint32_t w;
				struct
				{
					BIT(FUNC, 3);
					RESERVED(2);
					BIT(I2CSLEW, 1);
					BIT(INVERT, 1);
					BIT(DIGIMODE, 1);
					BIT(FILTEROFF, 1);
					BIT(I2CDRIVE, 1);
					BIT(I2CFILTER, 1);
				};
			};
		};
		struct IOCON_PORT0
		{
			union
			{
				volatile PIN Pins[32];
				struct
				{
					volatile PIN_D PIO0_0;
					volatile PIN_D PIO0_1;
					volatile PIN_D PIO0_2;
					volatile PIN_D PIO0_3;
					volatile PIN_D PIO0_4;
					volatile PIN_D PIO0_5;
					volatile PIN_D PIO0_6;
					volatile PIN_D PIO0_7;
					volatile PIN_D PIO0_8;
					volatile PIN_D PIO0_9;
					volatile PIN_D PIO0_10;
					volatile PIN_D PIO0_11;
					volatile PIN_D PIO0_12;
					volatile PIN_D PIO0_13;
					volatile PIN_D PIO0_14;
					volatile PIN_D PIO0_15;
					volatile PIN_D PIO0_16;
					volatile PIN_D PIO0_17;
					volatile PIN_D PIO0_18;
					volatile PIN_D PIO0_19;
					volatile PIN_D PIO0_20;
					volatile PIN_D PIO0_21;
					volatile PIN_D PIO0_22;
					volatile PIN_I PIO0_23;
					volatile PIN_I PIO0_24;
					volatile PIN_I PIO0_25;
					volatile PIN_I PIO0_26;
					volatile PIN_A PIO0_29;
					volatile PIN_A PIO0_30;
					volatile PIN_A PIO0_31;
				};
			};
		};
		struct IOCON_PORT1
		{
			union
			{
				volatile PIN Pins[18];
				struct
				{
					volatile PIN_A PIO1_0;
					volatile PIN_A PIO1_1;
					volatile PIN_A PIO1_2;
					volatile PIN_A PIO1_3;
					volatile PIN_A PIO1_4;
					volatile PIN_A PIO1_5;
					volatile PIN_A PIO1_6;
					volatile PIN_A PIO1_7;
					volatile PIN_A PIO1_8;
					volatile PIN_D PIO1_9;
					volatile PIN_D PIO1_10;
					volatile PIN_D PIO1_11;
					volatile PIN_D PIO1_12;
					volatile PIN_D PIO1_13;
					volatile PIN_D PIO1_14;
					volatile PIN_D PIO1_15;
					volatile PIN_D PIO1_16;
					volatile PIN_D PIO1_17;
				};

			};
		};
		struct DMA_ITRIG
		{
			BIT(INP, 5);
		};
		struct INMUX_DMA_ITRIG
		{
			volatile DMA_ITRIG Channels[18];
		};
		struct DMA_OTRIG
		{
			BIT(INP, 5);
		};
		struct INMUX_DMA_OTRIG
		{
			volatile DMA_OTRIG Outputs[4];
		};
		struct INMUX_FREQMEAS_REF
		{
			BIT(CLKIN, 5);
		};
		typedef INMUX_FREQMEAS_REF INMUX_FREQMEAS_TARGET;
		struct GPIO_BYTE
		{
			volatile uint8_t PBYTE[50];
		};
		struct GPIO_WORD
		{
			volatile uint32_t PWORD[50];
		};
		struct FLEXCOMM_PSELID
		{
			BIT(PERSEL, 3);
			BIT(LOCK, 1);
			BIT(USARTPRESENT, 1);
			BIT(SPIPRESENT, 1);
			BIT(I2CPRESENT, 1);
			BIT(I2SPRESENT, 1);
			RESERVED(4);
			BIT(ID, 20);
		};
		struct FLEXCOMM_PID
		{
			RESERVED(8);
			BIT(Minor_Rev, 4);
			BIT(Major_Rev, 4);
			BIT(ID, 16);
		};
		struct SYSCON_FROCTRL
		{
			RESERVED(14);
			BIT(SEL, 1);
			RESERVED(1);
			BIT(FREQTRIM, 8);
			BIT(USBCLKADJ, 1);
			BIT(USBMODCHG, 1);
			RESERVED(4);
			BIT(HSPDCLK, 1);
		};
		struct SPI_CFG
		{
			BIT(ENABLE, 1);
			RESERVED(1);
			BIT(MASTER, 1);
			BIT(LSBF, 1);
			BIT(CPHA, 1);
			BIT(CPOL, 1);
			RESERVED(1);
			BIT(LOOP, 1);
			BIT(SPOL0, 1);
			BIT(SPOL1, 1);
			BIT(SPOL2, 1);
			BIT(SPOL3, 1);
		};
		struct SPI_DLY
		{
			BIT(PRE_DELAY, 4);
			BIT(POST_DELAY, 4);
			BIT(FRAME_DELAY, 4);
			BIT(TRANSFER_DELAY, 4);
		};
		struct SPI_STAT
		{
			RESERVED(4);
			BIT(SSA, 1);
			BIT(SSD, 1);
			BIT(STALLED, 1);
			BIT(ENDTRANSFER, 1);
			BIT(MSTIDLE, 1);
		};
		struct SPI_INTENSET
		{
			RESERVED(4);
			BIT(SSAEN, 1);
			BIT(SSDEN, 1);
			RESERVED(2);
			BIT(MSTIDLEEN, 1);
		};
		typedef struct SPI_INTENSET SPI_INTENCLR;
		typedef uint16_t SPI_DIV;
		struct SPI_INTSTAT
		{
			RESERVED(4);
			BIT(SSA, 1);
			BIT(SSD, 1);
			RESERVED(2);
			BIT(MSTIDLE, 1);
		};
		struct SPI_FIFOCFG
		{
			BIT(ENABLETX, 1);
			BIT(ENABLERX, 1);
			RESERVED(2);
			BIT(SIZE, 2);
			RESERVED(6);
			BIT(DMATX, 1);
			BIT(DMARX, 1);
			BIT(WAKETX, 1);
			BIT(WAKERX, 1);
			BIT(EMPTYTX, 1);
			BIT(EMPTYRX, 1);
		};
		struct SPI_FIFOSTAT
		{
			BIT(TXERR, 1);
			BIT(RXERR, 1);
			RESERVED(1);
			BIT(PERINT, 1);
			BIT(TXEMPTY, 1);
			BIT(TXNOTFULL, 1);
			BIT(RXNOTEMPTY, 1);
			BIT(RXFULL, 1);
			BIT(TXLVL, 5);
			RESERVED(3);
			BIT(RXLVL, 5);
		};
		struct SPI_FIFOTRIG
		{
			BIT(TXLVLENA, 1);
			BIT(RXLVLENA, 1);
			RESERVED(6);
			BIT(TXLVL, 4);
			RESERVED(4);
			BIT(RXLVL, 4);
		};
		struct SPI_FIFOINTENSET
		{
			BIT(TXERR, 1);
			BIT(RXERR, 1);
			BIT(TXLVL, 1);
			BIT(RXLVL, 1);
		};
		typedef struct SPI_FIFOINTENSET SPI_FIFOINTENCLR;
		struct SPI_FIFOINTSTAT
		{
			BIT(TXERR, 1);
			BIT(RXERR, 1);
			BIT(TXLVL, 1);
			BIT(RXLVL, 1);
			BIT(PERINT, 1);
		};
		struct SPI_FIFOWR
		{
			BIT16(TXDATA, 16);
			BIT16(TXSSEL0_N, 1);
			BIT16(TXSSEL1_N, 1);
			BIT16(TXSSEL2_N, 1);
			BIT16(TXSSEL3_N, 1);
			BIT16(EOT, 1);
			BIT16(EOF, 1);
			BIT16(RXIGNORE, 1);
			RESERVED16(1);
			BIT16(LEN, 4);
		};
		struct SPI_FIFORD
		{
			BIT16(RXDATA, 16);
			BIT16(RXSSEL0_N, 1);
			BIT16(RXSSEL1_N, 1);
			BIT16(RXSSEL2_N, 1);
			BIT16(RXSSEL3_N, 1);
			BIT16(SOT, 1);
		};
		typedef struct SPI_FIFORD SPI_FIFORDNOPOP;
		struct SPI_ID
		{
			BIT(APERTURE, 8);
			BIT(MINOR_REV, 4);
			BIT(MAJOR_REV, 4);
			BIT(ID, 16);
		};
		struct SYSCON_SYSPLLCTRL
		{
			BIT(SELR, 4);
			BIT(SELI, 6);
			BIT(SELP, 5);
			BIT(BYPASS, 1);
			BIT(BYPASSCCODIV2, 1);
			BIT(UPLIMOFF, 1);
			BIT(BANDSEL, 1);
			BIT(DIRECTI, 1);
			BIT(DIRECTO, 1);
		};
		struct SYSCON_SYSPLLSTAT
		{
			BIT(LOCK, 1);
		};
		struct SYSCON_SYSPLLNDEC
		{
			BIT(NDEC, 10);
			BIT(NREQ, 1);
		};
		struct SYSCON_SYSPLLPDEC
		{
			BIT(PDEC, 7);
			BIT(PREQ, 1);
		};
		struct SYSCON_SYSPLLSSCTRL0
		{
			BIT(MDEC, 17);
			BIT(MREQ, 1);
			BIT(SEL_EXT, 1);
		};
		struct SYSCON_SYSPLLSSCTRL1
		{
			BIT(MD, 19);
			BIT(MDREQ, 1);
			BIT(MF, 3);
			BIT(MR, 3);
			BIT(MC, 2);
			BIT(PD, 1);
			BIT(DITHER, 1);
		};
		struct SYSCON_AHBCLKCTRL0
		{
			RESERVED(1);
			BIT(ROM, 1);
			RESERVED(5);
			BIT(FLASH, 1);
			BIT(FMC, 1);
			RESERVED(2);
			BIT(INPUTMUX, 1);
			RESERVED(1);
			BIT(IOCON, 1);
			BIT(GPIO0, 1);
			BIT(GPIO1, 1);
			RESERVED(2);
			BIT(PINT, 1);
			BIT(GINT, 1);
			BIT(DMA ,1);
			BIT(CRC, 1);
			BIT(WWDT, 1);
			BIT(RTC, 1);
			RESERVED(3);
			BIT(ADC0, 1);
		};
		struct SYSCON_AHBCLKCTRL1
		{
			BIT(MRT, 1);
			RESERVED(1);
			BIT(SCT0, 1);
			RESERVED(7);
			BIT(UTICK, 1);
			BIT(FLEXCOMM0, 1);
			BIT(FLEXCOMM1, 1);
			BIT(FLEXCOMM2, 1);
			BIT(FLEXCOMM3, 1);
			BIT(FLEXCOMM4, 1);
			BIT(FLEXCOMM5, 1);
			BIT(FLEXCOMM6, 1);
			BIT(FLEXCOMM7, 1);
			RESERVED(6);
			BIT(USB, 1);
			BIT(CTIMER0, 1);
			BIT(CTIMER1, 1);
		};
		typedef SYSCON_AHBCLKCTRL0 SYSCON_AHBCLKCTRLCLR0, SYSCON_AHBCLKCTRLSET0;
		typedef SYSCON_AHBCLKCTRL1 SYSCON_AHBCLKCTRLCLR1, SYSCON_AHBCLKCTRLSET1;
		typedef uint32_t SYSCON_PIOPORCAP0, SYSCON_PIOPORCAP1, SYSCON_PIORESCAP0, SYSCON_PIORESCAP1, SYSCON_PRESETCTRLSET0,
			SYSCON_PRESETCTRLSET1, SYSCON_PRESETCTRLCLR0, SYSCON_PRESETCTRLCLR1, SYSCON_PDRUNCFGSET0, SYSCON_PDRUNCFGCLR0,
			SYSCON_STARTERSET0, SYSCON_STARTERCLR0, SYSCON_JTAGIDCODE, SYSCON_DEVICE_ID0, SYSCON_DEVICE_ID1;
		typedef uint32_t ASYSCON_ASYNCPRESETCTRLSET, ASYSCON_ASYNCPRESETCTRLCLR, ASYSCON_ASYNCAPBCLKCTRLSET,
			ASYSCON_ASYNCAPBCLKCTRLCLR;
		typedef uint32_t GPIO_SET0, GPIO_SET1, GPIO_CLR0, GPIO_CLR1, GPIO_NOT0, GPIO_NOT1, GPIO_DIRSET0, GPIO_DIRSET1,
			GPIO_DIRCLR0, GPIO_DIRCLR1, GPIO_DIRNOT0, GPIO_DIRNOT1;
		struct USB_DEVCMDSTAT
		{
			BIT(DEV_ADDR, 7);
			BIT(DEV_EN, 1);
			BIT(SETUP, 1);
			BIT(FORCE_NEEDCLK, 1);
			RESERVED(1);
			BIT(LPM_SUP, 1);
			BIT(INTONNAK_AO, 1);
			BIT(INTONNAK_AI, 1);
			BIT(INTONNAK_CO, 1);
			BIT(INTONNAK_CI, 1);
			BIT(DCON, 1);
			BIT(DSUS, 1);
			RESERVED(1);
			BIT(LPM_SUS, 1);
			BIT(LPM_REWP, 1);
			RESERVED(3);
			BIT(DCON_C, 1);
			BIT(DSUS_C, 1);
			BIT(DRES_C, 1);
			RESERVED(1);
			BIT(VBUSDEBOUNCED, 1);
		};
		struct USB_INFO
		{
			BIT(FRAME_NR, 11);
			BIT(ERR_CODE, 4);
		};
		struct USB_EPLISTSTART
		{
			RESERVED(8);
			BIT(EP_LIST, 24);
		};
		struct USB_DATABUFSTART
		{
			RESERVED(22);
			// Start address of the buffer pointer page where all endpoint data buffers are located.
			BIT(DA_BUF, 10);
		};
		struct USB_LPM
		{
			BIT(HIRD_HW, 4);
			BIT(HIRD_SW, 4);
			BIT(DATA_PENDING, 1);
		};
		struct USB_EPSKIP
		{
			BIT(SKIP, 30);
		};
		struct USB_EPINUSE
		{
			RESERVED(2);
			BIT(BUF, 8);
		};
		struct USB_EPBUFCFG
		{
			RESERVED(2);
			BIT(BUF_SB, 8);
		};
		struct USB_INTSTAT
		{
			// Interrupt status register bit for the Control EP0 OUT direction.
			// This bit will be set if NBytes transitions to zero or the skip bit is set by software or a SETUP packet is successfully received for the control EP0.
			// If the IntOnNAK_CO is set, this bit will also be set when a NAK is transmitted for the Control EP0 OUT direction.
			// Software can clear this bit by writing a one to it.
			BIT(EP0OUT, 1);
			// Interrupt status register bit for the Control EP0 IN direction.
			// This bit will be set if NBytes transitions to zero or the skip bit is set by software.
			// If the IntOnNAK_CI is set, this bit will also be set when a NAK is transmitted for the Control EP0 IN direction.
			// Software can clear this bit by writing a one to it.
			BIT(EP0IN, 1);
			BIT(EP1OUT, 1);
			BIT(EP1IN, 1);
			BIT(EP2OUT, 1);
			BIT(EP2IN, 1);
			BIT(EP3OUT, 1);
			BIT(EP3IN, 1);
			BIT(EP4OUT, 1);
			BIT(EP4IN, 1);
			RESERVED(20);
			BIT(FRAME_INT, 1);
			// Device status interrupt. This bit is set by HW when one of the bits in the Device
			// Status Change register are set.
			// Software can clear this bit by writing a one to it.
			BIT(DEV_INT, 1);
		};
		struct USB_INTEN
		{
			// (10 bits wide) If this bit is set and the corresponding USB interrupt status bit is set, a HW
			// interrupt is generated on the interrupt line indicated by the corresponding
			// USB interrupt routing bit.
			BIT(EP_INT_EN, 10);
			RESERVED(20);
			BIT(FRAME_INT_EN, 1);
			BIT(DEV_INT_EN, 1);
		};
		struct USB_INTSETSTAT
		{
			BIT(EP_SET_INT, 10);
			RESERVED(20);
			BIT(FRAME_SET_INT, 1);
			BIT(DEV_SET_INT, 1);
		};
		struct USB_EPTOGGLE
		{
			BIT(TOGGLE, 10);
		};
	}
}
#define RegisterType(reg) std::remove_reference<decltype(reg)>::type::Type
namespace Helpers
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
	static constexpr unsigned int CalculateSELP(const unsigned int M, const bool BYPASSCCODIV2)
	{
		const unsigned int m = BYPASSCCODIV2 ? M / 2 : M;
		unsigned int selp = ((550u * m) / (150u * 2u)) + 1u;
		if (selp > 0x1Fu)
			selp = 0x1Fu;
		return selp;
	};
	static constexpr unsigned int CalculateSELI(const unsigned int M, const bool BYPASSCCODIV2)
	{
		const unsigned int m = BYPASSCCODIV2 ? M / 2 : M;
		unsigned int SELI = 0u;

		if (m > (16384u * 150u / 550u))
			SELI = 1;
		else if (m > (8192u * 150u / 550u))
			SELI = 2;
		else if (m > (2048u * 150u / 550u))
			SELI = 4;
		else if (m >= (501u * 150u / 550u))
			SELI = 8;
		else if (m >= (60u * 150u / 550u))
			SELI = 4u * (1024u / (((550u * m) / 150u) + 9u) + 1u);
		else
			SELI = 4u * (((550u * m) / (150u * 4u)) + 1u);

		if (SELI > 0x3Fu)
			SELI = 0x3Fu;
		return SELI;
	};
	static constexpr unsigned int CalculateSELR(const unsigned int M, const bool BYPASSCCODIV2)
	{
		return 0u;
	};
	static constexpr unsigned int CalculateNDEC(const unsigned int N)
	{
		unsigned int N_max = 0x100u;
		unsigned int x = 0x80u;
		switch (N)
		{
		case 0: x = 0xFFFFFFFFu;
		case 1: x = 0x00000302u;
		case 2: x = 0x00000202u;
		default:
			for (unsigned int i = N; i <= N_max; i++)
				x = (((x ^ (x >> 2) ^ (x >> 3) ^ (x >> 4)) & 1) << 7) | ((x >> 1) & 0x7F);
		}
		return x;
	};

	static constexpr unsigned int CalculateMDEC(const unsigned int M)
	{
		unsigned int M_max = 0x00008000, x = 0x00004000;
		switch (M) {
		case 0: x = 0xFFFFFFFF;
		case 1: x = 0x00018003;
		case 2: x = 0x00010003;
		default: for (unsigned int i = M; i <= M_max; i++)
			x = (((x ^ (x >> 1)) & 1) << 14) | ((x >> 1) & 0x3FFF);
		}
		return x;
	};

	static constexpr unsigned int CalculatePDEC(const unsigned int P)
	{
		unsigned int P_max = 0x20, x = 0x10;
		switch (P) {
		case 0: x = 0xFFFFFFFF;
		case 1: x = 0x00000062;
		case 2: x = 0x00000042;
		default: for (unsigned int i = P; i <= P_max; i++)
			x = (((x ^ (x >> 2)) & 1) << 4) | ((x >> 1) & 0xF);
		}
		return x;
	};
#pragma GCC diagnostic pop
}
#undef REGISTERBASE
#undef REGISTERPARENT
#define REGISTERPARENT USART
#define REGISTERBASE Internal::Bases::Flexcomm0_BASE
namespace USART0
{
	REGISTER(CFG);
	REGISTER(CTL);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER16(BRG);
	REGISTERR(INTSTAT);
	REGISTER(OSR);
	REGISTER8(ADDR);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm1_BASE
namespace USART1
{
	REGISTER(CFG         );
	REGISTER(CTL         );
	REGISTER(STAT        );
	REGISTER(INTENSET    );
	REGISTERW(INTENCLR   );
	REGISTER16(BRG       );
	REGISTERR(INTSTAT    );
	REGISTER(OSR         );
	REGISTER8(ADDR       );
	REGISTER(FIFOCFG     );
	REGISTER(FIFOSTAT    );
	REGISTER(FIFOTRIG    );
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR     );
	REGISTERR(FIFORD     );
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID          );
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm2_BASE
namespace USART2
{
	REGISTER(CFG);
	REGISTER(CTL);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER16(BRG);
	REGISTERR(INTSTAT);
	REGISTER(OSR);
	REGISTER8(ADDR);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm3_BASE
namespace USART3
{
	REGISTER(CFG);
	REGISTER(CTL);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER16(BRG);
	REGISTERR(INTSTAT);
	REGISTER(OSR);
	REGISTER8(ADDR);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm4_BASE
namespace USART4
{
	REGISTER(CFG);
	REGISTER(CTL);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER16(BRG);
	REGISTERR(INTSTAT);
	REGISTER(OSR);
	REGISTER8(ADDR);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm5_BASE
namespace USART5
{
	REGISTER(CFG);
	REGISTER(CTL);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER16(BRG);
	REGISTERR(INTSTAT);
	REGISTER(OSR);
	REGISTER8(ADDR);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm6_BASE
namespace USART6
{
	REGISTER(CFG);
	REGISTER(CTL);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER16(BRG);
	REGISTERR(INTSTAT);
	REGISTER(OSR);
	REGISTER8(ADDR);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm7_BASE
namespace USART7
{
	REGISTER(CFG);
	REGISTER(CTL);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER16(BRG);
	REGISTERR(INTSTAT);
	REGISTER(OSR);
	REGISTER8(ADDR);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTER(ID);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT Nvic
#define REGISTERBASE Internal::Bases::Nvic_BASE
namespace Nvic
{
	REGISTER32(ISER0);
	REGISTER32(ICER0);
	REGISTER32(ISPR0);
	REGISTER32(ICPR0);
	REGISTER32(IPR0);
	REGISTER32(IPR1);
	REGISTER32(IPR2);
	REGISTER32(IPR3);
	REGISTER32(IPR4);
	REGISTER32(IPR5);
	REGISTER32(IPR6);
	REGISTER32(IPR7);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT SYSCON
#define REGISTERBASE Internal::Bases::SYSCON_BASE
namespace SYSCON
{
	REGISTER32(AHBMATPRIO);
	REGISTER32(SYSTCKCAL);
	REGISTER32(NMISRC);
	REGISTER32(ASYNCAPBCTRL);
	REGISTERR(PIOPORCAP0);
	REGISTERR(PIOPORCAP1);
	REGISTERR(PIORESCAP0);
	REGISTERR(PIORESCAP1);
	REGISTER32(PRESETCTRL0);
	REGISTER32(PRESETCTRL1);
	REGISTERW(PRESETCTRLSET0);
	REGISTERW(PRESETCTRLSET1);
	REGISTERW(PRESETCTRLCLR0);
	REGISTERW(PRESETCTRLCLR1);
	REGISTER32(SYSRSTSTAT);
	REGISTER(AHBCLKCTRL0);
	REGISTER(AHBCLKCTRL1);
	REGISTERW(AHBCLKCTRLSET0);
	REGISTERW(AHBCLKCTRLSET1);
	REGISTERW(AHBCLKCTRLCLR0);
	REGISTERW(AHBCLKCTRLCLR1);
	REGISTER32(MAINCLKSELA);
	REGISTER32(MAINCLKSELB);
	REGISTER32(CLKOUTSELA);
	REGISTER32(SYSPLLCLKSEL);
	REGISTER32(ADCCLKSEL);
	REGISTER32(USBCLKSEL);
	REGISTER32(FCLKSEL0);
	REGISTER32(FCLKSEL1);
	REGISTER32(FCLKSEL2);
	REGISTER32(FCLKSEL3);
	REGISTER32(FCLKSEL4);
	REGISTER32(FCLKSEL5);
	REGISTER32(FCLKSEL6);
	REGISTER32(FCLKSEL7);
	REGISTER32(MCLKCLKSEL);
	REGISTER32(FRGCLKSEL);
	REGISTER32(SYSTICKCLKDIV);
	REGISTER32(AHBCLKDIV);
	REGISTER32(CLKOUTDIV);
	REGISTER32(ADCCLKDIV);
	REGISTER32(USBCLKDIV);
	REGISTER32(FRGCTRL);
	REGISTER32(I2SMCLKDIV);
	REGISTER32(FLASHCFG);
	REGISTER32(USBCLKCTRL);
	REGISTER32(USBCLKSTAT);
	REGISTER32(FREQMECTRL);
	REGISTER32(MCLKIO);
	REGISTER(FROCTRL);
	REGISTER32(WDTOSCCTRL);
	REGISTER32(RTCOSCCTRL);
	REGISTER(SYSPLLCTRL);
	REGISTERR(SYSPLLSTAT);
	REGISTER(SYSPLLNDEC);
	REGISTER(SYSPLLPDEC);
	REGISTER(SYSPLLSSCTRL0);
	REGISTER(SYSPLLSSCTRL1);
	REGISTER32(PDSLEEPCFG0);
	REGISTER32(PDRUNCFG0);
	REGISTERW(PDRUNCFGSET0);
	REGISTERW(PDRUNCFGCLR0);
	REGISTER32(STARTER0);
	REGISTERW(STARTERSET0);
	REGISTERW(STARTERCLR0);
	REGISTER32(HWWAKE);
	REGISTERR(JTAGIDCODE);
	REGISTERR(DEVICE_ID0);
	REGISTERR(DEVICE_ID1);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT ASYSCON
#define REGISTERBASE Internal::Bases::ASYSCON_BASE
namespace ASYSCON
{
	REGISTER32(ASYNCPRESETCTRL);
	REGISTERW(ASYNCPRESETCTRLSET);
	REGISTERW(ASYNCPRESETCTRLCLR);
	REGISTER32(ASYNCAPBCLKCTRL);
	REGISTERW(ASYNCAPBCLKCTRLSET);
	REGISTERW(ASYNCAPBCLKCTRLCLR);
	REGISTER32(ASYNCAPBCLKSELA);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT IOCON
#define REGISTERBASE Internal::Bases::IOCON_BASE
namespace IOCON
{
	REGISTER(PORT0);
	REGISTER(PORT1);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT INMUX
#define REGISTERBASE Internal::Bases::INMUX_BASE
namespace INMUX
{
	REGISTER8(PINTSEL0);
	REGISTER8(PINTSEL1);
	REGISTER8(PINTSEL2);
	REGISTER8(PINTSEL3);
	REGISTER(DMA_ITRIG);
	REGISTER(DMA_OTRIG);
	REGISTER(FREQMEAS_REF);
	REGISTER(FREQMEAS_TARGET);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT USB
#define REGISTERBASE Internal::Bases::FSUSB_BASE
namespace USB
{
	REGISTER(DEVCMDSTAT);
	REGISTER(INFO);
	REGISTER(EPLISTSTART);
	// USB Data buffer start address
	// This register indicates the page of the AHB address where the endpoint data can be located.
	REGISTER(DATABUFSTART);
	REGISTER(LPM);
	REGISTER(EPSKIP);
	REGISTER(EPINUSE);
	REGISTER(EPBUFCFG);
	REGISTER(INTSTAT);
	REGISTER(INTEN);
	REGISTER(INTSETSTAT);
	REGISTERR(EPTOGGLE);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT GPIO
#define REGISTERBASE Internal::Bases::GPIO_BASE
namespace GPIO
{
	REGISTER(BYTE);
	REGISTER(WORD);
	
	// 0: Input
	// 1: Output
	REGISTER32(DIR0);
	// 0: Input
	// 1: Output
	REGISTER32(DIR1);
	// 0: Include in MPINx register read/writes.
	// 1: Mask value in MPINx register. Read as 0, writes are ignored.
	REGISTER32(MASK0);
	// 0: Include in MPINx register read/writes.
	// 1: Mask value in MPINx register. Read as 0, writes are ignored.
	REGISTER32(MASK1);
	REGISTER32(PIN0);
	REGISTER32(PIN1);
	REGISTER32(MPIN0);
	REGISTER32(MPIN1);
	REGISTERW(SET0);
	REGISTERW(SET1);
	REGISTERW(CLR0);
	REGISTERW(CLR1);
	REGISTERW(NOT0);
	REGISTERW(NOT1);
	// Selects bits representing port pins to set as an output.
	REGISTERW(DIRSET0);
	// Selects bits representing port pins to set as an output.
	REGISTERW(DIRSET1);
	// Selects bits representing port pins to set as an input.
	REGISTERW(DIRCLR0);
	// Selects bits representing port pins to set as an input.
	REGISTERW(DIRCLR1);
	// Selects bits representing port pins to switch direction.
	REGISTERW(DIRNOT0);
	// Selects bits representing port pins to switch direction.
	REGISTERW(DIRNOT1);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT FLEXCOMM
#define REGISTERBASE Internal::Bases::Flexcomm0_BASE
namespace Flexcomm0
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm1_BASE
namespace Flexcomm1
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm2_BASE
namespace Flexcomm2
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm3_BASE
namespace Flexcomm3
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm4_BASE
namespace Flexcomm4
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm5_BASE
namespace Flexcomm5
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm6_BASE
namespace Flexcomm6
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm7_BASE
namespace Flexcomm7
{
	REGISTER(PSELID);
	REGISTERR(PID);
}
#undef REGISTERPARENT
#undef REGISTERBASE
#define REGISTERPARENT SPI
#define REGISTERBASE Internal::Bases::Flexcomm0_BASE
namespace SPI0
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm1_BASE
namespace SPI1
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm2_BASE
namespace SPI2
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm3_BASE
namespace SPI3
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm4_BASE
namespace SPI4
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm5_BASE
namespace SPI5
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm6_BASE
namespace SPI6
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#define REGISTERBASE Internal::Bases::Flexcomm7_BASE
namespace SPI7
{
	REGISTER(CFG);
	REGISTER(DLY);
	REGISTER(STAT);
	REGISTER(INTENSET);
	REGISTERW(INTENCLR);
	REGISTER(DIV);
	REGISTERR(INTSTAT);
	REGISTER(FIFOCFG);
	REGISTER(FIFOSTAT);
	REGISTER(FIFOTRIG);
	REGISTER(FIFOINTENSET);
	REGISTER(FIFOINTENCLR);
	REGISTERR(FIFOINTSTAT);
	REGISTERW(FIFOWR);
	REGISTERR(FIFORD);
	REGISTERR(FIFORDNOPOP);
	REGISTERR(ID);
}
#undef REGISTERBASE
#undef REGISTERPARENT
#undef REGISTER
#undef REGISTER8
#undef REGISTER16
#undef REGISTER32
#undef RESERVED
#undef BIT
#pragma GCC diagnostic pop

#endif
