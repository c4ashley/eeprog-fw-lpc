OBJS += main
PROJECT := eeprog
MCU = LPC51U68

#MCUXPRESSO       = /usr/local/mcuxpressoide-11.7.1_9221/ide
#MCUXPRESSOTOOLS  = $(MCUXPRESSO)/plugins/com.nxp.mcuxpresso.tools.bin.linux_11.7.1.202303220859
MCUXPRESSOTOOLS = /opt/nxp
WORKSPACE        = ${HOME}/Documents/MCUXpresso_11.7.1_9221/workspace
WORKSPACESUPPORT = $(WORKSPACE)/.mcuxpressoide_packages_support/LPC51U68_support
FLASHDIRS       += --flash-dir $(MCUXPRESSOTOOLS)/binaries/Flash --flash-dir $(WORKSPACESUPPORT)/Flash

#TOOLCHAINDIR = $(MCUXPRESSOTOOLS)/tools/bin/
TOOLCHAINDIR = /opt/arm-none-eabi/bin/
TOOLCHAIN = arm-none-eabi-
GDB = $(TOOLCHAINDIR)$(TOOLCHAIN)gdb
# GDB = gdb-multiarch

DEFINES    += CPU_LPC51U68JBD64 CPU_LPC51U68JBD64_cm0plus
LFLAGS     += -Xlinker --sort-section=alignment
LFLAGS     += -L./system/
LFLAGS     += -T lpc51u68.ld
FLAGS      += -g3 -mcpu=cortex-m0plus -mthumb -nostdlib -Og -Wall -Wextra -specs=nano.specs -fno-exceptions 
CFLAGS     += -std=gnu17 -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fstack-usage  -Wno-missing-field-initializers 
CXXFLAGS   += -std=gnu++20 -fno-common -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fstack-usage  -Wno-missing-field-initializers -Wno-volatile -fno-rtti
#INCDIRS    += $(MCUXPRESSOTOOLS)/tools/arm-none-eabi/include
SYSINCDIRS += ./system


all: $(PROJECT).axf $(PROJECT).lst $(PROJECT).sz $(PROJECT).bin

clean:
	rm $(PROJECT).axf $(PROJECT).lst $(PROJECT).sz $(PROJECT).bin $(addprefix $(OBJS),.o) system/startup.o $(addprefix $(OBJS),.su) out.rom system/startup.su

install: install-jlink
#install: install-redlink

$(PROJECT).axf: $(addprefix $(OBJS),.o) system/startup.o
	$(TOOLCHAINDIR)$(TOOLCHAIN)g++ $(FLAGS) $(LFLAGS) -o $@ $^

$(PROJECT).bin: $(PROJECT).axf
	-@$(TOOLCHAINDIR)$(TOOLCHAIN)objcopy -O binary $^ $@

$(PROJECT).sz: $(PROJECT).axf
	-@$(TOOLCHAINDIR)$(TOOLCHAIN)size $^ | tee $@

$(PROJECT).lst: $(PROJECT).axf
	-@$(TOOLCHAINDIR)$(TOOLCHAIN)objdump -dS $^ > $@

%.o: %.cpp
	-@$(TOOLCHAINDIR)$(TOOLCHAIN)g++ $(FLAGS) $(CXXFLAGS) $(addprefix -D,$(DEFINES)) $(addprefix -I,$(INCDIRS)) $(addprefix -isystem,$(SYSINCDIRS)) -c -o $@ $^

%.o: %.c
	-@$(TOOLCHAINDIR)$(TOOLCHAIN)gcc $(FLAGS) $(CFLAGS) $(addprefix -D,$(DEFINES)) $(addprefix -I,$(INCDIRS)) $(addprefix -isystem,$(SYSINCDIRS)) -c -o $@ $^

install-redlink: $(PROJECT).axf
	$(GDB) --batch -ex 'target extended-remote | $(MCUXPRESSOTOOLS)/binaries/crt_emu_cm_redlink --vendor NXP -p $(MCU) -x $(WORKSPACE)/LPC51U68_Project/Debug' \
		-ex 'load' \
		-ex 'mon reset halt' \
		-ex 'c' \
		$^
	#gdb-multiarch $^
	#target extended-remote | $(MCUXPRESSOTOOLS)/binaries/crt_emu_cm_redlink --vendor NXP -p $(MCU) -x $(WORKSPACE)/LPC51U68_Project/Debug
	#$(MCUXPRESSOTOOLS)/binaries/crt_emu_cm_redlink --flash-load-exec "$^" -g --debug 2 --vendor NXP -p $(MCU) --probeserial ESATAQMQ -CoreIndex=1 --flash-driver= -x $(WORKSPACE)/LPC51U68_Project/Debug $(FLASHDIRS) --flash-hashing

install-jlink: $(PROJECT).axf
	if ! pidof JLinkGDBServerCLExe > /dev/null; then { JLinkGDBServerCLExe -device $(MCU) -if SWD & sleep 1; }; fi && \
	$(GDB) --batch -ex 'target extended-remote :2331' \
		-ex 'load' \
		$^ && \
	killall JLinkGDBServerCLExe

install-file: $(PROJECT).bin
	./install-file.sh "$^"

bootlink:
	$(MCUXPRESSOTOOLS)/binaries/boot_link2



