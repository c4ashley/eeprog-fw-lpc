#!/bin/sh
if command -v udisksctl &>/dev/null; then
	if [ -d "/run/media/$USER/CRP DISABLD" ]; then
		cp $1 /run/media/$USER/CRP\ DISABLD/firmware.bin
	else
		udisksctl mount -b /dev/disk/by-id/usb-NXP_LPC_MCU_IFLASH_ISP-0\:0 && \
		cp $1 /run/media/$USER/CRP\ DISABLD/firmware.bin && \
		sync && \
		udisksctl unmount -b /dev/disk/by-id/usb-NXP_LPC_MCU_IFLASH_ISP-0\:0
	fi
else
	mkdir iflash && \
	mount -o gid=$UID,uid=$UID /dev/disk/by/id/usb-NXP_LPC_MCU_IFLASH_ISP-0\:0 iflash && \
	cp $1 iflash/firmware.bin && \
	sync && \
	sudo umount iflash && \
	rmdir iflash
fi
