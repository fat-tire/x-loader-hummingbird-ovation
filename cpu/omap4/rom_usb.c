/*
 * Copyright (C) 2010 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/omap_rom.h>

struct usb usbdev;

static void memset(void *_ptr, unsigned char c, unsigned size)
{
	unsigned char *ptr = _ptr;

	while (size--)
		*ptr++ = c;
}

int usb_open(struct usb *usb)
{
	struct per_handle *boot;
	int n;

	/*clear global usb structure*/
	memset(usb, 0, sizeof(*usb));

	/* get peripheral device descriptor
	that was used during rom usb boot */
	n = rom_get_per_device(&boot);
	if (n)
		return n;

	/* exit if boot device is not usb */
	if ((boot->device_type != DEVICE_USB) &&
	    (boot->device_type != DEVICE_USBEXT))
		return -1;

	/* get rom usb driver */
	n = rom_get_per_driver(&usb->io, boot->device_type);
	if (n)
		return n;

#if defined(CONFIG_IS_OMAP4)
	usb->dread.xfer_mode = boot->xfer_mode;
#endif
	usb->dread.config_object = boot->config_object;
	usb->dread.options = boot->options;
	usb->dread.device_type = boot->device_type;
	usb->dread.device_data = 0;

#if defined(CONFIG_IS_OMAP4)
	usb->dwrite.xfer_mode = boot->xfer_mode;
#endif
	usb->dwrite.config_object = boot->config_object;
	usb->dwrite.options = boot->options;
	usb->dwrite.device_type = boot->device_type;
	usb->dwrite.device_data = 0;

	return 0;
}


struct usb *local_read_usb;
static void rom_read_callback(struct per_handle *rh)
{

	local_read_usb->dread.status = rh->status;
	return;
}

void usb_queue_read(struct usb *usb, void *data, unsigned len)
{
	int n;
	usb->dread.data = data;
	usb->dread.length = len;
	usb->dread.status = -1;
	usb->dread.device_type = DEVICE_USB;
	usb->dread.device_data = 0;
#if defined(CONFIG_IS_OMAP4)
	usb->dread.xfer_mode = 1;
#endif
	usb->dread.callback = rom_read_callback;
	local_read_usb = usb;
	n = usb->io->read(&usb->dread);
	if (n)
		usb->dread.status = n;
}

int usb_wait_read(struct usb *usb)
{
	for (;;) {
		if (usb->dread.status == -1)
			continue;
		if (usb->dread.status == STATUS_WAITING)
			continue;
		return usb->dread.status;
	}
}

struct usb *local_write_usb;
void rom_write_callback(struct per_handle *rh)
{
	local_write_usb->dwrite.status = rh->status;
	return;
}

void usb_queue_write(struct usb *usb, void *data, unsigned len)
{
	int n;
	usb->dwrite.data = data;
	usb->dwrite.length = len;
	usb->dwrite.status = -1;
#if defined(CONFIG_IS_OMAP4)
	usb->dwrite.xfer_mode = 1;
#endif
	usb->dwrite.device_type = DEVICE_USB;
	usb->dwrite.device_data = 0;
	usb->dwrite.callback = rom_write_callback;
	local_write_usb = usb;
	n = usb->io->write(&usb->dwrite);
	if (n)
		usb->dwrite.status = n;
}

int usb_wait_write(struct usb *usb)
{
	for (;;) {
		if (usb->dwrite.status == -1)
			continue;
		if (usb->dwrite.status == STATUS_WAITING)
			continue;
		return usb->dwrite.status;
	}
}

#define USB_MAX_IO 65536
int usb_read(struct usb *usb, void *data, unsigned len)
{
	unsigned xfer;
	unsigned char *x = data;
	int n;
	while (len > 0) {
		xfer = (len > USB_MAX_IO) ? USB_MAX_IO : len;
		usb_queue_read(usb, x, xfer);
		n = usb_wait_read(usb);
		if (n)
			return n;
		x += xfer;
		len -= xfer;
	}
	return 0;
}

int usb_write(struct usb *usb, void *data, unsigned len)
{
	usb_queue_write(usb, data, len);
	return usb_wait_write(usb);
}

void usb_close(struct usb *usb)
{
	usb->io->close(&usb->dread);
}
