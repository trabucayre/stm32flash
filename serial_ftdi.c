/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <assert.h>

#include <sys/errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <libudev.h>
#include <libusb.h>
#include <ftdi.h>
#include <string.h>

#include "serial.h"

#define TIOCM_CBUS 0x545E // to use ioctl for CBUS pins

struct serial {
	int			fd;
	struct ftdi_context *ftdi;
	int vid;
	int pid;
	int interface;

	struct termios		oldtio;
	struct termios		newtio;

	char			configured;
	serial_baud_t		baud;
	serial_bits_t		bits;
	serial_parity_t		parity;
	serial_stopbit_t	stopbit;
};

unsigned int udevstufftoint(const char *udevstring, int base)
{
	char *endp;
	int ret;
	errno = 0;

	if (udevstring == NULL)
		return (-1);

	ret = (unsigned int)strtol(udevstring, &endp, base);
	if (errno) {
		fprintf(stderr,
			"udevstufftoint: Unable to parse number Error : %s (%d)\n",
			strerror(errno), errno);
		return (-2);
	}
	if (endp == optarg) {
		fprintf(stderr, "udevstufftoint: No digits were found\n");
		return (-3);
	}
	return (ret);
}

void find_ftdi_and_open(serial_t *h, int devnum)
{
    int ret;
    struct ftdi_device_list *devlist, *curdev;
	
	ret = ftdi_usb_find_all(h->ftdi, &devlist, h->vid, h->pid);
	/* test needed */
	/* ... */

	for (curdev = devlist; curdev != NULL; curdev = curdev->next) {
		// test needed
		if (libusb_get_device_address(curdev->dev) == devnum)
			break;
	}

	if (curdev == NULL)
		h->fd = -1;
	else 
		h->fd = ftdi_usb_open_dev(h->ftdi, curdev->dev);
	
	ftdi_list_free(&devlist);
}

void serial_set_cbus(const serial_t *h, int cbus) {
	ftdi_set_bitmode(h->ftdi, cbus, BITMODE_CBUS);
}

serial_t* serial_open(const char *device) {
	serial_t *h = calloc(sizeof(serial_t), 1);
	struct udev *udev;
	struct udev_device *dev, *usbdeviceparent;
	char devtype;

	struct stat statinfo;
	if (stat(device, &statinfo) < 0) {
		printf("unable to stat file\n");
		return NULL;
	}

	/* get device type */
	switch (statinfo.st_mode & S_IFMT) {
	case S_IFBLK:
		devtype = 'b';
		break;
	case S_IFCHR:
		devtype = 'c';
		break;
	default:
		printf("not char or block device\n");
		return NULL;
	}
	printf("Using %s (UID=%ld GID=%ld perm=%lo)  %d:%d\n",
	       device, (long)statinfo.st_uid, (long)statinfo.st_gid,
	       (unsigned long)statinfo.st_mode,
	       major(statinfo.st_rdev), minor(statinfo.st_rdev));

	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		return NULL;
	}

	dev = udev_device_new_from_devnum(udev, devtype, statinfo.st_rdev);

	if (dev == NULL) {
		printf("no dev\n");
		udev_unref(udev);
		exit(EXIT_FAILURE);
	}

	/* Get closest usb device parent (we need VIP/PID)  */
	usbdeviceparent =
	    udev_device_get_parent_with_subsystem_devtype(dev, "usb",
							  "usb_device");
	if (!usbdeviceparent) {
		printf
		    ("Unable to find parent usb device! Is this actually an USB device ?\n");
		udev_device_unref(dev);
		udev_unref(udev);
		return NULL;
	}

	h->vid =
	    udevstufftoint(udev_device_get_sysattr_value
			   (usbdeviceparent, "idVendor"), 16);
	h->pid =
	    udevstufftoint(udev_device_get_sysattr_value
			   (usbdeviceparent, "idProduct"), 16);
	int devnum =
	    udevstufftoint(udev_device_get_sysattr_value
			   (usbdeviceparent, "devnum"), 10);
	printf("%04x %04x %04x\n", h->vid, h->pid, devnum);

	udev_device_unref(dev);
	udev_unref(udev);

	/* libftdi init */
	h->interface = INTERFACE_A;//NY;

	h->ftdi = ftdi_new();
	if (h->ftdi == 0) {
		fprintf(stderr, "ftdi_new failed\n");
		return NULL;
	}
	ftdi_set_interface(h->ftdi, h->interface);
	find_ftdi_and_open(h, devnum);
	/* h->fd = ftdi_usb_open(h->ftdi, h->vid, h->pid);*/
	if (h->fd < 0) {
		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", h->fd, 
						ftdi_get_error_string(h->ftdi));
		return NULL;
	}
	h->ftdi->usb_write_timeout = 10000;
	h->ftdi->usb_read_timeout = 10000;
    if (h->ftdi->type == TYPE_R)
    {
		unsigned int chipid;
		printf("ftdi_read_chipid: %d\n", ftdi_read_chipid(h->ftdi, &chipid));
		printf("FTDI chipid: %X\n", chipid);
	}

	
	return h;
}

/* cf. ftdi.c same function */
static void ftdi_usb_close_internal (struct ftdi_context *ftdi)
{
	libusb_close (ftdi->usb_dev);
	ftdi->usb_dev = NULL;
}

void serial_close(serial_t *h) {
	assert(h && h->ftdi !=NULL);
	struct ftdi_context *ftdi = h->ftdi;
	int rtn;
	ftdi_usb_purge_tx_buffer(h->ftdi); 
	ftdi_usb_purge_rx_buffer(h->ftdi); 

	/*ftdi_usb_close(h->ftdi);
	 * repompe de la fonction et des suivantes
	 */
	 if (ftdi->usb_dev != NULL) {
		rtn = libusb_release_interface(ftdi->usb_dev, ftdi->interface);
		if (rtn < 0) {
			printf("release interface failed %d\n", rtn);
			return;
		}
		if (ftdi->module_detach_mode == AUTO_DETACH_SIO_MODULE) {
			rtn = libusb_attach_kernel_driver(ftdi->usb_dev, ftdi->interface);
			if( rtn != 0)
				printf("detach error %d\n",rtn);
		}
	}
	ftdi_usb_close_internal(ftdi);

	ftdi_free(ftdi);
	free(h);
}

void serial_flush(const serial_t *h) {
	ftdi_usb_purge_tx_buffer(h->ftdi); 
	ftdi_usb_purge_rx_buffer(h->ftdi); 
}

serial_err_t serial_setup(serial_t *h, const serial_baud_t baud, 
				const serial_bits_t bits, const serial_parity_t parity, 
				const serial_stopbit_t stopbit) 
{
	assert(h && h->ftdi);

	int	port_baud;
	enum ftdi_bits_type port_bits;
	enum ftdi_parity_type port_parity;
	enum ftdi_stopbits_type port_stop;

	switch(baud) {
		case SERIAL_BAUD_1200  : port_baud = 1200  ; break;
		case SERIAL_BAUD_1800  : port_baud = 1800  ; break;
		case SERIAL_BAUD_2400  : port_baud = 2400  ; break;
		case SERIAL_BAUD_4800  : port_baud = 4800  ; break;
		case SERIAL_BAUD_9600  : port_baud = 9600  ; break;
		case SERIAL_BAUD_19200 : port_baud = 19200 ; break;
		case SERIAL_BAUD_38400 : port_baud = 38400 ; break;
		case SERIAL_BAUD_57600 : port_baud = 57600 ; break;
		case SERIAL_BAUD_115200: port_baud = 115200; break;

		case SERIAL_BAUD_INVALID:
		default:
			return SERIAL_ERR_INVALID_BAUD;
	}

	switch(bits) {
		//case SERIAL_BITS_5: port_bits = CS5; break;
		//case SERIAL_BITS_6: port_bits = CS6; break;
		case SERIAL_BITS_7: port_bits = BITS_7; break;
		case SERIAL_BITS_8: port_bits = BITS_8; break;

		default:
			return SERIAL_ERR_INVALID_BITS;
	}

	switch(parity) {
		case SERIAL_PARITY_NONE: port_parity = NONE;	break;
		case SERIAL_PARITY_EVEN: port_parity = EVEN;	break;
		case SERIAL_PARITY_ODD : port_parity = ODD;		break;

		default:
			return SERIAL_ERR_INVALID_PARITY;
	}

	switch(stopbit) {
		case SERIAL_STOPBIT_1: port_stop = STOP_BIT_1;	break;
		case SERIAL_STOPBIT_2: port_stop = STOP_BIT_2;	break;

		default:
			return SERIAL_ERR_INVALID_STOPBIT;
	}

	/* if the port is already configured, no need to do anything */
	if (
		h->configured        &&
		h->baud	   == baud   &&
		h->bits	   == bits   &&
		h->parity  == parity &&
		h->stopbit == stopbit
	) return SERIAL_ERR_OK;

	/* setup the new settings */
	int f = ftdi_set_baudrate(h->ftdi, port_baud);
	if (f < 0)
	{
		fprintf(stderr, "unable to set baudrate: %d (%s)\n", f, 
					ftdi_get_error_string(h->ftdi));
		return SERIAL_ERR_SYSTEM;
	}	
	f = ftdi_set_line_property(h->ftdi, port_bits, port_stop, port_parity);
	if (f < 0) {
		fprintf(stderr, "unable to set line parameters: %d (%s)\n", f, 
					ftdi_get_error_string(h->ftdi));
		return SERIAL_ERR_SYSTEM;
	}

	serial_flush(h);
	h->configured = 1;
	h->baud	      = baud;
	h->bits	      = bits;
	h->parity     = parity;
	h->stopbit    = stopbit;
	h->ftdi->usb_write_timeout = 10000;
	h->ftdi->usb_read_timeout = 10000;
	ftdi_write_data_set_chunksize(h->ftdi, 512);

	//ftdi_set_bitmode(h->ftdi, bitmask, BITMODE_CBUS);

	return SERIAL_ERR_OK;
}

serial_err_t serial_write(const serial_t *h, const void *buffer, unsigned int len) {
	assert(h && h->ftdi != NULL && h->configured);

	ssize_t r;
	uint8_t *pos = (uint8_t*)buffer;
	int baudrate=115200;

	while(len > 0) {
		r = ftdi_write_data(h->ftdi, pos, (baudrate/512 >len)?(len):
		                                (baudrate/512)?baudrate/512:1);
		//len);
		if (r < 1) return SERIAL_ERR_SYSTEM;

		len -= r;
		pos += r;
	}

	return SERIAL_ERR_OK;
}

serial_err_t serial_read(const serial_t *h, const void *buffer, unsigned int len) {
	assert(h && h->ftdi != NULL && h->configured);

	int timeout;
	ssize_t r;
	uint8_t *pos = (uint8_t*)buffer;
	while(len > 0) {
		timeout = 0;
		do {
			r = ftdi_read_data(h->ftdi, pos, len);//read(h->fd, pos, len);
		} while(r < 1 && timeout++ < 100);
			if (r == 0) {
				fprintf(stderr, "erreur no data\n");
				return SERIAL_ERR_NODATA;
			}
			else  if (r <  0) {
				fprintf(stderr, "erreur err system %d\n", (int)r);
				return SERIAL_ERR_SYSTEM;
			}

		len -= r;
		pos += r;
	}

	return SERIAL_ERR_OK;
}

const char* serial_get_setup_str(const serial_t *h) {
	/* TBD */
	static char str[11];
	if (!h->configured)
		snprintf(str, sizeof(str), "INVALID");
	else
		snprintf(str, sizeof(str), "%u %d%c%d",
			serial_get_baud_int   (h->baud   ),
			serial_get_bits_int   (h->bits   ),
			serial_get_parity_str (h->parity ),
			serial_get_stopbit_int(h->stopbit)
		);

	return str;
}

