// Framebuffer-copy-to-two-SPI-screens utility for "Pi Eyes" project.
// Compatible with Raspberry Pi 3B, 4 and 5 running Raspberry Pi OS Trixie.
// Uses two RGB screens with SPI interface, either:
//  - SSD1351 OLED   www.adafruit.com/products/1431
//  - ST7789 IPS TFT www.adafruit.com/products/3787
//  - ST7735 TFT LCD www.adafruit.com/products/2088 ("green tab" version)
// NOT COMPATIBLE WITH OTHER DISPLAYS.

// Requires SPI and (optionally) I2C enabled in /boot/firmware/config.txt:
//     dtparam=spi=on
//     dtparam=spi1=on
//     dtoverlay=spi1-3cs
//     dtparam=i2c_arm=on   (only if using the ADC)
// Increase SPI buffer size by appending to /boot/firmware/cmdline.txt:
//     spidev.bufsiz=8192
// THE ABOVE ARE HANDLED BY THE pi-eyes.sh INSTALLER SCRIPT:
// https://github.com/adafruit/Raspberry-Pi-Installer-Scripts

// Options: -o OLED  -t TFT  -i IPS
//          -b ### SPI bitrate  -w ### window sync interval  -s show FPS

// Screen layout: the display is divided in half horizontally. Centered in
// each half, a 256x256 region (OLED/TFT) or 480x480 region (IPS) is
// scaled 50% via 2x2 box filter to produce the 128x128 or 240x240 bitmap
// sent to each SPI screen. Configure the eye renderer for 640x480 (OLED/TFT)
// or 1280x720 (IPS). The 2x2 filter gives effective 16x antialiasing.

// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.
// Trixie port: X11 MIT-SHM capture replaces dispmanx (removed in Bookworm).
//              Linux GPIO character device replaces /dev/mem mmap
//              (works on Pi 3B, 4 and 5 without any model-specific code).
// Compile: gcc -O2 -o fbx2 fbx2.c -lpthread -lm -lX11 -lXext

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/shm.h>
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>


// CONFIGURATION AND GLOBAL STUFF ------------------------------------------

#define DC_PIN    5   // BCM GPIO pin numbers — connect to BOTH screens
#define RESET_PIN 6

#define SCREEN_OLED      0
#define SCREEN_TFT_GREEN 1
#define SCREEN_IPS       2

uint8_t screenType = SCREEN_OLED;

static const uint8_t initOLED[] = {
  0xFD,  1, 0x12,
  0xFD,  1, 0xB1,
  0xAE,  0,
  0xB3,  1, 0xF0,
  0xCA,  1, 0x7F,
  0xA2,  1, 0x00,
  0xA1,  1, 0x00,
  0xA0,  1, 0x74,
  0xB5,  1, 0x00,
  0xAB,  1, 0x01,
  0xB4,  3, 0xA0, 0xB5, 0x55,
  0xC1,  3, 0xFF, 0xA3, 0xFF,
  0xC7,  1, 0x0F,
  0xB1,  1, 0x32,
  0xBB,  1, 0x07,
  0xB2,  3, 0xA4, 0x00, 0x00,
  0xB6,  1, 0x01,
  0xBE,  1, 0x05,
  0xA6,  0,
  0xAF,  0,
  0xB8, 64,
    0x00, 0x08, 0x0D, 0x12, 0x17, 0x1B, 0x1F, 0x22,
    0x26, 0x2A, 0x2D, 0x30, 0x34, 0x37, 0x3A, 0x3D,
    0x40, 0x43, 0x46, 0x49, 0x4C, 0x4F, 0x51, 0x54,
    0x57, 0x59, 0x5C, 0x5F, 0x61, 0x64, 0x67, 0x69,
    0x6C, 0x6E, 0x71, 0x73, 0x76, 0x78, 0x7B, 0x7D,
    0x7F, 0x82, 0x84, 0x86, 0x89, 0x8B, 0x8D, 0x90,
    0x92, 0x94, 0x97, 0x99, 0x9B, 0x9D, 0x9F, 0xA2,
    0xA4, 0xA6, 0xA8, 0xAA, 0xAD, 0xAF, 0xB1, 0xB3,
  0x00 },
initTFT[] = {
  0x01, 0x80, 150,
  0x11, 0x80, 255,
  0xB1,    3, 0x01, 0x2C, 0x2D,
  0xB2,    3, 0x01, 0x2C, 0x2D,
  0xB3,    6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
  0xB4,    1, 0x07,
  0xC0,    3, 0xA2, 0x02, 0x84,
  0xC1,    1, 0xC5,
  0xC2,    2, 0x0A, 0x00,
  0xC3,    2, 0x8A, 0x2A,
  0xC4,    2, 0x8A, 0xEE,
  0xC5,    1, 0x0E,
  0x20,    0,
  0x36,    1, 0xC8,
  0x3A,    1, 0x05,
  0x2A,    4, 0x00, 0x00, 0x00, 0x7F,
  0x2B,    4, 0x00, 0x00, 0x00, 0x7F,
  0xE0,   16, 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
              0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
  0xE1,   16, 0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
              0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
  0x13, 0x80,  10,
  0x29, 0x80, 100,
  0x00 },
initIPS[] = {
  0x01, 0x80, 150,
  0x11, 0x80, 255,
  0x3A, 0x81, 0x55,  10,
  0x36,    1, 0x00,
  0x26,    1, 0x02,
  0xBA,    1, 0x04,
  0x21, 0x80,  10,
  0x13, 0x80,  10,
  0x29, 0x80, 255,
  0x00 },
winOLED[] = {
  0x15, 2, 0x00, 0x7F,
  0x75, 2, 0x00, 0x7F,
  0x5C,
  0x00 },
winTFT[] = {
  0x2A, 4, 0, 2, 0, 129,
  0x2B, 4, 0, 3, 0, 130,
  0x2C,
  0x00 },
winIPS[] = {
  0x2A, 4, 0, 0, 0, 239,
  0x2B, 4, 0, 0, 0, 239,
  0x2C,
  0x00 };

static const struct {
	const int      width;
	const int      height;
	const int      bitrate;
	const uint8_t *init;
	const uint8_t *win;
} screenInfo[] = {
  { 128, 128, 10000000, initOLED, winOLED },
  { 128, 128, 12000000, initTFT,  winTFT  },
  { 240, 240, 80000000, initIPS,  winIPS  } };

static struct {
	int        fd;
	uint16_t  *buf[2];
	pthread_t  thread;
	struct spi_ioc_transfer xfer;
} eye[2];

static pthread_barrier_t barr;
static uint8_t           bufIdx = 0;
static int               bufsiz = 4096;
static struct spi_ioc_transfer xfer = {
  .rx_buf        = 0,
  .delay_usecs   = 0,
  .bits_per_word = 8,
  .pad           = 0,
  .tx_nbits      = 0,
  .rx_nbits      = 0,
  .cs_change     = 0 };

// GPIO state — uses Linux GPIO character device, works on Pi 3B, 4 and 5
static int gpioFd = -1; // fd from GPIO_V2_GET_LINE_IOCTL


// UTILITY FUNCTIONS -------------------------------------------------------

static void setDC(int val) {
	struct gpio_v2_line_values v = {
	  .mask = 1ULL,
	  .bits = val ? 1ULL : 0ULL };
	ioctl(gpioFd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
}

static void setRST(int val) {
	struct gpio_v2_line_values v = {
	  .mask = 2ULL,
	  .bits = val ? 2ULL : 0ULL };
	ioctl(gpioFd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
}

#define COMMAND 0
#define DATA    1

static void dcX2(uint8_t x, uint8_t dc) {
	setDC(dc);
	xfer.tx_buf = (__u64)(uintptr_t)&x;
	xfer.len    = 1;
	(void)ioctl(eye[0].fd, SPI_IOC_MESSAGE(1), &xfer);
	(void)ioctl(eye[1].fd, SPI_IOC_MESSAGE(1), &xfer);
}

static void commandList(const uint8_t *ptr) {
	int i, j, ms;
	for(i=0; (j=ptr[i++]);) {
		dcX2(j, COMMAND);
		j  = ptr[i++];
		ms = j & 0x80;
		j &= ~0x80;
		while(j--) dcX2(ptr[i++], DATA);
		if(ms) {
			ms = ptr[i++];
			if(ms == 255) ms = 500;
			usleep(ms * 1000);
		}
	}
}

void *spiThreadFunc(void *data) {
	int      i = *(uint8_t *)data;
	uint32_t bytesThisPass, bytesToGo, screenBytes =
	  screenInfo[screenType].width * screenInfo[screenType].height * 2;

	for(;;) {
		pthread_barrier_wait(&barr);
		pthread_barrier_wait(&barr);

		eye[i].xfer.tx_buf = (__u64)(uintptr_t)eye[i].buf[bufIdx];
		bytesToGo = screenBytes;
		do {
			bytesThisPass = bytesToGo;
			if(bytesThisPass > (uint32_t)bufsiz) bytesThisPass = bufsiz;
			eye[i].xfer.len = bytesThisPass;
			(void)ioctl(eye[i].fd, SPI_IOC_MESSAGE(1), &eye[i].xfer);
			eye[i].xfer.tx_buf += bytesThisPass;
			bytesToGo          -= bytesThisPass;
		} while(bytesToGo > 0);
	}
	return NULL;
}

static int err(int code, char *string) {
	(void)puts(string);
	exit(code);
}


// INIT AND MAIN LOOP ------------------------------------------------------

int main(int argc, char *argv[]) {

	uint8_t showFPS   = 0;
	int     bitrate   = 0,
	        winFrames = 1,
	        i, j, fd;

	while((i = getopt(argc, argv, "otib:w:s")) != -1) {
		switch(i) {
		   case 'o': screenType = SCREEN_OLED;      break;
		   case 't': screenType = SCREEN_TFT_GREEN;  break;
		   case 'i': screenType = SCREEN_IPS;        break;
		   case 'b': bitrate   = strtol(optarg, NULL, 0); break;
		   case 'w': winFrames = strtol(optarg, NULL, 0); break;
		   case 's': showFPS   = 1;                  break;
		}
	}

	if(!bitrate) bitrate = screenInfo[screenType].bitrate;

	FILE *fp;
	if((fp = fopen("/sys/module/spidev/parameters/bufsiz", "r"))) {
		if(fscanf(fp, "%d", &i) == 1) bufsiz = i;
		fclose(fp);
	}

	// GPIO INIT -------------------------------------------------------
	// Uses the Linux GPIO character device — no /dev/mem mmap needed.
	// Works identically on Pi 3B (BCM2837), Pi 4 (BCM2711) and
	// Pi 5 (RP1) without any model-specific code.

	{
		int chipFd;
		struct gpio_v2_line_request req;

		if((chipFd = open("/dev/gpiochip0", O_RDONLY)) < 0)
			err(1, "Can't open /dev/gpiochip0 (try 'sudo')");

		memset(&req, 0, sizeof(req));
		req.offsets[0] = DC_PIN;
		req.offsets[1] = RESET_PIN;
		req.num_lines  = 2;
		req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
		strncpy(req.consumer, "fbx2", GPIO_MAX_NAME_SIZE - 1);

		if(ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &req) < 0)
			err(2, "Can't request GPIO lines");
		close(chipFd);
		gpioFd = req.fd;
	}

	if((eye[0].fd = open("/dev/spidev0.0", O_WRONLY|O_NONBLOCK)) < 0)
		err(3, "Can't open spidev0.0, is SPI enabled?");
	if((eye[1].fd = open("/dev/spidev1.2", O_WRONLY|O_NONBLOCK)) < 0)
		err(4, "Can't open spidev1.2, is spi1-3cs overlay enabled?");

	xfer.speed_hz = bitrate;
	uint8_t mode = SPI_MODE_0;
	for(i=0; i<2; i++) {
		ioctl(eye[i].fd, SPI_IOC_WR_MODE, &mode);
		ioctl(eye[i].fd, SPI_IOC_WR_MAX_SPEED_HZ, bitrate);
		memcpy(&eye[i].xfer, &xfer, sizeof(xfer));
		for(j=0; j<2; j++) {
			if(NULL == (eye[i].buf[j] = (uint16_t *)malloc(
			  screenInfo[screenType].width *
			  screenInfo[screenType].height * sizeof(uint16_t))))
				err(5, "Eye buffer malloc failed");
		}
	}

	// INITIALIZE SPI SCREENS ------------------------------------------

	setRST(1); usleep(5);
	setRST(0); usleep(5);
	setRST(1); usleep(5);

	commandList(screenInfo[screenType].init);

	// X11 CAPTURE INIT ------------------------------------------------
	// eyes.py renders to X display :0 via xinit.  We connect as a
	// client and use MIT-SHM to capture frames from the root window
	// into shared memory.  No /dev/fb0 or dispmanx required.

	Display        *dpy;
	XImage         *ximg;
	XShmSegmentInfo shminfo;

	// Retry for up to 10 s — fbx2 may start before xinit is ready
	for(i=0; i<20; i++) {
		dpy = XOpenDisplay(":0");
		if(dpy) break;
		usleep(500000);
	}
	if(!dpy) err(6, "Can't connect to X display :0");

	if(!XShmQueryExtension(dpy))
		err(7, "X MIT-SHM extension not available");

	int    xscreen  = DefaultScreen(dpy);
	Window root     = RootWindow(dpy, xscreen);
	int    fb_width  = DisplayWidth(dpy, xscreen);
	int    fb_height = DisplayHeight(dpy, xscreen);
	Visual *visual  = DefaultVisual(dpy, xscreen);
	int    depth    = DefaultDepth(dpy, xscreen);

	ximg = XShmCreateImage(dpy, visual, depth, ZPixmap,
	  NULL, &shminfo, fb_width, fb_height);
	if(!ximg) err(8, "XShmCreateImage failed");

	shminfo.shmid = shmget(IPC_PRIVATE,
	  ximg->bytes_per_line * ximg->height, IPC_CREAT | 0777);
	if(shminfo.shmid < 0) err(9, "shmget failed");

	shminfo.shmaddr = ximg->data = (char *)shmat(shminfo.shmid, 0, 0);
	shminfo.readOnly = False;
	XShmAttach(dpy, &shminfo);
	XSync(dpy, False);

	// Downsampled buffer: half the display in each axis.
	// eyes.py uses 640x480 (OLED/TFT) or 1280x720 (IPS).
	int width  = (fb_width  + 1) / 2;
	int height = (fb_height + 1) / 2;

	// Pixel channel offsets from XImage masks — handles XRGB and BGRX
	int rShift = 0, gShift = 0, bShift = 0;
	unsigned long m;
	for(m = ximg->red_mask;   !(m & 1); m >>= 1) rShift++;
	for(m = ximg->green_mask; !(m & 1); m >>= 1) gShift++;
	for(m = ximg->blue_mask;  !(m & 1); m >>= 1) bShift++;

	int offset0 = width * ((height - screenInfo[screenType].height) / 2) +
	             (width / 2 - screenInfo[screenType].width) / 2,
	    offset1 = offset0 + width / 2;

	uint16_t *pixelBuf;
	if(!(pixelBuf = (uint16_t *)malloc(width * height * 2)))
		err(10, "Can't malloc pixelBuf");

	pthread_barrier_init(&barr, NULL, 3);
	uint8_t aa = 0, bb = 1;
	pthread_create(&eye[0].thread, NULL, spiThreadFunc, &aa);
	pthread_create(&eye[1].thread, NULL, spiThreadFunc, &bb);

	// MAIN LOOP -------------------------------------------------------

	uint32_t  frames=0, t, prevTime = time(NULL);
	uint16_t *src0, *dst0, *src1, *dst1;
	int       winCount = 0,
	          w = screenInfo[screenType].width,
	          h = screenInfo[screenType].height;

	for(;;) {
		// Capture current X frame into shared memory
		XShmGetImage(dpy, root, ximg, 0, 0, AllPlanes);

		// 2x2 box filter downsample + XRGB8888 → RGB565 conversion
		{
			uint32_t *src = (uint32_t *)(void *)ximg->data;
			uint32_t p0, p1, p2, p3;
			int r, g, b;
			for(j=0; j<height; j++) {
				for(i=0; i<width; i++) {
					p0 = src[(j*2)   * fb_width + (i*2)  ];
					p1 = src[(j*2)   * fb_width + (i*2+1)];
					p2 = src[(j*2+1) * fb_width + (i*2)  ];
					p3 = src[(j*2+1) * fb_width + (i*2+1)];
					r = (((p0>>rShift)&0xFF)+((p1>>rShift)&0xFF)+
					     ((p2>>rShift)&0xFF)+((p3>>rShift)&0xFF)) >> 2;
					g = (((p0>>gShift)&0xFF)+((p1>>gShift)&0xFF)+
					     ((p2>>gShift)&0xFF)+((p3>>gShift)&0xFF)) >> 2;
					b = (((p0>>bShift)&0xFF)+((p1>>bShift)&0xFF)+
					     ((p2>>bShift)&0xFF)+((p3>>bShift)&0xFF)) >> 2;
					pixelBuf[j * width + i] =
					  ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
				}
			}
		}

		// Crop eye regions and byte-swap for SPI
		j    = 1 - bufIdx;
		src0 = &pixelBuf[offset0];
		src1 = &pixelBuf[offset1];
		dst0 = eye[0].buf[j];
		dst1 = eye[1].buf[j];
		for(j=0; j<h; j++) {
			for(i=0; i<w; i++) {
				dst0[i] = __builtin_bswap16(src0[i]);
				dst1[i] = __builtin_bswap16(src1[i]);
			}
			src0 += width;
			src1 += width;
			dst0 += w;
			dst1 += w;
		}

		pthread_barrier_wait(&barr);

		if(++winCount >= winFrames) {
			commandList(screenInfo[screenType].win);
			setDC(1); // DC high = data
			winCount = 0;
		}

		bufIdx = 1 - bufIdx;
		pthread_barrier_wait(&barr);

		if(showFPS) {
			frames++;
			if((t = time(NULL)) != prevTime) {
				(void)printf("%d fps\n", frames);
				frames   = 0;
				prevTime = t;
			}
		}
	}

	XShmDetach(dpy, &shminfo);
	XDestroyImage(ximg);
	shmdt(shminfo.shmaddr);
	shmctl(shminfo.shmid, IPC_RMID, 0);
	XCloseDisplay(dpy);
	close(gpioFd);
	return 0;
}
