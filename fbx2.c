// Framebuffer-copy-to-two-128x128-screens utility for "Pi Eyes" project.
// Requires a Raspberry Pi Model A+, B+, Zero, Pi 2 or Pi 3 (older models
// lack the auxiliary SPI port and will not work).  Uses two 128x128 pixel
// RGB screens with SPI interface, either:
//  - SSD1351 OLED   www.adafruit.com/products/1431  -or-
//  - ST7735 TFT LCD www.adafruit.com/products/2088 ("green tab" version) 
// Not compatible with other displays, period.

// Enable both the primary and auxiliary SPI devices in /boot/config.txt
// and enable device tree overlay for the aux port (recent Raspbian Jessie
// releases include this overlay, but not enabled by default):
//     dtparam=spi=on
//     dtparam=spi1=on
//     dtoverlay=spi1-3cs
// If a project uses an I2C analog-to-digital converter, also enable that
// interface in /boot/config.txt:
//     dtparam=i2c_arm=on

// Must be run as root (e.g. sudo fbx2), because hardware.  Options:
// -o or -t to select OLED or TFT display
// -b ### to specify bitrate (default is based on screen type)
// -f ### to specify max FPS (default is based on single- or multi-core Pi)
// -s to print FPS while running (default is silent)

// This code works regardless of screen resolution and aspect ratio, but
// ideally should be set for 640x480 pixels, reason is the scaling method
// in the dispmanx library, this res provides best quality interpolation.
// Do this even if no monitor attached; this still configures the
// framebuffer.  In /boot/config.txt:
//     hdmi_force_hotplug=1
//     hdmi_group=2
//     hdmi_mode=87
//     hdmi_cvt=640 480 60 1 0 0 0

// This code runs in the background for an accompanying eye rendering
// application.  This separation allows for new and different custom eye
// renderers to be written in whatever language or library of choice.

// To determine regions copied to each SPI screen: picture two squares,
// side-by-side.  Add padding as needed, top and bottom or left and right,
// so the two squares span the full width of the display (or height, if
// an exceptionally wide aspect ratio).  Within each of the two squares,
// picture an inset square, 80% the size of the two main squares.  Those
// two inset squares are the sections of the display framebuffer that will
// be scaled to 128x128 pixels and issued to the two SPI screens.
// e.g. if framebuffer is 640x480, that's two 320x320 squares side-by-side,
// vertically centered; 80 pixels padding top and bottom.  Areas copied
// to each screen will be 256x256 pixels centered in each 320x320 square
// (320*0.8 = 256), scaled down 2:1 to the 128x128 pixel screens.  Scaling
// provides additional antialiasing for OpenGL, which offers at most 4X
// multisampling (2x2) on Raspberry Pi -- so effectively now 16X (4x4).
// Don't bother with higher res; this yields lesser quality downsampling!
// The eye renderer should follow similar constraints (e.g. 80% inset
// squares) and avoid being resolution-independent.  The 20% inset allows
// the renderer to get away with small artifacts, or for status or
// debugging information to be displayed in the margins, that won't be
// copied to the screens.

// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.
// Insights from Tasanakorn's fbcp tool: github.com/tasanakorn/rpi-fbcp

#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <bcm_host.h>


// CONFIGURATION AND GLOBAL STUFF ------------------------------------------

#define DC_PIN    5             // These pins connect
#define RESET_PIN 6             // to BOTH screens
#define DCMASK    (1 << DC_PIN) // GPIO pin bitmasks of reset + D/C pins
#define RESETMASK (1 << RESET_PIN)

// Main and auxiliary SPI buses are used concurrently, thus MOSI and SCLK
// are unique to each screen, plus CS as expected.  First screen ("right
// eye") connects to SPI0, which is on Broadcom GPIO pins #10 (MOSI), #11
// (SCLK) and #8 (CE0).  Second screen ("left eye") connects to SPI1, on
// GPIO #20 (MOSI), #21 (SCLK) and #16 (CE2).  CE2 is used for 2nd screen
// as it simplified PCB routing

// The following are defaults, most can be overridden via command line.

// Datasheet figures for SPI screen throughput don't always match reality;
// factors like wire length and quality of connections, phase of the moon
// and other mysterious influences play a part...run them too fast and the
// screen will exhibit visual glitches or just not initialize correctly.
// You may need to tweak these numbers for your particular reality, or
// use the -b command-line option to set a maximum bitrate.
#define MAX_OLED_BITRATE 14000000 // Peak SPI rate to OLED w/o glitching
#define MAX_TFT_BITRATE  24000000 // Peak SPI rate to TFT w/o glitching

// The concurrent nature of this code plus the eye renderer (which may be
// performing heavy math) can be taxing, mostly on single-core systems; a
// balance must be established or one task or the other will suffer (and
// frame rates with it).  Limiting the peak frame rate of this code leaves
// some cycles free for the renderer.  These are maximums; actual refresh
// rate will be somewhat less due to bitrates or available cycles!
#define MAX_FPS_PI_1 30 // Max frames/sec on single-core Pi
#define MAX_FPS_PI_2 60 // Max frames/sec on multi-core Pi

static volatile unsigned              // GPIO stuff:
  *gpio = NULL,                       // Memory-mapped GPIO peripheral
  *gpioSet,                           // Write bitmask of GPIO pins to set
  *gpioClr;                           // Write bitmask of GPIO pins to clear

static struct {                       // Per-eye structure:
	int       fd;                 // SPI file descriptor
	uint16_t  buf[2][128 * 128];  // Double-buffered eye data 16 BPP
	pthread_t thread;             // Thread ID of eye's spiThreadFunc()
	struct spi_ioc_transfer xfer; // ioctl() transfer struct
} eye[2];                             // For two eyes

static pthread_barrier_t barr;        // For thread synchronization
static uint8_t bufIdx = 0;            // Double-buffering index
static struct spi_ioc_transfer xfer = {
  .rx_buf        = 0, // ioctl() transfer structure for issuing
  .delay_usecs   = 0, // commands (not pixel data) to both screens.
  .bits_per_word = 8,
  .pad           = 0,
  .tx_nbits      = 0,
  .rx_nbits      = 0,
  .cs_change     = 0 };

// From GPIO example code by Dom and Gert van Loo on elinux.org:
#define PI1_BCM2708_PERI_BASE 0x20000000
#define PI1_GPIO_BASE         (PI1_BCM2708_PERI_BASE + 0x200000)
#define PI2_BCM2708_PERI_BASE 0x3F000000
#define PI2_GPIO_BASE         (PI2_BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE            (4*1024)
#define INP_GPIO(g)          *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)          *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_READ(g)        (*(gpio + 13) & (1<<(g)))


// UTILITY FUNCTIONS -------------------------------------------------------

// Detect Pi board type.  Doesn't return super-granular details,
// just the most basic distinction needed for GPIO compatibility:
// 0: Pi 1 Model B revision 1
// 1: Pi 1 Model B revision 2, Model A, Model B+, Model A+
// 2: Pi 2 Model B (or Pi 3)
static int boardType(void) {
	FILE *fp;
	char  buf[1024], *ptr;
	int   n, board = 1; // Assume Pi1 Rev2 by default

	// Relies on info in /proc/cmdline.  If this becomes unreliable
	// in the future, alt code below uses /proc/cpuinfo if any better.
#if 1
	if((fp = fopen("/proc/cmdline", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "mem_size=")) &&
			   (sscanf(&ptr[9], "%x", &n) == 1) &&
			   (n == 0x3F000000)) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "boardrev=")) &&
			          (sscanf(&ptr[9], "%x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#else
	char s[8];
	if((fp = fopen("/proc/cpuinfo", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "Hardware")) &&
			   (sscanf(&ptr[8], " : %7s", s) == 1) &&
			   (!strcmp(s, "BCM2709"))) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "Revision")) &&
			          (sscanf(&ptr[8], " : %x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#endif

	return board;
}

// Crude error 'handler' (prints message, returns same code as passed in)
static int err(int code, char *string) {
	(void)puts(string);
	return code;
}

#define COMMAND 0 // Values for last argument
#define DATA    1 // to dcX2() function below

// Issue data or command to both SPI displays:
static void dcX2(uint8_t x, uint8_t dc) {
	if(dc) *gpioSet = DCMASK; // 0/low = command, 1/high = data
	else   *gpioClr = DCMASK;
	xfer.tx_buf = (uint32_t)&x; // Uses global xfer struct,
	xfer.len    = 1;            // as most elements don't change
	(void)ioctl(eye[0].fd, SPI_IOC_MESSAGE(1), &xfer);
	(void)ioctl(eye[1].fd, SPI_IOC_MESSAGE(1), &xfer);
}

// Each eye's SPI transfers are handled by a separate thread, to provide
// concurrent non-blocking transfers to both displays while the main thread
// processes the next frame.  Same function is used for both eyes, each in
// its own thread; eye index is passed in.
void *spiThreadFunc(void *data) {
	int y, i = *(uint8_t *)data; // Pass in eye index
	for(;;) {
		// POSIX thread "barriers" are used to sync the main thread
		// with the SPI transfer threads.  This needs to happen at
		// two points: just after finishing the pixel data transfer,
		// and just before starting the next, so that the screen-
		// rectangle commands (which fiddle the shared 'DC' pin)
		// don't corrupt the transfer.  Both barrier waits occur at
		// the *top* of this function to match up with the way the
		// main() loop is entered; it processes a frame before
		// waiting for prior transfers to finish.
		pthread_barrier_wait(&barr); // This is the 'after' wait
		pthread_barrier_wait(&barr); // And the 'before' wait
		for(y=0; y < (128*128); y += (16*128)) {
			eye[i].xfer.tx_buf = (uint32_t)&eye[i].buf[bufIdx][y];
			eye[i].xfer.len    = 4096;
			(void)ioctl(eye[i].fd, SPI_IOC_MESSAGE(1),
			  &eye[i].xfer);
		}
	}
	return NULL;
}


// INIT AND MAIN LOOP ------------------------------------------------------

#define SCREEN_OLED      0 // Compatible screen types,
#define SCREEN_TFT_GREEN 1 // just these two for now.

// Screen initialization commands and data.  Derived from Adafruit Arduino
// libraries, stripped bare here...see corresponding original libraries for
// a more in-depth explanation of each screen command.
static uint8_t *screenInit[] = {
(uint8_t[]) { // OLED INIT; Distilled from Adafruit SSD1351 Arduino lib
  0xFD, 1, 0x12,              // Command lock setting, unlock 1/2
  0xFD, 1, 0xB1,              // Command lock setting, unlock 2/2
  0xAE, 0,                    // Display off
  0xB3, 1, 0xF0,              // Clock div (F1=typical, F0=faster refresh)
  0xCA, 1, 0x7F,              // Duty cycle (128 lines)
  0xA2, 1, 0x00,              // Display offset (0)
  0xA1, 1, 0x00,              // Start line (0)
  0xA0, 1, 0x74,              // Set remap, color depth (5/6/5)
  0xB5, 1, 0x00,              // Set GPIO (disable)
  0xAB, 1, 0x01,              // Function select (internal regulator)
  0xB4, 3, 0xA0, 0xB5, 0x55,  // Set VSL (external)
  0xC1, 3, 0xFF, 0xA3, 0xFF,  // Contrast A/B/C
  0xC7, 1, 0x0F,              // Contrast master (reset)
  0xB1, 1, 0x32,              // Set precharge & discharge
  0xBB, 1, 0x07,              // Precharge voltage of color A/B/C
  0xB2, 3, 0xA4, 0x00, 0x00,  // Display enhanvement
  0xB6, 1, 0x01,              // Precharge period
  0xBE, 1, 0x05,              // Set VcomH (0.82 x Vcc)
  0xA6, 0,                    // Normal display
  0xAF, 0,                    // Display on
  0x00 },                     // EOD
(uint8_t[]) { // TFT INIT; from Adafruit ST7735 Arduino lib ('green tab')
  0x01, 0x80, 150,            // Software reset, 0 args, w/150ms delay
  0x11, 0x80, 255,            // Out of sleep mode, 0 args, w/500ms delay
  0xB1, 3,                    // Frame rate ctrl - normal mode, 3 args:
    0x01, 0x2C, 0x2D,         // Rate = fosc/(1x2+40) * (LINE+2C+2D)
  0xB2, 3,                    // Frame rate control - idle mode, 3 args:
    0x01, 0x2C, 0x2D,         // Rate = fosc/(1x2+40) * (LINE+2C+2D)
  0xB3, 6,                    // Frame rate ctrl - partial mode, 6 args:
    0x01, 0x2C, 0x2D,         // Dot inversion mode
    0x01, 0x2C, 0x2D,         // Line inversion mode
  0xB4, 1, 0x07,              // Display inversion ctrl: no inversion
  0xC0, 3,                    // Power control 1, 3 args, no delay:
    0xA2, 0x02, 0x84,         // -4.6V, AUTO mode
  0xC1, 1, 0xC5,              // Pwr ctrl 2: VGH25=2.4C VGSEL=-10 VGH=3*AVDD
  0xC2, 2, 0x0A, 0x00,        // Pwr ctrl 3: opamp current small, boost freq
  0xC3, 2, 0x8A, 0x2A,        // Pwr ctrl 4: BCLK/2, Opamp small & med low
  0xC4, 2, 0x8A, 0xEE,        // Power control 5, 2 args, no delay
  0xC5, 1, 0x0E,              // Power control, 1 arg, no delay
  0x20, 0,                    // Don't invert display, no args, no delay
  0x36, 1, 0xC8,              // MADCTL: row addr/col addr, bottom-to-top
  0x3A, 1, 0x05,              // set color mode, 1 arg: 16-bit color
  0x2A, 4,                    // Column addr set, 4 args, no delay:
    0x00, 0x00, 0x00, 0x7F,   // XSTART = 0, XEND = 127
  0x2B, 4,                    // Row addr set, 4 args, no delay:
    0x00, 0x00, 0x00, 0x7F,   // XSTART = 0, XEND = 127
  0xE0, 16,                   // ???, 16 args, no delay:
    0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
    0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
  0xE1, 16,                   // ???, 16 args, no delay:
    0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
    0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
  0x13, 0x80, 10,             // Normal display on, no args, w/10ms delay
  0x29, 0x80, 100,            // Main screen turn on, no args w/100ms delay
  0x00 } };                   // EOD

int main(int argc, char *argv[]) {

	uint8_t screenType = SCREEN_OLED, // SCREEN_OLED or SCREEN_TFT_GREEN
	        isPi2      = 0,           // Will set to 1 if multi-core
	        showFPS    = 0;
	int     maxBitrate=0, maxFPS=0,   // If 0, use defaults
	        i, j, fd, fpsBitrate, finalBitrate;

	while((i = getopt(argc, argv, "otb:f:s")) != -1) {
		switch(i) {
		   case 'o': // Select OLED screen type
			screenType = SCREEN_OLED;
			break;
		   case 't': // Select TFT screen type
			screenType = SCREEN_TFT_GREEN;
			break;
			break;
		   case 'b': // Max bitrate
			maxBitrate = strtol(optarg, NULL, 0);
			break;
		   case 'f': // Max frames/second
			maxFPS = strtol(optarg, NULL, 0);
			break;
		   case 's': // Show FPS
			showFPS = 1;
			break;
		}
	}

	isPi2 = (boardType() == 2);
	if(!maxFPS) maxFPS = isPi2 ? MAX_FPS_PI_2 : MAX_FPS_PI_1;
	if(!maxBitrate) {
		maxBitrate = (screenType == SCREEN_OLED) ?
		  MAX_OLED_BITRATE : MAX_TFT_BITRATE;
	}

	// To meet frame rate and bitrate limits, rather than introducing
	// more code complexity with alarm signals or CPU-intensive time
	// polling, we exploit the inherent delay of SPI blocking
	// transfers...intentionally limiting bitrates to less than the
	// maximum, this FPS throttling comes free.  Determine bitrate
	// needed to achieve maxFPS, then take the lesser of this or
	// maxBitrate.
	fpsBitrate   = ((128 * 128 * 16) * maxFPS); // 128x128 16-bit pixels
	finalBitrate = (fpsBitrate < maxBitrate) ? fpsBitrate : maxBitrate;

	// GPIO AND OLED SCREEN INIT ---------------------------------------

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		return err(1, "Can't open /dev/mem (try 'sudo')\n");
	}
	gpio = (volatile unsigned *)mmap( // Memory-map I/O
	  NULL,                 // Any adddress will do
	  BLOCK_SIZE,           // Mapped block length
	  PROT_READ|PROT_WRITE, // Enable read+write
	  MAP_SHARED,           // Shared w/other processes
	  fd,                   // File to map
	  isPi2 ?
	  PI2_GPIO_BASE :      // -> GPIO registers
	  PI1_GPIO_BASE);
	close(fd);              // Not needed after mmap()
	if(gpio == MAP_FAILED) {
		return err(2, "Can't mmap()");
	}
	gpioSet = &gpio[7];
	gpioClr = &gpio[10];

	if(((eye[0].fd = open("/dev/spidev0.0", O_WRONLY|O_NONBLOCK)) < 0) ||
	   ((eye[1].fd = open("/dev/spidev1.2", O_WRONLY|O_NONBLOCK)) < 0)) {
		return err(3, "spiOpen() failed");
	}

	INP_GPIO(DC_PIN);    OUT_GPIO(DC_PIN); // Must INP before OUT
	INP_GPIO(RESET_PIN); OUT_GPIO(RESET_PIN);


	xfer.speed_hz = finalBitrate;
	uint8_t  mode = SPI_MODE_0;
	for(i=0; i<2; i++) {
		ioctl(eye[i].fd, SPI_IOC_WR_MODE, &mode);
		ioctl(eye[i].fd, SPI_IOC_WR_MAX_SPEED_HZ, finalBitrate);
		memcpy(&eye[i].xfer, &xfer, sizeof(xfer));
	}

	*gpioSet = RESETMASK; usleep(5); // Reset high,
	*gpioClr = RESETMASK; usleep(5); // low,
	*gpioSet = RESETMASK; usleep(5); // high

	// Initialize SPI screens
	if(screenType == SCREEN_OLED) {
		for(i=0;;) {
			if(!(j=screenInit[screenType][i++])) break;
			dcX2(j, COMMAND);
			for(j=screenInit[screenType][i++]; j; j--)
				dcX2(screenInit[screenType][i++], DATA);
		}
		dcX2(0xB8, COMMAND); // Gamma table
		for(i=0; i<64; i++) {
			dcX2((int)(pow((float)i/63.0, 0.75) * 179.0 + 0.5),
			  DATA);
		}
	} else {
		int ms;
		for(i=0;;) {
			if(!(j=screenInit[screenType][i++])) break;
			dcX2(j, COMMAND);
			j  = screenInit[screenType][i++]; // # args
			ms = j & 0x80; // 0x80 = delay flag
			j &= ~0x80;
			while(j--) dcX2(screenInit[screenType][i++], DATA);
			if(ms) {
				ms = screenInit[screenType][i++];
				if(ms = 255) ms = 500;
				usleep(ms * 1000);
			}
		}
	}

	// DISPMANX INIT ---------------------------------------------------

	// Insights gained from Tasanakorn's fbcp utility:
	// https://github.com/tasanakorn/rpi-fbcp
	// Rather than copying framebuffer-to-framebuffer, this code
	// issues screen data directly and concurrently to two 'raw'
	// SPI displays (no secondary framebuffer device / driver).

	DISPMANX_DISPLAY_HANDLE_T  display; // Primary framebuffer display
	DISPMANX_MODEINFO_T        info;    // Screen dimensions, etc.
	DISPMANX_RESOURCE_HANDLE_T screen_resource; // Intermediary buf
	uint32_t                   handle;
	VC_RECT_T                  rect;
	uint16_t                  *pixelBuf;

	bcm_host_init();

	if(!(display = vc_dispmanx_display_open(0))) {
		return err(4, "Can't open primary display");
	}

	if(vc_dispmanx_display_get_info(display, &info)) {
		return err(5, "Can't get primary display information");
	}

	// info.width and info.height are primary display dimensions.
	// Create a 16-bit (5/6/5) offscreen resource with the same aspect
	// ratio as framebuffer, only smaller: no less than 320 pixels wide
	// (height proportional to framebuffer), or 160 pixels high (width
	// proportional to framebuffer).  This intentionally creates some
	// padding around the areas copied to the SPI screens, so that
	// status/debugging info can be displayed in the margins but won't
	// be copied to the screens.

	int width, height; // Resource dimensions

	// Also determine positions of upper-left corners for the two
	// SPI screens, and corresponding offsets into pixelBuf[].
	// Rendering application will need to observe similar size and
	// position constraints to produce desired results.
	int offset0, offset1, x, y;

	if(info.width <= (info.height * 2)) {
		// Framebuffer is <= 2:1 aspect ratio (e.g. 4:3 or 16:9)
		width   = 320;
		height  = 320 * info.height / info.width;
		y       = height / 2 - 64;
		offset0 = y * width + width     / 4 - 64,
		offset1 = y * width + width * 3 / 4 - 64;
	} else {
		// Framebuffer is > 2:1 aspect ratio (e.g. 21:9)
		width   = 160 * info.width / info.height;
		height  = 160;
		y       = height / 2 - 64;
		offset0 = y * width + (width - height) / 2 - 64;
		offset1 = y * width + (width + height) / 2 - 64;
	}

	// screen_resource is an intermediary between framebuffer and
	// main RAM -- VideoCore will copy the primary framebuffer
	// contents to this resource while providing interpolated
	// scaling plus 8/8/8 -> 5/6/5 dithering.
	if(!(screen_resource = vc_dispmanx_resource_create(
	  VC_IMAGE_RGB565, width, height, &handle))) {
		vc_dispmanx_display_close(display);
		return err(6, "Can't create screen buffer");
	}
	vc_dispmanx_rect_set(&rect, 0, 0, width, height);

	// Create a buffer in RAM w/same dimensions as offscreen
	// resource, 16 bits per pixel.
	if(!(pixelBuf = (uint16_t *)malloc(width * height * 2))) {
		vc_dispmanx_display_close(display);
		return err(7, "Can't malloc");
	}

	// Initialize SPI transfer threads and synchronization barrier
	pthread_barrier_init(&barr, NULL, 3);
	uint8_t aa = 0, bb = 1;
	pthread_create(&eye[0].thread, NULL, spiThreadFunc, &aa);
	pthread_create(&eye[1].thread, NULL, spiThreadFunc, &bb);

	// MAIN LOOP -------------------------------------------------------

	uint32_t frames=0, t, prevTime = time(NULL);

	for(;;) {

		// Framebuffer -> scale & dither to intermediary
		vc_dispmanx_snapshot(display, screen_resource, 0);
		// Intermediary -> main RAM
		vc_dispmanx_resource_read_data(screen_resource, &rect,
		  pixelBuf, width * 2);

		// Crop & transfer rects to eye buffers, flip hi/lo bytes
		j = 1 - bufIdx; // Render to 'back' buffer
		for(y=0; y<128; y++) {
			// HHLL -> HHLLHHLL -> HHLLHH -> LLHH
			for(x=0; x<128; x++) {
				eye[0].buf[j][y * 128 + x] =
				  (pixelBuf[offset0 + y * width + x] *
				  0x00010001) >> 8;
				eye[1].buf[j][y * 128 + x] =
				  (pixelBuf[offset1 + y * width + x] *
				  0x00010001) >> 8;
			}
		}

		// Sync up all threads; wait for prior transfers to finish
		pthread_barrier_wait(&barr);

		// Before pushing data to SPI screens, their column and
		// row ranges are reset every frame to force screen data
		// pointer back to (0,0).  Though the pointer will
		// automatically 'wrap' when the end of the screen is
		// reached, this is extra insurance in case there's a
		// glitch where a byte doesn't get through to one or both
		// displays (which would then be out of sync in all
		// subsequent frames).
		if(screenType == SCREEN_OLED) {
			dcX2(0x15, COMMAND);
			dcX2(0x00, DATA); dcX2(0x7F, DATA);
			dcX2(0x75, COMMAND);
			dcX2(0x00, DATA); dcX2(0x7F, DATA);
			dcX2(0x5C, COMMAND); // Write to display RAM
		} else {
			int colstart = 2;
			int rowstart = 3;

			dcX2(0x2A, COMMAND); // Column set
			dcX2(0x00, DATA);
			dcX2(colstart, DATA);
			dcX2(0x00, DATA);
			dcX2(127 + colstart, DATA);

			dcX2(0x2B, COMMAND); // Row set
			dcX2(0x00, DATA);
			dcX2(rowstart, DATA);
			dcX2(0x00, DATA);
			dcX2(127 + rowstart, DATA);

			dcX2(0x2C, COMMAND); // RAM write
		}

		*gpioSet = DCMASK;     // DC high
		bufIdx   = 1 - bufIdx; // Swap buffers

		// With screen commands now issued, sync up the threads
		// again, they'll start pushing data...
		pthread_barrier_wait(&barr);

		if(showFPS) {
			// Show approximate frames-per-second once per
			// second.  This is the copy speed of the fbx2
			// code and is disengaged from the eye-rendering
			// application, which will be operating at its
			// own unrelated refresh rate.
			frames++;
			if((t = time(NULL)) != prevTime) {
				(void)printf("%d fps\n", frames);
				frames   = 0;
				prevTime = t;
			}
		}
	}

	vc_dispmanx_resource_delete(screen_resource);
	vc_dispmanx_display_close(display);
	return 0;
}
