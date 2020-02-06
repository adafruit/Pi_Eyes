// Framebuffer-copy-to-two-SPI-screens utility for "Pi Eyes" project.
// Requires a Raspberry Pi Model A+, B+, Zero, Pi 2 or Pi 3 (older models
// lack the auxiliary SPI port and will not work).  Uses two RGB screens
// with SPI interface, either:
//  - SSD1351 OLED   www.adafruit.com/products/1431  -or-
//  - ST7789 IPS TFT www.adafruit.com/products/3787
//  - ST7735 TFT LCD www.adafruit.com/products/2088 ("green tab" version)
// NOT COMPATIBLE WITH OTHER DISPLAYS, PERIOD.

// Enable both the primary and auxiliary SPI devices in /boot/config.txt
// and enable device tree overlay for the aux port (recent Raspbian Jessie
// releases include this overlay, but not enabled by default):
//     dtparam=spi=on
//     dtparam=spi1=on
//     dtoverlay=spi1-3cs
// If a project uses an I2C analog-to-digital converter, also enable that
// interface in /boot/config.txt:
//     dtparam=i2c_arm=on
// Increase the spidev buffer size to 8K by appending to /boot/cmdline.txt:
//     spidev.bufsiz=8192
// The latter improves frame rates and avoids some spi1 (2nd eye) glitching!
// THE ABOVE STEPS ARE ALL HANDLED BY THE pi-eyes.sh INSTALLER SCRIPT HERE:
// https://github.com/adafruit/Raspberry-Pi-Installer-Scripts

// Must be run as root (e.g. sudo fbx2), because hardware.  Options:
// -o, -t or -i to select OLED, TFT or IPS display
// -b ### to specify bitrate (default is based on screen type)
// -w ### to specify number of frames between pixel window writes (default 1)
// -s to print FPS while running (default is silent)

// This code works regardless of screen resolution and aspect ratio, but
// ideally should be set for 640x480 pixels for 128x128 screens (OLED or
// TFT), or 1280x720 for 240x240 screens (IPS) for optimal interpolation.
// Do this even if no monitor attached; this still configures the
// framebuffer.  In /boot/config.txt:
//     hdmi_force_hotplug=1
//     hdmi_group=2
//     hdmi_mode=87
//     hdmi_cvt=640 480 60 1 0 0 0
// or
//     hdmi_cvt=1280 720 60 1 0 0 0
// (Again, the pi-eyes.sh installer script takes care of this.)

// This code runs in the background for an accompanying eye rendering
// application.  This separation allows for new and different custom eye
// renderers to be written in whatever language or library of choice.

// To determine regions copied to each SPI screen: picture the screen
// divided in half, two equal regions side-by-side.  Centered within each
// region, a 256x256 pixel square (for OLED and TFT) is scaled to 50%
// (with 2x2 area filtering) to produce 128x128 bitmaps send to each
// screen.  For IPS screens, the squares are 480x480 pixels and scaled to
// 240x240.  The framebuffer size and the eye-rendering code both need
// some configuration to work properly with all this; it's not entirely
// automagic (earlier versions tried that, but certain screen sizes caused
// problems with the dispmanx functions, so it needs to be a little more
// manual for now).  The 2x2 filtering provides additional antialiasing
// for OpenGL, which offers at most 4X multisampling (2x2) on Raspberry Pi,
// so effectively now 16X (4x4).  Don't bother with higher res; this yields
// lesser quality downsampling!

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
// eye" -- positioned on the left from observer's POV) connects to SPI0,
// which is on Broadcom GPIO pins #10 (MOSI), #11 (SCLK) and #8 (CE0).
// Second screen ("left eye" -- positioned on right) connects to SPI1, on
// GPIO #20 (MOSI), #21 (SCLK) and #16 (CE2).  CE2 (rather than CE1) is
// used for 2nd screen as it simplified PCB routing

#define SCREEN_OLED      0 // Compatible screen types
#define SCREEN_TFT_GREEN 1 // Just these three
#define SCREEN_IPS       2 // Other types WILL NOT WORK!

uint8_t screenType = SCREEN_OLED;

// Screen initialization commands and data.  Derived from Adafruit Arduino
// libraries, stripped bare here...see corresponding original libraries for
// a more in-depth explanation of each screen command.
static const uint8_t initOLED[] = {
  // OLED initialization distilled from Adafruit SSD1351 Arduino library
  0xFD,  1, 0x12,             // Command lock setting, unlock 1/2
  0xFD,  1, 0xB1,             // Command lock setting, unlock 2/2
  0xAE,  0,                   // Display off
  0xB3,  1, 0xF0,             // Clock div (F1=typical, F0=faster refresh)
  0xCA,  1, 0x7F,             // Duty cycle (128 lines)
  0xA2,  1, 0x00,             // Display offset (0)
  0xA1,  1, 0x00,             // Start line (0)
  0xA0,  1, 0x74,             // Set remap, color depth (5/6/5)
  0xB5,  1, 0x00,             // Set GPIO (disable)
  0xAB,  1, 0x01,             // Function select (internal regulator)
  0xB4,  3, 0xA0, 0xB5, 0x55, // Set VSL (external)
  0xC1,  3, 0xFF, 0xA3, 0xFF, // Contrast A/B/C
  0xC7,  1, 0x0F,             // Contrast master (reset)
  0xB1,  1, 0x32,             // Set precharge & discharge
  0xBB,  1, 0x07,             // Precharge voltage of color A/B/C
  0xB2,  3, 0xA4, 0x00, 0x00, // Display enhanvement
  0xB6,  1, 0x01,             // Precharge period
  0xBE,  1, 0x05,             // Set VcomH (0.82 x Vcc)
  0xA6,  0,                   // Normal display
  0xAF,  0,                   // Display on
  0xB8, 64,                   // Gamma table, 64 values, no delay
    0x00, 0x08, 0x0D, 0x12, 0x17, 0x1B, 0x1F, 0x22,
    0x26, 0x2A, 0x2D, 0x30, 0x34, 0x37, 0x3A, 0x3D,
    0x40, 0x43, 0x46, 0x49, 0x4C, 0x4F, 0x51, 0x54,
    0x57, 0x59, 0x5C, 0x5F, 0x61, 0x64, 0x67, 0x69,
    0x6C, 0x6E, 0x71, 0x73, 0x76, 0x78, 0x7B, 0x7D,
    0x7F, 0x82, 0x84, 0x86, 0x89, 0x8B, 0x8D, 0x90,
    0x92, 0x94, 0x97, 0x99, 0x9B, 0x9D, 0x9F, 0xA2,
    0xA4, 0xA6, 0xA8, 0xAA, 0xAD, 0xAF, 0xB1, 0xB3,
  0x00 },                     // EOD
initTFT[] = {
  // TFT initialization from Adafruit ST7735 Arduino library ('green tab')
  0x01, 0x80, 150,            // Software reset, 0 args, w/150ms delay
  0x11, 0x80, 255,            // Out of sleep mode, 0 args, w/500ms delay
  0xB1,    3,                 // Frame rate ctrl - normal mode, 3 args:
    0x01, 0x2C, 0x2D,         // Rate = fosc/(1x2+40) * (LINE+2C+2D)
  0xB2,    3,                 // Frame rate control - idle mode, 3 args:
    0x01, 0x2C, 0x2D,         // Rate = fosc/(1x2+40) * (LINE+2C+2D)
  0xB3,    6,                 // Frame rate ctrl - partial mode, 6 args:
    0x01, 0x2C, 0x2D,         // Dot inversion mode
    0x01, 0x2C, 0x2D,         // Line inversion mode
  0xB4, 1, 0x07,              // Display inversion ctrl: no inversion
  0xC0,    3,                 // Power control 1, 3 args, no delay:
    0xA2, 0x02, 0x84,         // -4.6V, AUTO mode
  0xC1,    1, 0xC5,           // Pwr ctrl 2: VGH25=2.4C VGSEL=-10 VGH=3*AVDD
  0xC2,    2, 0x0A, 0x00,     // Pwr ctrl 3: opamp current small, boost freq
  0xC3,    2, 0x8A, 0x2A,     // Pwr ctrl 4: BCLK/2, Opamp small & med low
  0xC4,    2, 0x8A, 0xEE,     // Power control 5, 2 args, no delay
  0xC5,    1, 0x0E,           // Power control, 1 arg, no delay
  0x20,    0,                 // Don't invert display, no args, no delay
  0x36,    1, 0xC8,           // MADCTL: row addr/col addr, bottom-to-top
  0x3A,    1, 0x05,           // set color mode, 1 arg: 16-bit color
  0x2A,    4,                 // Column addr set, 4 args, no delay:
    0x00, 0x00, 0x00, 0x7F,   // XSTART = 0, XEND = 127
  0x2B,    4,                 // Row addr set, 4 args, no delay:
    0x00, 0x00, 0x00, 0x7F,   // XSTART = 0, XEND = 127
  0xE0,   16,                 // ???, 16 args, no delay:
    0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
    0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
  0xE1,   16,                 // ???, 16 args, no delay:
    0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
    0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
  0x13, 0x80, 10,             // Normal display on, no args, w/10ms delay
  0x29, 0x80, 100,            // Main screen turn on, no args w/100ms delay
  0x00 },                     // EOD
initIPS[] = {
  // IPS initialization
  0x01, 0x80,       150,      // Soft reset, no args, 150 ms delay
  0x11, 0x80,       255,      // Out of sleep, no args, 500 ms delay
  0x3A, 0x81, 0x55,  10,      // COLMOD, 1 arg, 10ms delay
  0x36,    1, 0x00,           // MADCTL, 1 arg (RGB), no delay,
  0x26,    1, 0x02,           // GAMSET, 1 arg (curve 2 (G1.8)), no delay
  0xBA,    1, 0x04,           // DGMEN, 1 arg (enable gamma), no delay,
  0x21, 0x80,        10,      // INVON, no args, 10 ms delay
  0x13, 0x80,        10,      // NORON, no args, 10 ms delay
  0x29, 0x80,       255,      // DISPON, no args, 500 ms delay
  0x00 },
winOLED[] = {
  0x15, 2, 0x00, 0x7F,         // Column range
  0x75, 2, 0x00, 0x7F,         // Row range
  0x5C,                        // Write to display RAM
  0x00 },                      // EOD
winTFT[] = {
  0x2A, 4, 0, 2, 0, 129,       // Column set, xtart, xend (MSB first)
  0x2B, 4, 0, 3, 0, 130,       // Row set, ystart, yend (MSB first)
  0x2C,                        // RAM write
  0x00 },                      // EOD
winIPS[] = {
  0x2A, 4, 0, 0, 0, 239,       // CASET (column set) xstart, xend (MSB first)
  0x2B, 4, 0, 0, 0, 239,       // RASET (row set) ystart, yend (MSB first)
  0x2C,                        // RAMWR (RAM write)
  0x00 };                      // EOD

// Further data specific to each screen type: pixel dimensions, maximum
// stable SPI bitrate, pointer to initialization commands above.
// Datasheet figures for SPI screen throughput don't always match reality;
// factors like wire length and quality of connections, phase of the moon
// and other mysterious influences play a part...run them too fast and the
// screen will exhibit visual glitches or just not initialize correctly.
// You may need to use the -b command-line option to set the bitrate.
static const struct {
	const int      width;   // Width in pixels
	const int      height;  // Height in pixels
	const int      bitrate; // Default stable SPI bitrate
	const uint8_t *init;    // Pointer to initialization command list
	const uint8_t *win;     // Pointer to window command list
} screen[] = {
  { 128, 128, 10000000, initOLED, winOLED },
  { 128, 128, 12000000, initTFT , winTFT  },
  { 240, 240, 80000000, initIPS , winIPS  } };

// The concurrent nature of this code plus the eye renderer (which may be
// performing heavy math) can be taxing, mostly on single-core systems; a
// balance must be established or one task or the other will suffer (and
// frame rates with it).  Limiting the peak frame rate of this code can
// be accomplished by selecting a lower SPI bitrate.

// Per-eye structure
static struct {
	int        fd;                // SPI file descriptor
	uint16_t  *buf[2];            // Double-buffered eye data 16 BPP
	pthread_t  thread;            // Thread ID of eye's spiThreadFunc()
	struct spi_ioc_transfer xfer; // ioctl() transfer struct
} eye[2];

static pthread_barrier_t barr;          // For thread synchronization
static uint8_t           bufIdx = 0;    // Double-buffering index
static int               bufsiz = 4096; // SPI block xfer size (4K default)
static struct spi_ioc_transfer xfer = {
  .rx_buf        = 0, // ioctl() transfer structure for issuing
  .delay_usecs   = 0, // commands (not pixel data) to both screens.
  .bits_per_word = 8,
  .pad           = 0,
  .tx_nbits      = 0,
  .rx_nbits      = 0,
  .cs_change     = 0 };

// From GPIO example code by Dom and Gert van Loo on elinux.org:
#define GPIO_BASE    0x200000
#define BLOCK_SIZE   (4*1024)
#define INP_GPIO(g)  *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)  *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_READ(g) (*(gpio + 13) & (1<<(g)))

static volatile unsigned
  *gpio = NULL, // Memory-mapped GPIO peripheral
  *gpioSet,     // Write bitmask of GPIO pins to set
  *gpioClr;     // Write bitmask of GPIO pins to clear


// UTILITY FUNCTIONS -------------------------------------------------------

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

// Issue a list of commands (and arguments, delays) to both displays:
static void commandList(const uint8_t *ptr) { // pass in -> command list
	int i, j, ms;
	for(i=0; (j=ptr[i++]);) {                // 0 = EOD
		dcX2(j, COMMAND);                // First byte = command
		j  = ptr[i++];                   // Delay flag | num args
		ms = j & 0x80;                   // Mask delay flag
		j &= ~0x80;                      // Mask arg count
		while(j--) dcX2(ptr[i++], DATA); // Issue args (data)
		if(ms) {                         // Delay flag set?
			ms = ptr[i++];           // Next byte = milliseconds
			if(ms == 255) ms = 500;  // If 255, make it 500
			usleep(ms * 1000);
		}
	}
}

// Each eye's SPI transfers are handled by a separate thread, to provide
// concurrent non-blocking transfers to both displays while the main thread
// processes the next frame.  Same function is used for both eyes, each in
// its own thread; eye index is passed in.
void *spiThreadFunc(void *data) {
	int      i = *(uint8_t *)data; // Pass in eye index
	uint32_t bytesThisPass, bytesToGo, screenBytes =
	  screen[screenType].width * screen[screenType].height * 2;

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

		eye[i].xfer.tx_buf = (uint32_t)eye[i].buf[bufIdx];
		bytesToGo = screenBytes;
		do {
			bytesThisPass = bytesToGo;
			if(bytesThisPass > bufsiz) bytesThisPass = bufsiz;
			eye[i].xfer.len = bytesThisPass;
			(void)ioctl(eye[i].fd, SPI_IOC_MESSAGE(1),
			  &eye[i].xfer);
			eye[i].xfer.tx_buf += bytesThisPass;
			bytesToGo          -= bytesThisPass;
		} while(bytesToGo > 0);
	}
	return NULL;
}

// Crude error handler (prints message, exits program with status code)
static int err(int code, char *string) {
	(void)puts(string);
	exit(code);
}


// INIT AND MAIN LOOP ------------------------------------------------------

int main(int argc, char *argv[]) {

	uint8_t showFPS   = 0;
	int     bitrate   = 0, // If 0, use default
	        winFrames = 1, // How often to reset pixel window
	        i, j, fd;

	while((i = getopt(argc, argv, "otib:w:s")) != -1) {
		switch(i) {
		   case 'o': // Select OLED screen type
			screenType = SCREEN_OLED;
			break;
		   case 't': // Select TFT screen type
			screenType = SCREEN_TFT_GREEN;
			break;
		   case 'i': // Select IPS screen type
			screenType = SCREEN_IPS;
			break;
		   case 'b': // SPI bitrate
			bitrate = strtol(optarg, NULL, 0);
			break;
		   case 'w': // Number of frames between window sync
			winFrames = strtol(optarg, NULL, 0);
			break;
		   case 's': // Show FPS
			showFPS = 1;
			break;
		}
	}

	if(!bitrate) bitrate = screen[screenType].bitrate;

	// Get SPI buffer size from sysfs.  Default is 4K.
	FILE *fp;
	if((fp = fopen("/sys/module/spidev/parameters/bufsiz", "r"))) {
		if(fscanf(fp, "%d", &i) == 1) bufsiz = i;
		fclose(fp);
	}

	// GPIO AND SCREEN INIT --------------------------------------------

	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
		err(1, "Can't open /dev/mem (try 'sudo')");
	gpio = (volatile unsigned *)mmap( // Memory-map I/O
	  NULL,                 // Any adddress will do
	  BLOCK_SIZE,           // Mapped block length
	  PROT_READ|PROT_WRITE, // Enable read+write
	  MAP_SHARED,           // Shared w/other processes
	  fd,                   // File to map
	  bcm_host_get_peripheral_address() + GPIO_BASE);
	close(fd);              // Not needed after mmap()
	if(gpio == MAP_FAILED) {
		err(2, "mmap() failed");
	}
	gpioSet = &gpio[7];
	gpioClr = &gpio[10];

	if((eye[0].fd = open("/dev/spidev0.0", O_WRONLY|O_NONBLOCK)) < 0)
		err(3, "Can't open spidev0.0, is SPI enabled?");
	if((eye[1].fd = open("/dev/spidev1.2", O_WRONLY|O_NONBLOCK)) < 0)
		err(4, "Can't open spidev1.2, is spi1-3cs overlay enabled?");

	xfer.speed_hz = bitrate;
	uint8_t  mode = SPI_MODE_0;
	for(i=0; i<2; i++) {
		ioctl(eye[i].fd, SPI_IOC_WR_MODE, &mode);
		ioctl(eye[i].fd, SPI_IOC_WR_MAX_SPEED_HZ, bitrate);
		memcpy(&eye[i].xfer, &xfer, sizeof(xfer));
		for(j=0; j<2; j++) {
			if(NULL == (eye[i].buf[j] = (uint16_t *)malloc(
			  screen[screenType].width *
			  screen[screenType].height * sizeof(uint16_t)))) {
				err(5, "Eye buffer malloc failed");
			}
		}
	}

	// INITIALIZE SPI SCREENS ------------------------------------------

	INP_GPIO(DC_PIN);    OUT_GPIO(DC_PIN); // Must INP before OUT
	INP_GPIO(RESET_PIN); OUT_GPIO(RESET_PIN);

	*gpioSet = RESETMASK; usleep(5); // Reset high,
	*gpioClr = RESETMASK; usleep(5); // low,
	*gpioSet = RESETMASK; usleep(5); // high

	commandList(screen[screenType].init); // Send init commands

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
		err(6, "Can't open primary display");
	}

	if(vc_dispmanx_display_get_info(display, &info)) {
		err(7, "Can't get primary display information");
	}

	// info.width and info.height are primary display dimensions.
	// Create a 16-bit (5/6/5) offscreen resource that's 1/2 the display
	// size.  GPU bilinear interpolation then yields 2x2 pixel averaging.
	// Note that some resource constraints exist but possibly not
	// documented -- appears that width needs to be a multiple of 32?
	// Not entirely sure.  Point is, while the framebuffer resolution
	// is extremely flexible, not all resolutions will work here, and
	// it may require some configuration, testing and reboots.

	int width  = (info.width  + 1) / 2, // Resource dimensions
	    height = (info.height + 1) / 2;

	// Also determine positions of upper-left corners for the two
	// SPI screens, and corresponding offsets into pixelBuf[].
	// Rendering application will need to observe similar size and
	// position constraints to produce desired results.
	int x, y,
	    offset0 = width * ((height - screen[screenType].height) / 2) +
	             (width / 2 - screen[screenType].width) / 2,
	    offset1 = offset0 + width / 2;

	// screen_resource is an intermediary between framebuffer and
	// main RAM -- VideoCore will copy the primary framebuffer
	// contents to this resource while providing interpolated
	// scaling plus 8/8/8 -> 5/6/5 dithering.
	if(!(screen_resource = vc_dispmanx_resource_create(
	  VC_IMAGE_RGB565, width, height, &handle))) {
		vc_dispmanx_display_close(display);
		err(8, "Can't create screen buffer");
	}
	vc_dispmanx_rect_set(&rect, 0, 0, width, height);

	// Create a buffer in RAM w/same dimensions as offscreen
	// resource, 16 bits per pixel.
	if(!(pixelBuf = (uint16_t *)malloc(width * height * 2))) {
		vc_dispmanx_display_close(display);
		err(9, "Can't malloc pixelBuf");
	}

	// Initialize SPI transfer threads and synchronization barrier
	pthread_barrier_init(&barr, NULL, 3);
	uint8_t aa = 0, bb = 1;
	pthread_create(&eye[0].thread, NULL, spiThreadFunc, &aa);
	pthread_create(&eye[1].thread, NULL, spiThreadFunc, &bb);

	// MAIN LOOP -------------------------------------------------------

	uint32_t  frames=0, t, prevTime = time(NULL);
	uint16_t *src0, *dst0, *src1, *dst1;
	int       winCount = 0,
	          w = screen[screenType].width,
	          h = screen[screenType].height;

	for(;;) {

		// Framebuffer -> scale & dither to intermediary
		vc_dispmanx_snapshot(display, screen_resource, 0);
		// Intermediary -> main RAM (tried doing some stuff with
		// an 'optimal' bounding rect but it just crashed & burned,
		// so doing full-screen transfers for now).
		vc_dispmanx_resource_read_data(screen_resource, &rect,
		  pixelBuf, width * 2);

		// Crop & transfer rects to eye buffers, flip hi/lo bytes
		j    = 1 - bufIdx; // Render to 'back' buffer
		src0 = &pixelBuf[offset0];
		src1 = &pixelBuf[offset1];
		dst0 = eye[0].buf[j];
		dst1 = eye[1].buf[j];
		for(y=0; y<h; y++) {
			for(x=0; x<w; x++) {
				dst0[x] = __builtin_bswap16(src0[x]);
				dst1[x] = __builtin_bswap16(src1[x]);
			}
			src0 += width;
			src1 += width;
			dst0 += w;
			dst1 += w;
		}

		// Sync up all threads; wait for prior transfers to finish
		pthread_barrier_wait(&barr);

		// Before pushing data to SPI screens, the pixel 'window'
		// is periodically reset to force screen data pointer back
		// to (0,0).  The pointer automatically 'wraps' when the end
		// of the screen is reached, but a periodic reset provides
		// extra insurance in case of SPI glitches (which would
		// put one or both screens out of sync for all subsequent
		// frames).  Default behavior is to reset on every frame
		// (performance difference is negligible).
		if(++winCount >= winFrames) {
			commandList(screen[screenType].win);
			*gpioSet = DCMASK; // DC high (data)
			winCount = 0;
		}

		// With screen commands now issued, sync up the
		// threads again, they'll start pushing data...
		bufIdx   = 1 - bufIdx;       // Swap buffers
		pthread_barrier_wait(&barr); // Activates data-write thread

		if(showFPS) {
			// Show approx. frames-per-second once per second.
			// This is the update speed of fbx2 alone and is
			// disengaged from the eye-rendering application,
			// which operates at its own unrelated refresh rate.
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
