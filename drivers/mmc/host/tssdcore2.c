/*
 * Copyright (c) 2006-2012, Technologic Systems
 * All rights reserved.
 */

/*
 * This code is 100% operating system/CPU independent-- not a single global
 * reference, external symbol, or #include is required.  Centric upon one data 
 * structure "struct sdcore".  OS-specific callbacks for things like DMA 
 * acceleration and sleeping are defined by function pointers to OS-specific
 * code in the struct sdcore.  Minimally requires the os_sleep() callback to be 
 * implemented for proper SD card initialization and a pointer to start
 * of SD card registers.  Auto-determines TS SD core version.  All other
 * callback functions may be left NULL-- they are only to allow speed/CPU
 * utilization improvements.
 * 
 * 3 main public functions - sdreset(), sdread() and sdwrite().  sdreset()
 * returns card size.  
 *
 * Not all SD cards over the years have followed spec perfectly -- many
 * don't even check CRC's on the CMD or DAT busses and some have problems
 * (lock up) when reading/writing the last sectors with SD read/write multiple
 * commands.
 *
 * The TS SD hardware cores are not much more than GPIO bit-bang cores with
 * a few well-placed hardware optimizations to achieve reasonable
 * performance goals.  In the roughly 2500 lines of code that follow, there
 * is support for all distinct TS hardware SD cores on PPC and ARM platforms, 
 * a generic (private) SD command layer, sdcmd(), and SD flash card 
 * (public) routines for initialization + read/write + some SD security 
 * features. 
 *
 */

/* Register offset definitions.  TS-SDCORE is 4 regs total. */
#define SDCMD		0
#define SDGPIO		0	/* version 2 register */
#define SDDAT		1
#define SDSTAT2		1
#define SDSTATE		2
#define SDCTRL		3
#define SDDAT2		4
#define SDCMD2		8
#define SDCTRL2		12
#define SDLUN2		2

struct sdcore {
	/* virtual address of SD block register start, to be filled in
	 * by client code before calling any sdcore functions.
	 */
	size_t sd_regstart; 

	/* public bits for sd_state bitfield, can be read from client code.
	 * Do not write!  Other bits are used internally.
	 */
	#define SDDAT_RX	(1<<0)
	#define SDDAT_TX	(1<<1)
	#define SDCMD_RX	(1<<2)
	#define SDCMD_TX	(1<<3)
	unsigned int sd_state; 

	/* Erase hint for subsequent sdwrite() call, used to optimize 
	 * write throughput on multi-sector writes by pre-erasing this
	 * many sectors. XXX: this doesn't have much benefit on most SDs
	 */
	unsigned int sd_erasehint;

	int refcnt;

	/* Following this comment are 3 function pointer declarations to 
	 * OS helper functions.  The 'os_arg' member is passed as the 
	 * first argument to the helpers and should be set by
	 * client code before issueing sdreset()
	 *
	 * os_dmastream(os_arg, buf, buflen)
	 * This function should look at sd_state and set up and run an
	 * appropriate DMA transfer.  If buf is NULL, callee doesn't care 
	 * about the actual data sent/received and helper function 
	 * can do whatever it wants.  Should return 0 when DMA transfer was 
	 * run and completed successfully.  If this function pointer is
	 * NULL, PIO methods of transfer will be used instead of DMA.
	 *
	 * os_dmaprep(os_arg, buf, buflen)
	 * This function is used to prepare an area of memory for a possible 
	 * DMA transfer.  This function is called once per distinct buffer
	 * passed in.  After this function is called, os_dmastream() may be
	 * called one or more times (for sequential addresses) on subregions
	 * of the address range passed here.  Should write-back or invalidate 
	 * L1 cache lines and possibly look up physical addresses for buf 
	 * passed in if I/O buffers.  If 'os_dmaprep' is set to NULL, function
	 * call will not happen. (though os_dmastream() calls may still)
	 *
	 * os_delay(os_arg, microseconds)
	 * This function is supposed to delay or stall the processor for
	 * the passed in value number of microseconds.
	 */
	void *os_arg;
	int (*os_dmastream)(void *, unsigned char *, unsigned int);
	void (*os_dmaprep)(void *, unsigned char *, unsigned int);
	void (*os_delay)(void *, unsigned int);
	void (*os_irqwait)(void *, unsigned int);
	int (*os_powerok)(void *);
	int (*os_timeout)(void *);
	void (*os_reset_timeout)(void *);

	/* If the SD card last successfully reset is write protected, this
	 * member will be non-zero.
	 */
	unsigned int sd_wprot;

	/* If this card may have been already initialized by TS-SDBOOT, place
	 * the magic token it placed in the EP93xx SYSCON ScratchReg1 here
	 * to avoid re-initialization.
	 */
	unsigned int sdboot_token;

	/* CRC hint for subsequent sdwrite() call, used to optimize
	 * write throughput while using DMA by pre-calculating CRC's for
	 * next write
	 */
	unsigned char *sd_crchint;

	/* The block size of the memory device.  Normally 512, but can be 1024
	 * for larger cards
	 */
	unsigned int sd_blocksize;

	/* Password for auto-unlocking in sdreset()
	 */
	unsigned char *sd_pwd;
	
	/* If the SD card was password locked, this will be non-zero. 
	 */
	unsigned int sd_locked;

	/* Whether or not writes can be parked.
	 */
	unsigned int sd_writeparking;

	/* Logical unit number.  Some SD cores will have multiple card slots.
	 */
	unsigned int sd_lun;

	/* Whether or not we use the multiple block SD write command.
	 */
	unsigned int sd_nomultiwrite;

	/* Debug callback for extra info */
	void (*debug)(void *, unsigned int, const char *, unsigned int, ...);
	void *debug_arg;

	/* The rest of these members are for private internal use and should
	 * not be of interest to client code.
	 */
	unsigned int sd_rcaarg;
	unsigned int sd_csd[17];
	unsigned int sd_crcseq;
	unsigned short sd_crcs[4];
	unsigned int sd_crctmp[4];
	unsigned int sd_timeout;
	unsigned int parked_sector;
	unsigned int hw_version;
	unsigned char sd_scr[8];
	unsigned int sd_sz;
	unsigned char sd_type;
};

enum { WRITE_FAIL, READ_FAIL, SD_RESP_WRONG_REQ, SD_RESP_BAD_CRC, SD_RESP_FAIL,
  SD_HW_TMOUT, SD_SW_TMOUT, SD_DAT_BAD_CRC, SD_STOP_FAIL };

#ifndef SDCORE_NDEBUG
#define DBG(x, ...) if (sd->debug) sd->debug(sd->debug_arg, x, __FUNCTION__, __LINE__, ## __VA_ARGS__)
#else
#define DBG(x, ...)
#endif

/* For sdreadv() / sdwritev() */
struct sdiov {
	unsigned char *sdiov_base;
	unsigned int sdiov_nsect;
};

int sdreset(struct sdcore *);
int sdread(struct sdcore *, unsigned int, unsigned char *, int);
int sdwrite(struct sdcore *, unsigned int, unsigned char *, int);
int sdreadv(struct sdcore *, unsigned int, struct sdiov *, int);
int sdwritev(struct sdcore *, unsigned int, struct sdiov *, int);
int sdsetwprot(struct sdcore *, unsigned int);
#define SDLOCK_UNLOCK	0
#define SDLOCK_SETPWD	1
#define SDLOCK_CLRPWD	2
#define SDLOCK_ERASE	8
#ifndef SD_NOLOCKSUPPORT
int sdlockctl(struct sdcore *, unsigned int, unsigned char *, unsigned char *);
#endif

/*
 * Everything below here is secret!  This code shouldn't have to change
 * even for different OS. 
 */

const static unsigned short crc16tbl[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

const static unsigned char destagger[256] = {
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
	0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
	2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3,
};

#ifndef MAX_SDCORES
#define MAX_SDCORES 64
#endif
static struct sdcore *sdcores[MAX_SDCORES];

#ifndef SD_NOMMC
static int mmcreset2(struct sdcore *);
#endif

static int sdreset2(struct sdcore *);
static int version(struct sdcore *);
static int sdfastinit(struct sdcore *sd);
static int sdcmd2(struct sdcore *, unsigned short, unsigned int, 
  unsigned int *, unsigned char **);
static int sdcmd(struct sdcore *, unsigned short, unsigned int,
  unsigned int *, unsigned char **);
static void mkcommand(unsigned int, unsigned int, unsigned int *);
static int stop(struct sdcore *);
static int stop2(struct sdcore *);
static int sdread2(struct sdcore *, unsigned int, unsigned char *, int)
  __attribute__ ((unused));
static int do_read2(struct sdcore *, unsigned int, struct sdiov *,
  unsigned int);
static int do_read(struct sdcore *, unsigned int, struct sdiov *,
  unsigned int);
static int do_write(struct sdcore *, unsigned int, struct sdiov *, 
  unsigned int);
static int do_write2(struct sdcore *, unsigned int, struct sdiov *, 
  unsigned int);
static int sdsetwprot2(struct sdcore *, unsigned int);
#ifndef SD_NOLOCKSUPPORT
static int sdlockctl2(struct sdcore *, unsigned int, unsigned char *,
  unsigned char *);
#endif

#ifndef SDPOKE8
# define SDPOKE8(sd, x, y)	\
  *(volatile unsigned char *)((sd)->sd_regstart + (x)) = (y)
#endif
#ifndef SDPOKE32
# define SDPOKE32(sd, x, y)	\
  *(volatile unsigned int *)((sd)->sd_regstart + (x)) = (y)
#endif
#ifndef SDPOKE16
# define SDPOKE16(sd, x, y)	\
  *(volatile unsigned short *)((sd)->sd_regstart + (x)) = (y)
#endif
#ifndef SDPEEK8
# define SDPEEK8(sd, x)	*(volatile unsigned char *)((sd)->sd_regstart + (x))
#endif
#ifndef SDPEEK32
# define SDPEEK32(sd, x)	*(volatile unsigned int *)((sd)->sd_regstart + (x))
#endif
#ifndef SDPEEK16
# define SDPEEK16(sd, x)	*(volatile unsigned short *)((sd)->sd_regstart + (x))
#endif

#define S_DUMMY_CLK	0
#define S_SEND_CMD	1
#define S_WAIT_RESP	2
#define S_RX_RESP	3
#define S_WAIT_BUSY	4
#define S_TX_WRITE	5
#define S_CRC_CHECK	6
#define S_OFF		7

#define TYPE_SHORTRESP	2
#define TYPE_LONGRESP	3
#define TYPE_BSYRESP	4
#define TYPE_NORESP	1
#define TYPE_RXDAT	0
#define TYPE_TXDAT	5
#define TYPE_ABORT	6
#define TYPE_RXDAT_IGNRESP	7

#define CMD(idx, type)	(0x40 | (idx) | ((type)<<8))

#define CMD_GO_IDLE_STATE		CMD(0, TYPE_NORESP)
#define CMD_MMC_SEND_OP_COND		CMD(1, TYPE_SHORTRESP)
#define CMD_ALL_SEND_CID		CMD(2, TYPE_LONGRESP)
#define CMD_SEND_RELATIVE_ADDR		CMD(3, TYPE_SHORTRESP)
#define CMD_MMC_SET_RELATIVE_ADDR	CMD(3, TYPE_SHORTRESP)
#define CMD_MMC_SWITCH			CMD(6, TYPE_BSYRESP)
#define CMD_SWITCH_FUNC			CMD(6, TYPE_RXDAT)
#define CMD_SWITCH_FUNC2		CMD(6, TYPE_RXDAT_IGNRESP)
#define CMD_SELECT_CARD			CMD(7, TYPE_BSYRESP)
#define CMD_DESELECT_CARD		CMD(7, TYPE_NORESP)
#define CMD_SEND_IF_COND		CMD(8, TYPE_SHORTRESP)
#define CMD_MMC_SEND_EXT_CSD		CMD(8, TYPE_RXDAT_IGNRESP)
#define CMD_SEND_CSD			CMD(9, TYPE_LONGRESP)
#define CMD_PROGRAM_CSD			CMD(27, TYPE_TXDAT)
#define CMD_SET_BLOCKLEN		CMD(16, TYPE_SHORTRESP)
#define CMD_LOCK_UNLOCK			CMD(42, TYPE_TXDAT)
#define CMD_APP_CMD			CMD(55, TYPE_SHORTRESP)
#define CMD_READ_SINGLE_BLOCK		CMD(17, TYPE_RXDAT)
#define CMD_READ_MULTIPLE_BLOCK		CMD(18, TYPE_RXDAT)
#define CMD_READ_MULTIPLE_BLOCK2	CMD(18, TYPE_RXDAT_IGNRESP)
#define CMD_STOP_TRANSMISSION		CMD(12, TYPE_ABORT)
#define CMD_SEND_STATUS			CMD(13, TYPE_SHORTRESP)
#define CMD_WRITE_BLOCK			CMD(24, TYPE_TXDAT)
#define CMD_WRITE_MULTIPLE_BLOCK	CMD(25, TYPE_TXDAT)

#define ACMD_SD_SEND_OP_COND		CMD(41, TYPE_SHORTRESP)
#define ACMD_SET_CLR_CARD_DETECT	CMD(42, TYPE_SHORTRESP)
#define ACMD_SET_BUS_WIDTH		CMD(6, TYPE_SHORTRESP)
#define ACMD_SET_WR_BLK_ERASE_COUNT	CMD(23, TYPE_SHORTRESP)
#define ACMD_SEND_NUM_WR_BLOCKS		CMD(22, TYPE_RXDAT)
#define ACMD_SEND_SCR			CMD(51, TYPE_RXDAT)
#define ACMD_SEND_SCR2			CMD(51, TYPE_RXDAT_IGNRESP)

/* Private bits for struct sdcore, sd_state member */
#define DATSSP_NOCRC		(1<<4)
#define DATSSP_4BIT		(1<<5)
#define SD_HC			(1<<6)
#define SD_HISPEED		(1<<7)
#define SD_LOSPEED		(1<<8)
#define SD_SELECTED		(1<<9)
#define SD_RESET		(1<<10)

#ifndef NULL
#define	NULL			((void *)0)
#endif

static void remember_sdcore(struct sdcore *sd) {
	int i, newlun = 0;
	
	for (i = 0; i < (sizeof(sdcores)/sizeof(sdcores[0])); i++) {
		if (sdcores[i] == NULL) {
			/* new core, first reset */
			sdcores[i] = sd;
			/* core was almost definitely power-cycled on prev lun
			 * sdreset2(), so we don't need to have the sdreset2()
			 * do it again.
			 */
			if (newlun) sd->sd_state = SD_RESET;
			break;
		} else if (sdcores[i]->sd_regstart == sd->sd_regstart) {
			newlun = 1;
			if (sdcores[i]->sd_lun == sd->sd_lun) {
				sdcores[i] = sd;
				break;
			}
		}
	}
}

#if 0
static void forget_sdcore(struct sdcore *sd) {
	int i, found;

	for (found = i = 0; i < (sizeof(sdcores)/sizeof(sdcores[0])); i++) {
		if (sdcores[i] == sd) found = 1;
		if (found) sdcores[i] = 
		  (i == sizeof(sdcores)/sizeof(sdcores[0]))?NULL:sdcores[i+1];
	}
}
#endif

static int activate(struct sdcore *sd) {
	int i;

	/* Are we already selected? */
	if ((sd->sd_state & (SD_SELECTED|SD_RESET)) == SD_SELECTED)
	  return 0;

	/* Find currently activated SD slot for this HW core */
	for (i = 0; i < (sizeof(sdcores)/sizeof(sdcores[0])); i++) {
		if (sdcores[i] == NULL) break;
		if (sdcores[i]->sd_regstart == sd->sd_regstart &&
		  sdcores[i]->sd_state & SD_SELECTED) break;
	}

	/* Stop whatever parked transfer it has going on. */
	if (sdcores[i]) {
		stop2(sdcores[i]);
		sdcores[i]->sd_state &= ~SD_SELECTED;
	}

	/* Change clock routing, mark us as selected */
#ifdef BIGENDIAN
	SDPOKE16(sd, SDLUN2, sd->sd_lun << 8);
#else
	SDPOKE16(sd, SDLUN2, sd->sd_lun);
#endif

	if (sd->sd_nomultiwrite) i = 0x8; else i = 0x18;

	/* Change clock frequency */
	if (sd->sd_state & SD_HISPEED) SDPOKE8(sd, SDSTAT2, i | 0x20);
	else SDPOKE8(sd, SDSTAT2, i);

	sd->sd_state |= SD_SELECTED;
	if (sd->sd_state & SD_RESET) return 1;
	else return 0;
}


inline static unsigned short
crc16_acc(unsigned short crc, unsigned int b) {
	return (crc << 8) ^ crc16tbl[(crc >> 8) ^ b];
}


static void sd_initcrc(struct sdcore *sd) {
	int i;
	
	for (i = 0; i < 4; i++) {
		sd->sd_crctmp[i] = 0;
		sd->sd_crcs[i] = 0;
	}
	sd->sd_crcseq = 6;
}


static void sd_1bit_feedcrc(struct sdcore *sd, unsigned int dat) {
	sd->sd_crcs[0] = crc16_acc(sd->sd_crcs[0], dat);
}


static void sd_4bit_feedcrc(struct sdcore *sd, unsigned int dat) {
	unsigned int a = 0, b = 0, c = 0, d = 0;
	unsigned int shift = (sd->sd_crcseq & 0x7);

	a = sd->sd_crctmp[0];
	b = sd->sd_crctmp[1];
	c = sd->sd_crctmp[2];
	d = sd->sd_crctmp[3];

	a |= destagger[dat] << shift;
	dat >>= 1;
	b |= destagger[dat] << shift;
	dat >>= 1;
	c |= destagger[dat] << shift;
	dat >>= 1;
	d |= destagger[dat] << shift;

	if (shift == 0) {
		sd->sd_crcs[0] = crc16_acc(sd->sd_crcs[0], a);
		sd->sd_crcs[1] = crc16_acc(sd->sd_crcs[1], b);
		sd->sd_crcs[2] = crc16_acc(sd->sd_crcs[2], c);
		sd->sd_crcs[3] = crc16_acc(sd->sd_crcs[3], d);
		a = b = c = d = 0;
	}

	sd->sd_crcseq -= 2;
	sd->sd_crctmp[0] = a;
	sd->sd_crctmp[1] = b;
	sd->sd_crctmp[2] = c;
	sd->sd_crctmp[3] = d;
}


/* This should be called 8 times to get the full 8 bytes of CRC generated */
static unsigned int sd_4bit_getcrc(struct sdcore *sd)
{
	static const unsigned char restaggertbl[4] = { 0x0, 0x1, 0x10, 0x11 };
	static const unsigned char restaggertbl_lsl1[4] =
		{ 0x0, 0x2, 0x20, 0x22 };
	static const unsigned char restaggertbl_lsl2[4] =
		{ 0x0, 0x4, 0x40, 0x44 };
	static const unsigned char restaggertbl_lsl3[4] =
		{ 0x0, 0x8, 0x80, 0x88 };
	unsigned int ret;

	ret = restaggertbl[sd->sd_crcs[0] >> 14];
	sd->sd_crcs[0] <<= 2;
	ret |= restaggertbl_lsl1[sd->sd_crcs[1] >> 14];
	sd->sd_crcs[1] <<= 2;
	ret |= restaggertbl_lsl2[sd->sd_crcs[2] >> 14];
	sd->sd_crcs[2] <<= 2;
	ret |= restaggertbl_lsl3[sd->sd_crcs[3] >> 14];
	sd->sd_crcs[3] <<= 2;

	return ret;
}


/* This should be called 2 times to get the full 2 bytes of CRC generated */
static unsigned int sd_1bit_getcrc(struct sdcore *sd)
{
	unsigned int ret;

	ret = sd->sd_crcs[0] >> 8;
	sd->sd_crcs[0] = (sd->sd_crcs[0] & 0xff) << 8;
	return ret;
}


static inline void datssp_feedcrc(struct sdcore *sd, unsigned int dat)
{
	if (!(sd->sd_state & DATSSP_NOCRC)) {
		if (sd->sd_state & DATSSP_4BIT) sd_4bit_feedcrc(sd, dat);
		else sd_1bit_feedcrc(sd, dat);
	}
}


static inline unsigned int datssp_getcrc(struct sdcore *sd)
{
	unsigned int ret = 0;

	if (!(sd->sd_state & DATSSP_NOCRC)) {
		if (sd->sd_state & DATSSP_4BIT) ret = sd_4bit_getcrc(sd);
		else ret = sd_1bit_getcrc(sd);
	}
	return ret;
}


static inline unsigned int
crc7(unsigned int crc, const unsigned int *pc, unsigned int len) {
	unsigned int i;
	unsigned char ibit;
	unsigned char c;

	for (i = 0; i < len; i++, pc++) {
		c = *pc;
		for (ibit = 0; ibit < 8; ibit++) {
			crc <<= 1;
			if ((c ^ crc) & 0x80) crc ^= 0x09;

			c <<= 1;
		}

		crc &= 0x7F;
	}

	return crc;
}


static inline void
mkcommand(unsigned int cmdidx, unsigned int arg, unsigned int *retcmd) {
	retcmd[0] = cmdidx;
	retcmd[1] = arg >> 24;
	retcmd[2] = arg >> 16;
	retcmd[3] = arg >> 8;
	retcmd[4] = arg;
	retcmd[5] = (0x1 | (crc7(0, retcmd, 5) << 1));
}


static inline void reset_timeout(struct sdcore *sd) {
	sd->sd_timeout = 0;
	if (sd->os_reset_timeout) sd->os_reset_timeout(sd); 
}


static inline int timeout(struct sdcore *sd) {
	int ret = 0;
	if (sd->sd_timeout > 1000000) ret = 1;
	else if (sd->os_timeout) ret = sd->os_timeout(sd);
	else sd->sd_timeout++;
	if (ret) DBG(SD_SW_TMOUT);
	return ret;
}


static unsigned int sdsize(struct sdcore *sd) {
	unsigned int csize, csize_mult, rd_bl_len;

	if (sd->sd_sz != 0) return sd->sd_sz;

	if (sd->sd_csd[1] & 0xc0) {
		csize = (sd->sd_csd[10] | (sd->sd_csd[9] << 8) | 
		  ((sd->sd_csd[8] & 0x3f) << 16));
		sd->sd_sz = (csize + 1) * 1024;
	} else {
	        rd_bl_len = 1 << ((sd->sd_csd[6] & 0xf) - 9);
		csize = ((sd->sd_csd[7] & 0x03) << 10) | 
		  ((sd->sd_csd[8] << 2) | ((sd->sd_csd[9] & 0xc0) >> 6));
		csize_mult = ((sd->sd_csd[10] & 0x03) << 1) | 
		  ((sd->sd_csd[11] & 0x80) >> 7);
		sd->sd_sz = (csize + 1) * (1 << (csize_mult + 2)) * rd_bl_len;
	}
	return sd->sd_sz;
}


static unsigned int tend_ssp(struct sdcore *sd, unsigned int **cmdresp, 
  unsigned char **dat) {
	unsigned int d;
	unsigned int s = SDPEEK8(sd, SDSTATE);

	if (s & 0x8) {
		if (sd->sd_state & SDCMD_RX) {
			d = SDPEEK8(sd, SDCMD);
			if (cmdresp) {
				**cmdresp = d;
				*cmdresp = *cmdresp + 1;
				reset_timeout(sd);
			}
		} else if (sd->sd_state & SDCMD_TX) {
			SDPOKE8(sd, SDCMD, **cmdresp);
			*cmdresp = *cmdresp + 1;
			reset_timeout(sd);
		}
	}

	if (s & 0x10) {
		if (sd->sd_state & SDDAT_RX) {
			d = SDPEEK8(sd, SDDAT);
			if (dat) {
				**dat = d;
				*dat = *dat + 1;
				reset_timeout(sd);
			}
		} else if (sd->sd_state & SDDAT_TX) {
			reset_timeout(sd);
			if (dat) {
				d = **dat;
				*dat = *dat + 1;
				SDPOKE8(sd, SDDAT, d);
				datssp_feedcrc(sd, d);
			} else {
				d = datssp_getcrc(sd);
				SDPOKE8(sd, SDDAT, d);
			}
		}
	}

	return s;
}


static int error(struct sdcore *sd, unsigned int *resp, unsigned short req) {
	unsigned int crc, status, ret;

	if ((req & 0x3f) != resp[0]) {
		DBG(SD_RESP_WRONG_REQ, req & 0x3f, resp[0]);
		return 1;
	}

	crc = (0x1 | (crc7(0, resp, 5) << 1));
	if (crc != resp[5]) {
		DBG(SD_RESP_BAD_CRC, req & 0x3f, crc, resp[5]);
		return 1;
	}

	status = resp[1] << 24;
	status |= resp[2] << 16;
	status |= resp[3] << 8;
	status |= resp[4];
	ret = status & 0xfdf90008;
	if (ret) DBG(SD_RESP_FAIL, req & 0x3f, ret);
	return ret;
}


static int
sdcmd2(struct sdcore *sd, unsigned short req, unsigned int arg, 
  unsigned int *resp, unsigned char **dat) {
	unsigned int i, j, s, cmdresp[17];
	unsigned int resplen;
	unsigned int type = (req >> 8);
	unsigned int cmdidx = req; 
	unsigned int *cmdptr = cmdresp;
	unsigned int *respptr;
	unsigned int dly;
	int ok32 = (sd->hw_version == 2);
	int ok16 = (ok32 || (sd->hw_version == 3));
	int sddat2_8;

	// If no space for response provided by caller, use local buffer
	if (resp == NULL) resp = cmdresp;
	respptr = resp;

	if (activate(sd)) return 1;

	dly = sd->sd_state & SD_LOSPEED;

	if (!dly) {
		unsigned int x;
		SDPOKE8(sd, SDGPIO, 0xbf);
#ifdef BIGENDIAN
		x = (cmdidx & 0xff);
		x |= ((arg >> 24) & 0xff) << 8;
		x |= ((arg >> 16) & 0xff) << 16;
		x |= ((arg >> 8) & 0xff) << 24;
		if (ok32) SDPOKE32(sd, SDCMD2, x);
		else if (ok16) {
			SDPOKE16(sd, SDCMD2, x);
			SDPOKE16(sd, SDCMD2, x >> 16);
		} else {
			SDPOKE8(sd, SDCMD2, x);
			SDPOKE8(sd, SDCMD2, x >> 8);
			SDPOKE8(sd, SDCMD2, x >> 16);
			SDPOKE8(sd, SDCMD2, x >> 24);
		}
#else
		x = (cmdidx & 0xff) << 24;
		x |= ((arg >> 24) & 0xff) << 16;
		x |= ((arg >> 16) & 0xff) << 8;
		x |= ((arg >> 8) & 0xff);
		if (ok32) SDPOKE32(sd, SDCMD2, x);
		else if (ok16) {
			SDPOKE16(sd, SDCMD2, x >> 16);
			SDPOKE16(sd, SDCMD2, x);
		} else {
			SDPOKE8(sd, SDCMD2, x >> 24);
			SDPOKE8(sd, SDCMD2, x >> 16);
			SDPOKE8(sd, SDCMD2, x >> 8);
			SDPOKE8(sd, SDCMD2, x);
		}
#endif
		SDPOKE8(sd, SDCMD2, arg);
	} else {
		// Build command packet
		mkcommand(cmdidx, arg, cmdptr);

		// Send command
		for (i = 0; i < 6; i++) {
			unsigned int b = *cmdptr++;
			unsigned int x;
	
			if (timeout(sd)){
				printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
				break;
			}
			for (j = 0; j < 8; j++) {
				x = 0x8f | ((b & 0x80) >> 3);
				b = b << 1;
				SDPOKE8(sd, SDGPIO, x); // clk negedge
				SDPEEK8(sd, SDGPIO);    // delay
				SDPEEK8(sd, SDGPIO);    // delay 
				x |= 0x20; 
				SDPOKE8(sd, SDGPIO, x); // clk posedge
				SDPEEK8(sd, SDGPIO);    // delay
				SDPEEK8(sd, SDGPIO);    // delay
			}
		}
	}

	if (type == TYPE_NORESP) {
		printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	 	goto done;
	} else if (type == TYPE_RXDAT_IGNRESP) {
		printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	 	goto ignresp;
	} else if (type == TYPE_LONGRESP) {
		printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		resplen = 17;
	} else {
		//printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		resplen = 6; // shortresp
	}

	// clock until start bit on CMD pin
	reset_timeout(sd);
	while(1) {
		if (timeout(sd)) {
			printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
			goto done;
		}
		if (req == CMD_SEND_IF_COND) sd->sd_timeout += 100000;
		SDPOKE8(sd, SDGPIO, 0xdf); // clk negedge
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		s = SDPEEK8(sd, SDGPIO);   // sample
		if ((s & 0x10) == 0x0) break;
		SDPOKE8(sd, SDGPIO, 0xff); // clk posedge
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
	} 
	reset_timeout(sd);

	// Next we receive the response. 
	if (ok16 && !ok32) sddat2_8 = SDDAT2 + 1;
	else sddat2_8 = SDDAT2;
	if (dly) for (i = 0; i < resplen; i++) {
		unsigned int r = 0;

		for (j = 0; j < 8; j++) {
			SDPOKE8(sd, SDGPIO, 0xdf); // clk negedge
			SDPEEK8(sd, SDGPIO);       // delay
			s = SDPEEK8(sd, SDGPIO);   // sample
			SDPOKE8(sd, SDGPIO, 0xff); // clk posedge
			SDPEEK8(sd, SDGPIO);       // delay
			SDPEEK8(sd, SDGPIO);       // delay
			r = r << 1;
			r |= ((s & 0x10) >> 4);
		}
	
		*respptr++ = r;
	} else while (resplen > 0) {
		unsigned int r;

#ifdef BIGENDIAN
		if (ok32 && resplen >= 4) {
			r = SDPEEK32(sd, SDCMD2);
			*respptr++ = r & 0xff;
			*respptr++ = (r >> 8) & 0xff;
			*respptr++ = (r >> 16) & 0xff;
			*respptr++ = (r >> 24);
			resplen -= 4;
		} else if (ok16 && resplen >= 2) {
			r = SDPEEK16(sd, SDCMD2);	
			*respptr++ = r & 0xff;
			*respptr++ = (r >> 8) & 0xff;
			
			resplen -= 2;
		} else {
			*respptr++ = SDPEEK8(sd, sddat2_8);
			resplen--;
		}
#else
		if (ok32 && resplen >= 4) {
			r = SDPEEK32(sd, SDCMD2);
			*respptr++ = (r >> 24);
			*respptr++ = (r >> 16) & 0xff;
			*respptr++ = (r >> 8) & 0xff;
			*respptr++ = r & 0xff;
			resplen -= 4;
		} else if (ok16 && resplen >= 2) {
			r = SDPEEK16(sd, SDCMD2);	
			*respptr++ = (r >> 8) & 0xff;
			*respptr++ = r & 0xff;
			resplen -= 2;
		} else {
			*respptr++ = SDPEEK8(sd, sddat2_8);
			resplen--;
		}
#endif
	}
	if (type == TYPE_BSYRESP) {
		s = 0;
		while ((s & 0x7) != 0x7) {
			if (timeout(sd)){ 
				printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
				break;
			}
			SDPOKE8(sd, SDGPIO, 0x9f);  // clk negedge
			if (dly) SDPEEK8(sd, SDGPIO);        // delay
			s = s << 1;
			s |= SDPEEK8(sd, SDGPIO) & 0x1;
			SDPOKE8(sd, SDGPIO, 0xbf);
			if (dly) SDPEEK8(sd, SDGPIO);
		}
	}

ignresp:

	if (type == TYPE_ABORT) 
		sd->sd_state &= ~(SDDAT_RX|SDDAT_TX);

#ifndef SD_READONLYDMA
	if (type == TYPE_TXDAT) {
		sd->sd_state |= SDDAT_TX;
		/* 2 clocks for nWR */
		SDPOKE8(sd, SDGPIO, 0xdf); // clk negedge
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		if (dly) SDPEEK8(sd, SDGPIO);       // delay 
		SDPOKE8(sd, SDGPIO, 0xff); // clk posedge
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		SDPOKE8(sd, SDGPIO, 0xdf); // clk negedge
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		if (dly) SDPEEK8(sd, SDGPIO);       // delay 
		SDPOKE8(sd, SDGPIO, 0xff); // clk posedge
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		if (sd->sd_state & DATSSP_4BIT)
			SDPOKE8(sd, SDGPIO, 0x10); // assert start, clk negedge
		else
			SDPOKE8(sd, SDGPIO, 0x1e);
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		if (dly) SDPEEK8(sd, SDGPIO);       // delay 
		if (sd->sd_state & DATSSP_4BIT)
			SDPOKE8(sd, SDGPIO, 0x30); // clk posedge
		else
			SDPOKE8(sd, SDGPIO, 0x3e);
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
		if (dly) SDPEEK8(sd, SDGPIO);       // delay
	}
#endif

	if (type == TYPE_RXDAT || type == TYPE_RXDAT_IGNRESP) 
	  sd->sd_state |= SDDAT_RX;

done:
	// 8 clocks before stopping
	if (!(sd->sd_state & (SDDAT_TX|SDDAT_RX))) {
		if (dly) for (i = 0; i < 8; i++) {
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);       // delay
			SDPEEK8(sd, SDGPIO);       // delay 
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);       // delay
			SDPEEK8(sd, SDGPIO);       // delay
		} else {
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPOKE8(sd, SDCMD2, 0xff);
		}
	}
	if (timeout(sd)){
		printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	 	return 1;
	}
	else return 0;

}

static int
sdcmd(struct sdcore *sd, unsigned short req, unsigned int arg, 
  unsigned int *resp, unsigned char **dat) {
	unsigned int s, cmdresp[17];
	unsigned int resplen;
	unsigned int type = (req >> 8);
	unsigned int cmdidx = req; 
	unsigned int *cmdptr = cmdresp;
	unsigned int *cmd = cmdresp;
	unsigned int *respptr;
	unsigned int ndat;

	if (sd->hw_version != 0) return sdcmd2(sd, req, arg, resp, dat);

	// If no space for response provided by caller, use local buffer
	if (resp == NULL) resp = cmdresp;
	respptr = resp;

	// Before continuing, we must wait for the FSM to get to the
	// S_SEND_CMD state.  After a previous command, we may still be
	// in S_DUMMY_CLK or in case of an ABORT, we may be in the middle of
	// clocking a byte for TX or RX.
	s = SDPEEK8(sd, SDSTATE);
	while ((s & 0x7) != S_SEND_CMD) {
		if (timeout(sd)) break;
		s = SDPEEK8(sd, SDSTATE);
	}

	// We know we're in S_SEND_CMD, but we may need to change the
	// command type.  This won't cause a state change.  
	if ((s & 0xe7) != (S_SEND_CMD | (type << 5)))
		SDPOKE8(sd, SDSTATE, S_SEND_CMD | (type << 5));

	// Build command packet
	mkcommand(cmdidx, arg, cmdptr);

	// Next, we loop while tending the SSPs until we get our last
	// byte of command data out.  We may get a few bytes from the DAT
	// SSP if we are aborting a previous data transfer command.  If we do
	// those get placed in a buffer or thrown away based on the callers
	// "dat" parameter.
	sd->sd_state |= SDCMD_TX;
	while ((cmdptr - cmd) != 6) {
		if (timeout(sd)) break;
		s = tend_ssp(sd, &cmdptr, dat);
	}
	sd->sd_state &= ~SDCMD_TX;

	// If we got out of sync with the hardware, that would be bad.
	// The hardware should still be in S_SEND_CMD for the last CMDSSP
	// byte.
	if ((s & 0x7) != S_SEND_CMD) {
		SDPOKE8(sd, SDSTATE, S_OFF);
		return 1;
	}

	if (type == TYPE_NORESP) goto done;
	else if (type == TYPE_LONGRESP) resplen = 17;
	else resplen = 6;

	// Next state should be S_WAIT_RESP or S_RX_RESP.  We may get 
	// more bytes from the DATSSP while shifting out our last bits of cmd
	while (((s & 0x7) != S_WAIT_RESP) && ((s & 0x7) != S_RX_RESP)) {
		if (timeout(sd)) break;
		if (req == CMD_SEND_IF_COND) sd->sd_timeout += 1000;
		s = tend_ssp(sd, NULL, dat);
	}
	
	// Once we're in S_WAIT_RESP or S_RX_RESP though, the DATSSP is only
	// active for 2 more clocks at the beginning of the S_WAIT_RESP state.
	// This is enough for one more byte in 4-bit mode, though we may have
	// 2 bytes already in our DATSSP.
	if (sd->sd_state & (SDDAT_RX|SDDAT_TX)) {
		do {
			if (timeout(sd)) break;
			s = tend_ssp(sd, NULL, dat);
		} while (!(s & 0x18));

		// We've now read/wrote one more byte to the DATSSP
		// which should allow our FSM to advance to the RX_RESP state.
		// If we pick up more than 2 more DATSSP bytes, something is
		// wrong.
		ndat = 0;
		while ((s & 0x7) != S_RX_RESP) {
			if (timeout(sd) || ndat > 2) break;
			s = tend_ssp(sd, NULL, dat);
			if (s & 0x10) ndat++;
		}

		if (ndat > 2) {
			SDPOKE8(sd, SDSTATE, S_OFF);
			return 1;
		}
	}

	// We're now done with whatever business we had remaining with the
	// previous command's DATSSP transfer since we've either just got our 
	// first byte of response or our last byte of data
	sd->sd_state &= ~(SDDAT_RX|SDDAT_TX);
	if (type == TYPE_RXDAT) sd->sd_state |= SDDAT_RX;

	// Next we receive the response.  If this is TYPE_RXDAT command,
	// or an abortion of a previous TYPE_RXDAT command, we may get a 
	// few bytes from the DAT SSP also.
	sd->sd_state |= SDCMD_RX;
	while ((respptr - resp) != resplen) {
		if (timeout(sd)) break;
		s = tend_ssp(sd, &respptr, dat);
		if ((s & 0x10) && (resp == respptr)) {
			SDPOKE8(sd, SDSTATE, S_OFF);
			sd->sd_state &= ~(SDCMD_RX|SDDAT_RX);
			return 1;
		}
	}
	sd->sd_state &= ~SDCMD_RX;

	if (type == TYPE_ABORT) 
		sd->sd_state &= ~(SDDAT_RX|SDDAT_TX);

	if (type == TYPE_TXDAT) sd->sd_state |= SDDAT_TX;

done:
	if (timeout(sd)) return 1;
	else return 0;
}

int datssp_stream2_1bit(struct sdcore *sd, unsigned char **dat,
  unsigned int buflen) {
  	printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	if(buflen > 512) return -1;
	unsigned int x, s, bits;
	int i, j;
	unsigned char *d = *dat;
	unsigned int crc;
	int ret = 0;

	if (sd->os_irqwait) sd->os_irqwait(sd->os_arg, 1);

	bits = buflen * 8;

	printk(KERN_INFO "BYTES: %d, BITS %d\n", buflen, bits);

	if (sd->sd_state & SDDAT_RX) {
		printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		/* Read Data */
		for (x = 0; x < buflen; x++) {
			unsigned char byte = 0;
			for (i = 7; i >= 0; i--) {
				SDPOKE8(sd, SDGPIO, 0xdf);
				SDPEEK8(sd, SDGPIO); // delay
				s = SDPEEK8(sd, SDGPIO);
				SDPOKE8(sd, SDGPIO, 0xff);
				SDPEEK8(sd, SDGPIO); // delay
				byte |= (s & 0x1) << i;
			}
			d[x] = byte;
			sd_1bit_feedcrc(sd, byte);
		}

		for (x = 0; x < 2; x++) {
			unsigned char byte = 0;
			for (i = 7; i >= 0; i--) {
				SDPOKE8(sd, SDGPIO, 0xdf);
				SDPEEK8(sd, SDGPIO); // delay
				s = SDPEEK8(sd, SDGPIO);
				SDPOKE8(sd, SDGPIO, 0xff);
				SDPEEK8(sd, SDGPIO); // delay
				byte |= (s & 0x1) << i;
			}
			unsigned char crc = sd_1bit_getcrc(sd);
			if(crc != byte) {
				printk(KERN_INFO "CRC Expected 0x%X, got 0x%X\n", crc, byte);
			}
		}

		for (i = 0; i < 8; i++) {
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);
			SDPEEK8(sd, SDGPIO);
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);
		}
	} else {
		printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		/* Write data */
		for (j = buflen; j <= 0; j--) {
			unsigned char byte = d[j];
			sd_1bit_feedcrc(sd, byte);
			for (i = 0; i < 8; i++) {
				unsigned int val = 0x1e | ((byte >> 7) & 0x1);
				byte = byte << 1;
				SDPOKE8(sd, SDGPIO, val);
				SDPEEK8(sd, SDGPIO); // delay
				SDPEEK8(sd, SDGPIO);
				x |= 0x20;
				SDPOKE8(sd, SDGPIO, val);
				SDPEEK8(sd, SDGPIO); // delay
			}
		}
		/* Write CRC */
		for (x = 0; x < 2; x++) {
			unsigned int b = sd_1bit_getcrc(sd); 
			unsigned int val;

			for (i = 0; i < 8; i++) {
				val = 0x1e | ((b >> 7) & 0x1);
				b = b << 1;
				SDPOKE8(sd, SDGPIO, val);  // clk negedge
				SDPEEK8(sd, SDGPIO);
				SDPEEK8(sd, SDGPIO);
				val |= 0x20;
				SDPOKE8(sd, SDGPIO, val);  // clk posedge
				SDPEEK8(sd, SDGPIO);
			}
		}
	}
printk(KERN_INFO "HERE: %s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	return ret;
}


static int datssp_stream2(struct sdcore *sd, unsigned char **dat,
  unsigned int buflen) {
	unsigned char *d;
	int ret;
#ifndef SD_READONLYDMA
	int ok32;
	int ok16;
	int sddat2_8;
	unsigned int x;
#endif

	if (sd->os_dmastream /* && (sd->sd_state & SDDAT_RX) */) {
		d = dat ? *dat : NULL;
		ret = sd->os_dmastream(sd->os_arg, d, buflen);
		if (!ret && d) *dat += buflen;
		return ret;
	}

#ifndef SD_READONLYDMA
	d = *dat;

	while (buflen > 512) {
		datssp_stream2(sd, dat, 512);
		if (sd->os_irqwait) sd->os_irqwait(sd->os_arg, 1);
		buflen -= 512;
		d = *dat;
	}

	ok32 = (sd->hw_version == 2);
	ok16 = (ok32 || (sd->hw_version == 3));
	if (ok16 && !ok32) sddat2_8 = SDDAT2 + 1;
	else sddat2_8 = SDDAT2;

	if (sd->sd_state & SDDAT_RX) {

		while (((size_t)d & 0x1 && buflen > 0) || buflen == 1) {
			*d++ = SDPEEK8(sd, sddat2_8);
			buflen--;
		}
	
		if (((size_t)d & 0x2) && buflen >= 2) {
			if (ok16) *(unsigned short *)(d) = SDPEEK16(sd, SDDAT2);
			else {
#ifdef BIGENDIAN
				x = SDPEEK8(sd, sddat2_8) << 8;
				x |= SDPEEK8(sd, sddat2_8);
#else
				x = SDPEEK8(sd, sddat2_8);
				x |= SDPEEK8(sd, sddat2_8) << 8;
#endif
				*(unsigned short *)(d) = x;
			}
			buflen -= 2;
			d += 2;
		}

		if (ok32) while (buflen >= 4) {
			*(unsigned int *)(d) = SDPEEK32(sd, SDDAT2);
			buflen -= 4;
			d += 4;
		} else if (ok16) while (buflen >= 4) {
#ifdef BIGENDIAN
			x = SDPEEK16(sd, SDDAT2) << 16;
			x |= SDPEEK16(sd, SDDAT2);
#else
			x = SDPEEK16(sd, SDDAT2);
			x |= SDPEEK16(sd, SDDAT2) << 16;
#endif
			buflen -= 4;
			*(unsigned int *)(d) = x;
			d += 4;
		} else while (buflen >= 4) {
#ifdef BIGENDIAN
			x = SDPEEK8(sd, sddat2_8) << 24;
			x |= SDPEEK8(sd, sddat2_8) << 16;
			x |= SDPEEK8(sd, sddat2_8) << 8;
			x |= SDPEEK8(sd, sddat2_8);
#else
			x = SDPEEK8(sd, sddat2_8);
			x |= SDPEEK8(sd, sddat2_8) << 8;
			x |= SDPEEK8(sd, sddat2_8) << 16;
			x |= SDPEEK8(sd, sddat2_8) << 24;
#endif
			buflen -= 4;
			*(unsigned int *)(d) = x;
			d += 4;
		}
	} else {
		while (((size_t)d & 0x1) || buflen == 1) {
			SDPOKE8(sd, SDDAT2, *d++);
			buflen--;
		}
	
		if (((size_t)d & 0x2) && buflen >= 2) {
			if (ok16) SDPOKE16(sd, SDDAT2, *(unsigned short *)(d));
			else {
				x = *(unsigned short *)(d);
#ifdef BIGENDIAN
				SDPOKE8(sd, SDDAT2, x >> 8); 
				SDPOKE8(sd, SDDAT2, x); 
#else
				SDPOKE8(sd, SDDAT2, x); 
				SDPOKE8(sd, SDDAT2, x >> 8); 
#endif
			}
			buflen -= 2;
			d += 2;
		}

		if (ok32) while (buflen >= 4) {
			SDPOKE32(sd, SDDAT2, *(unsigned int *)(d));
			buflen -= 4;
			d += 4;
		} else if (ok16) while (buflen >= 4) {
			x = *(unsigned int *)(d);
			buflen -= 4;
			d += 4;
#ifdef BIGENDIAN
			SDPOKE16(sd, SDDAT2, x >> 16);
			SDPOKE16(sd, SDDAT2, x);
#else
			SDPOKE16(sd, SDDAT2, x);
			SDPOKE16(sd, SDDAT2, x >> 16);
#endif
		} else while (buflen >= 4) {
			x = *(unsigned int *)(d);
			buflen -= 4;
			d += 4;
#ifdef BIGENDIAN
			SDPOKE8(sd, SDDAT2, x >> 24);
			SDPOKE8(sd, SDDAT2, x >> 16);
			SDPOKE8(sd, SDDAT2, x >> 8);
			SDPOKE8(sd, SDDAT2, x);
#else
			SDPOKE8(sd, SDDAT2, x);
			SDPOKE8(sd, SDDAT2, x >> 8);
			SDPOKE8(sd, SDDAT2, x >> 16);
			SDPOKE8(sd, SDDAT2, x >> 24);
#endif
		}
	}

	*dat = d;

	if (buflen > 0) return datssp_stream2(sd, dat, buflen);
	else return 0;
#else
	return 0;
#endif
}


static int datssp_stream(struct sdcore *sd, unsigned char **dat, 
  unsigned int buflen) {
	unsigned int s, t, byte = 0;
	unsigned char *d;

	if (((sd->sd_state & SDDAT_RX) && sd->os_dmastream) /* ||
	  ((sd->sd_state & SDDAT_TX) && sd->os_dmastream && dat) */ ) {
		unsigned char *d = dat ? *dat : NULL;
		int ret = sd->os_dmastream(sd->os_arg, d, buflen);
		if (!ret && d) *dat += buflen;
		return ret;
	}

	if (sd->hw_version > 0) return datssp_stream2(sd, dat, buflen);

	while (buflen) {
		if (timeout(sd)) return 1;
		s = tend_ssp(sd, NULL, dat);
		if (s & 0x10) {
			buflen--;
			if (byte++ > 7) {
				if (sd->sd_state & SDDAT_RX)
				  goto fastrx;
				else goto fasttx;
			}
		} 
	}

	// Now we can go faster (PIO)
fastrx:
	if (dat) {
		d = *dat;
		while (buflen) {
			s = SDPEEK8(sd, SDDAT);
			*d = s;
			buflen--;
			d++;
		}
		*dat = d;
	} else {
		while (buflen--) SDPEEK8(sd, SDDAT);
	}
	return 0;

fasttx:
	if (dat) {
		d = *dat;
		while (buflen) {
			t = *d;
			SDPOKE8(sd, SDDAT, t);
			buflen--;
			d++;
			datssp_feedcrc(sd, t);
		}
		*dat = d;
	} else {
		while (buflen--) SDPOKE8(sd, SDDAT, datssp_getcrc(sd));
	}
	return 0;
}


static int stop(struct sdcore *sd) {
	int ret;
	unsigned int resp[6];

	if (sd->hw_version) return stop2(sd);

	if (sd->parked_sector) {
		if (sd->sd_state & SDDAT_TX) {
			/* wait to get out of S_WAIT_BUSY */
			while ((SDPEEK8(sd, SDSTATE) & 0x7) != S_TX_WRITE) 
			  if (timeout(sd)) break;

			/* abort parked write */
			SDPOKE8(sd, SDSTATE, S_SEND_CMD | (TYPE_ABORT << 5));
			sd->sd_state &= ~SDDAT_TX;
			sd->sd_state |= SDDAT_RX;
			ret = sdcmd(sd, CMD_STOP_TRANSMISSION, 0, resp, NULL);
			sd->sd_state &= ~SDDAT_RX;
			SDPOKE8(sd, SDSTATE, S_WAIT_BUSY | (TYPE_BSYRESP << 5));
		} else {
			/* abort parked read */
			SDPOKE8(sd, SDSTATE, S_SEND_CMD | (TYPE_ABORT << 5));
			ret = sdcmd(sd, CMD_STOP_TRANSMISSION, 0, resp, NULL);
		}
		sd->parked_sector = 0;
		if (ret||error(sd, resp, CMD_STOP_TRANSMISSION)||timeout(sd)) 
		  return 1;
	}
	return 0;
}


static int stop2(struct sdcore *sd) {
	int ret;
	unsigned int resp[6];

	if (sd->parked_sector) {
		if (sd->os_irqwait) sd->os_irqwait(sd->os_arg, 0);
		if (sd->sd_state & SDDAT_TX) {
			/* abort parked write */
		 	ret = sdcmd2(sd, CMD_STOP_TRANSMISSION, 0, resp, NULL);
			SDPOKE8(sd, SDCTRL2, 0x0);
			if (sd->os_irqwait) sd->os_irqwait(sd->os_arg, 5);
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPOKE8(sd, SDCMD2, 0xff);

			/*
			while ((SDPEEK8(sd, SDGPIO) & 0xf) != 0xf) {
				sd->os_delay(sd->os_arg, 1);
				SDPOKE8(sd, SDGPIO, 0xdf);
				SDPOKE8(sd, SDGPIO, 0xff);
				if (timeout(sd)) return 1;
			}
			*/
			reset_timeout(sd);
		} else {
			/* abort parked read */
			ret = sdcmd2(sd, CMD_STOP_TRANSMISSION, 0, resp, NULL);
		}
		sd->parked_sector = 0;
		if (ret||error(sd, resp, CMD_STOP_TRANSMISSION)||timeout(sd)) {
			DBG(SD_STOP_FAIL, ret);
			return 1;
		}
	}
	return 0;
}


static int do_read2(struct sdcore *sd, unsigned int sector, struct sdiov *iov, 
  unsigned int iovcnt) {
	unsigned int n, s, sz;
	unsigned char *datptr, *dat;

	if (iovcnt == 0) return 0;

	if (activate(sd)) return 1;

	n = iov->sdiov_nsect;
	datptr = dat = iov->sdiov_base;
	sz = sdsize(sd);
	if (sector >= sz) return 0;

	if (sd->parked_sector) {
		if (!(sd->sd_state & SDDAT_TX) && sd->parked_sector == sector) {
			if (sd->os_irqwait && !sd->os_dmastream)
			  sd->os_irqwait(sd->os_arg, 3);
			goto receive;
		}
		
		stop2(sd);
	}

	if (sd->sd_state & SD_HC) 
	  sdcmd2(sd, CMD_READ_MULTIPLE_BLOCK2, sector, NULL, NULL);
	else
	  sdcmd2(sd, CMD_READ_MULTIPLE_BLOCK2, sector * 512, NULL, NULL);

	do {
		if (timeout(sd)) {
			DBG(READ_FAIL, sector);
			return 1;
		}
		SDPOKE8(sd, SDGPIO, 0xdf);
		s = SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xff);
	} while ((s & 0xf) != 0x0);
	reset_timeout(sd);

receive:
	if (sd->os_dmaprep && sd->os_dmastream)
	  sd->os_dmaprep(sd->os_arg, datptr, n * 512);

	SDPOKE8(sd, SDGPIO, 0xdf);
	sd->parked_sector = sector + n;

nextiov:
	if (sd->parked_sector >= sz) {
		n -= sd->parked_sector - sz;
		if (n > 1) datssp_stream2(sd, &datptr, (n - 1) * 512);
		/* temp disable rdmult_en bit */
		SDPOKE8(sd, SDSTAT2, SDPEEK8(sd, SDSTAT2) & ~0x8);
		datssp_stream2(sd, &datptr, 512);
		SDPOKE8(sd, SDSTAT2, SDPEEK8(sd, SDSTAT2) | 0x8);
		stop2(sd);
		iovcnt = 1; /* Force this iov to be the last */
	} else datssp_stream2(sd, &datptr, n * 512);

	if (--iovcnt) {
		++iov;
		n = iov->sdiov_nsect;
		datptr = iov->sdiov_base;
		sd->parked_sector += n;
		if (sd->os_dmaprep && sd->os_dmastream)
		  sd->os_dmaprep(sd->os_arg, datptr, n * 512);
		goto nextiov;
	} 

	/* s = SDPEEK8(sd, SDSTAT2);
	if (s & 0x44) {
		sd->sd_timeout = 1000001;
		return 1;
	}
	else */ return 0;
}


static int do_read(struct sdcore *sd, unsigned int sector, struct sdiov *iov, 
  unsigned int iovcnt) {
	unsigned int resp[6], ret, n, sz;
	unsigned char *datptr, *dat;

	if (iovcnt == 0) return 0;

	n = iov->sdiov_nsect;
	datptr = dat = iov->sdiov_base;
	sz = sdsize(sd);
	if (sector >= sz) return 0;

	if (sd->parked_sector) {
		if (!(sd->sd_state & SDDAT_TX) && sd->parked_sector == sector) 
		  goto receive;
		
		stop(sd);
	}

	if (sd->sd_state & SD_HC)
	  ret = sdcmd(sd, CMD_READ_MULTIPLE_BLOCK, sector, resp, &datptr);
	else
	  ret = sdcmd(sd, CMD_READ_MULTIPLE_BLOCK, sector * 512, resp, &datptr);
	if (ret || error(sd, resp, CMD_READ_MULTIPLE_BLOCK)) return 1;

receive:
	if (sd->os_dmaprep && sd->os_dmastream)
	  sd->os_dmaprep(sd->os_arg, datptr, n * 512 - (datptr - dat));

	datssp_stream(sd, &datptr, 512 - (datptr - dat));
	datssp_stream(sd, NULL, 6);

	sd->parked_sector = sector + n;
	if (sd->parked_sector > sz) {
		n -= sd->parked_sector - sz;
		sd->parked_sector = sz;
	}
	n--;

nextiov:
	while (n--) {
		SDPOKE8(sd, SDSTATE, S_WAIT_RESP | (TYPE_RXDAT << 5));
		datssp_stream(sd, NULL, 2); // last part of prev CRC
		datssp_stream(sd, &datptr, 512);
		datssp_stream(sd, NULL, 6); // first part of CRC
	}

	if (--iovcnt) {
		++iov;
		n = iov->sdiov_nsect;
		datptr = iov->sdiov_base;
		sd->parked_sector += n;
		if (sd->parked_sector > sz) {
			n -= sd->parked_sector - sz;
			sd->parked_sector = sz;
		}
		if (sd->os_dmaprep && sd->os_dmastream && n > 0)
		  sd->os_dmaprep(sd->os_arg, datptr, n * 512);
		goto nextiov;
	} 

	SDPOKE8(sd, SDSTATE, S_WAIT_RESP | (TYPE_RXDAT << 5));
	datssp_stream(sd, NULL, 2); // last part of prev CRC
	return 0;
}


static int do_write2(struct sdcore *sd, unsigned int sector, struct sdiov *iov, 
  unsigned int iovcnt) {
	unsigned char *datptr;
	unsigned int resp[6], ret, n, s, sz;

	if (sd->sd_wprot) return 1;

	if (iovcnt == 0) return 0;
	
	if (activate(sd)) return 1;

	sz = sdsize(sd);
	if (sector >= sz) return 0;

	if (sd->os_powerok) {
		int ok = sd->os_powerok(sd);
		if (!ok && sd->parked_sector) {
			stop2(sd);
			return 1;
		} else if (!ok) return 1;
	}

	if (sd->parked_sector) {
		if ((sd->sd_state & SDDAT_TX) && sd->parked_sector == sector)
		  goto transmit;
		
		stop2(sd);
	}

	if (sd->sd_erasehint) {
		sdcmd2(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
		ret = sdcmd2(sd, ACMD_SET_WR_BLK_ERASE_COUNT, sd->sd_erasehint, 
		  resp, NULL);
		if (ret||error(sd,resp, ACMD_SET_WR_BLK_ERASE_COUNT)) return 1;
		sd->sd_erasehint = 0;
	}

	if (sd->sd_nomultiwrite || sector == sz - 1) for (;;) {
		if (sd->sd_state & SD_HC)
		  ret = sdcmd2(sd, CMD_WRITE_BLOCK, sector, resp, NULL);
		else
		  ret = sdcmd2(sd, CMD_WRITE_BLOCK, sector * 512, resp, NULL);

		if (ret || error(sd, resp, CMD_WRITE_BLOCK)) return 1;
		s = SDPEEK8(sd, SDSTAT2); /* reset crc */
		if (s & 0x10) SDPOKE8(sd, SDSTAT2, s & ~0x10); /* tmp disable multiwrite */
		datptr = iov->sdiov_base;
		datssp_stream2(sd, &datptr, 512);
		sector++;	
		if (--iov->sdiov_nsect == 0) {
			iovcnt--;
			iov++;
		} else iov->sdiov_base += 512;
		SDPOKE8(sd, SDCTRL2, 0x0); /* busy wait */
		if (sd->os_irqwait) sd->os_irqwait(sd->os_arg, 2);
		SDPOKE8(sd, SDGPIO, 0xff);
		sd->sd_state &= ~SDDAT_TX;
		sd->parked_sector = 0;
		if (s & 0x10) {
			s = SDPEEK8(sd, SDSTAT2);
			SDPOKE8(sd, SDSTAT2, s | 0x10); /* reenab multiwrite */
		} else s = SDPEEK8(sd, SDSTAT2);
		if (s & 0x44) {
			sd->sd_timeout = 1000001;
			return 1;
		} else if (iovcnt == 0) return 0;
	}

	if (sd->sd_state & SD_HC)
	  ret = sdcmd2(sd, CMD_WRITE_MULTIPLE_BLOCK, sector, resp, NULL);
	else
	  ret = sdcmd2(sd, CMD_WRITE_MULTIPLE_BLOCK, sector * 512, resp, NULL);
	if (ret || error(sd, resp, CMD_WRITE_MULTIPLE_BLOCK)) {
		DBG(WRITE_FAIL, sector, ret);
		return 1;
	}
	sd->parked_sector = sector;
	SDPEEK8(sd, SDSTAT2);

transmit:
	while (iovcnt--) {
		datptr = iov->sdiov_base;
		n = iov->sdiov_nsect;
		sd->parked_sector += n;
		if (sd->parked_sector >= sz) { 
			struct sdiov riov;
			n -= sd->parked_sector - sz;

			if (n > 1) {
				datssp_stream2(sd, &datptr, (n - 1) * 512);
				ret = stop2(sd);
				if (ret) return ret;
			} 
			riov.sdiov_base = datptr;
			riov.sdiov_nsect = 1;
			ret = do_write2(sd, sz - 1, &riov, 1);
			return ret;
		} else datssp_stream2(sd, &datptr, n * 512);
		iov++;
	}

	if (!sd->sd_writeparking || sd->parked_sector == sz - 1) {
		ret = stop2(sd);
		if (ret) return ret;
	}

	if (sd->os_irqwait) sd->os_irqwait(sd->os_arg, 2);

	s = SDPEEK8(sd, SDSTAT2);
	if (s & 0x44) {
		if (s & 0x40) DBG(SD_HW_TMOUT, sector, s);
		if (s & 0x4) DBG(SD_DAT_BAD_CRC, sector, s);
		sd->sd_timeout = 1000001;
		return 1;
	} else {
		reset_timeout(sd);
		return 0;
	}
}


static int do_write(struct sdcore *sd, unsigned int sector, struct sdiov *iov, 
  unsigned int iovcnt) {
	unsigned char *datptr, *crcptr, **crcptrptr;
	unsigned int resp[6], ret, n, sz;

	if (sd->sd_wprot) return 1;

	if (iovcnt == 0) return 0;

	sz = sdsize(sd);
	if (sector >= sz) return 0;

	if (0 /* sd->sd_crchint */) {
		// CRC is pre-calculated so don't recalculate
		crcptr = sd->sd_crchint;
		crcptrptr = &crcptr;
		sd->sd_state |= DATSSP_NOCRC;
		sd->sd_crchint = NULL;
	} else {
		crcptrptr = NULL;
		sd->sd_state &= ~DATSSP_NOCRC;
	}

	if (sd->parked_sector) {
		if ((sd->sd_state & SDDAT_TX) && sd->parked_sector == sector) 
		  goto transmit;
		
		stop(sd);
	}

	if (sd->sd_erasehint) {
		sdcmd(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
		ret = sdcmd(sd, ACMD_SET_WR_BLK_ERASE_COUNT, sd->sd_erasehint, 
		  resp, NULL);
		if (ret || error(sd, resp, ACMD_SET_WR_BLK_ERASE_COUNT)) return 1;
		sd->sd_erasehint = 0;
	}

	if (sd->sd_state & SD_HC)
	  ret = sdcmd(sd, CMD_WRITE_MULTIPLE_BLOCK, sector, resp, NULL);
	else
	  ret = sdcmd(sd, CMD_WRITE_MULTIPLE_BLOCK, sector * 512, resp, NULL);
	if (ret || error(sd, resp, CMD_WRITE_MULTIPLE_BLOCK)) {
		return 1;
	}
	sd->parked_sector = sector;

transmit:
	while (iovcnt--) {
		datptr = iov->sdiov_base;
		n = iov->sdiov_nsect;
		sd->parked_sector += n;
		if (sd->parked_sector > sz) {
			n -= sd->parked_sector - sz;
			sd->parked_sector = sz;
		}
		while (n--) {
			datssp_stream(sd, &datptr, 512);
			datssp_stream(sd, crcptrptr, 8); // CRC bytes
			SDPOKE8(sd, SDSTATE, S_CRC_CHECK | (TYPE_TXDAT << 5));
		}
		iov++;
	}

	if (!sd->sd_writeparking) {
		stop(sd);
	}

	return 0;
}


static int sdfastinit(struct sdcore *sd) {
	SDPOKE8(sd, SDCTRL, 0x40);
	sd->sd_state = DATSSP_4BIT;

	sd->sd_rcaarg = ~sd->sdboot_token;
	sdcmd(sd, CMD_DESELECT_CARD, ~sd->sd_rcaarg, NULL, NULL);
	sdcmd(sd, CMD_SEND_CSD, sd->sd_rcaarg, sd->sd_csd, NULL);
	sdcmd(sd, CMD_SELECT_CARD, sd->sd_rcaarg, NULL, NULL);

	if (sd->os_dmastream) SDPOKE8(sd, SDCTRL, 0x42);
	if ((SDPEEK8(sd, SDCTRL) & 0x80) || (sd->sd_csd[15] & 0x30)) 
	  sd->sd_wprot = 1;
	sd->sd_blocksize = 1 << ((sd->sd_csd[6] & 0xf));
	if (timeout(sd)) return 0;
	else return sdsize(sd);
}


static void reset_common(struct sdcore *sd) {
	int i;

	reset_timeout(sd);
	sd_initcrc(sd);
	sd->parked_sector = 0;
	sd->sd_wprot = 0;
	sd->sd_blocksize = 0;
	sd->sd_sz = 0;
	for (i = 0; i < 17; i++) sd->sd_csd[i] = 0;
	if (sd->hw_version == 0) sd->hw_version = version(sd);
	if (sd->hw_version == 0) return;
	sd->sd_state &= SD_RESET;

	printk(KERN_INFO "HW Version is %d\n", sd->hw_version);

	if (sd->os_irqwait) sd->os_irqwait(sd->os_arg, 4);
	remember_sdcore(sd);
	activate(sd);
	sd->sd_state |= SD_LOSPEED;

	if (!(sd->sd_state & SD_RESET) && (SDPEEK8(sd, SDGPIO) != 0x0)) {
		SDPOKE8(sd, SDGPIO, 0x0);
#ifdef BIGENDIAN
		for (i = 0; i < 8; i++) SDPOKE16(sd, SDLUN2, i << 8);
#else
		for (i = 0; i < 8; i++) SDPOKE16(sd, SDLUN2, i);
#endif
		sd->os_delay(sd->os_arg, 100000);
	
		/* this was a global reset, so let the other luns know */
		for (i = 0; i < (sizeof(sdcores)/sizeof(sdcores[0])); i++) {
			if (sdcores[i] == NULL) break;
			if (sdcores[i]->sd_regstart == sd->sd_regstart)
			  sdcores[i]->sd_state |= SD_RESET;
		}
#ifdef BIGENDIAN
		SDPOKE16(sd, SDLUN2, sd->sd_lun << 8);
#else
		SDPOKE16(sd, SDLUN2, sd->sd_lun);
#endif
	}
	sd->sd_state &= ~SD_RESET;

	// gratuitous clocks
	SDPOKE8(sd, SDGPIO, 0xff);
	sd->os_delay(sd->os_arg, 5000);
	for (i = 0; i < (750 *2); i++) {
		SDPOKE8(sd, SDGPIO, 0xff);
		SDPEEK8(sd, SDGPIO); /* delay */
		SDPEEK8(sd, SDGPIO); /* delay */
		SDPOKE8(sd, SDGPIO, 0xdf);
		SDPEEK8(sd, SDGPIO); /* delay */
		SDPEEK8(sd, SDGPIO); /* delay */
	}

	SDPEEK8(sd, SDSTAT2); /* reset any timeout/crc conditions */
	if (sd->sd_nomultiwrite) SDPOKE8(sd, SDSTAT2, 0x8);
	else SDPOKE8(sd, SDSTAT2, 0x18);
}


#ifndef SD_NOMMC
static void mmc_enhance(struct sdcore *sd) {
	unsigned int s;
	unsigned char dat[512], *datptr;

	sdcmd2(sd, CMD_MMC_SEND_EXT_CSD, 0, NULL, NULL);
	do {
		if (timeout(sd)) break;
		SDPOKE8(sd, SDGPIO, 0xdf);
		s = SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xff);
	} while ((s & 0xf) == 0xf);

	if (sd->os_dmaprep && sd->os_dmastream)
	  sd->os_dmaprep(sd->os_arg, dat, 512);

	SDPOKE8(sd, SDGPIO, 0xdf);
	datptr = dat;
	datssp_stream2(sd, &datptr, 512);

	sd->sd_state &= ~SDDAT_RX;
	/* ERASE_GROUP_DEF */
	sdcmd2(sd, CMD_MMC_SWITCH, (175<<16)|(1<<8)|(3<<24), NULL, NULL);

	/* Enable Enhanced User data area, max size */
	sdcmd2(sd, CMD_MMC_SWITCH, (140<<16)|(dat[157]<<8)|(3<<24), NULL, NULL);
	sdcmd2(sd, CMD_MMC_SWITCH, (141<<16)|(dat[158]<<8)|(3<<24), NULL, NULL);
	sdcmd2(sd, CMD_MMC_SWITCH, (142<<16)|(dat[159]<<8)|(3<<24), NULL, NULL);
	sdcmd2(sd, CMD_MMC_SWITCH, (156<<16)|(1<<8)|(3<<24), NULL, NULL);

	/* Enable write reliability */
	sdcmd2(sd, CMD_MMC_SWITCH, (167<<16)|(1<<8)|(3<<24), NULL, NULL);

	/* Partition setting completed */
	sdcmd2(sd, CMD_MMC_SWITCH, (155<<16)|(1<<8)|(3<<24), NULL, NULL);
}


static int mmcreset2(struct sdcore *sd) {
	unsigned int s, i;
	unsigned int resp[17];
	unsigned char dat[512], *datptr;

	reset_common(sd);
	if (sd->hw_version == 0) return 0;

	sdcmd2(sd, CMD_GO_IDLE_STATE, 0, NULL, NULL);

	i = 0;
	do {
		sdcmd2(sd, CMD_MMC_SEND_OP_COND, 0xc0ff8000, resp, NULL);
		if (i > 30000) sd->sd_timeout = 1000001;
		if (timeout(sd)) break;
		i++;
	} while (((resp[1] & 0x80) == 0x0));

	sdcmd2(sd, CMD_ALL_SEND_CID, 0, resp, NULL);
	sdcmd2(sd, CMD_MMC_SET_RELATIVE_ADDR, 0x200, resp, NULL);
	sd->sd_rcaarg = 0x200;

	sdcmd2(sd, CMD_SELECT_CARD, sd->sd_rcaarg, resp, NULL);

	/* Enable 4-bit data bus, X_CSD byte 183 */
	sdcmd2(sd, CMD_MMC_SWITCH, (183<<16)|(1<<8)|(3<<24), NULL, NULL);
	/* Enable highest power, X_CSD byte 187 */ 
	sdcmd2(sd, CMD_MMC_SWITCH, (187<<16)|(15<<8)|(3<<24), NULL, NULL);
	/* Enable high speed 50Mhz data bus, X_CSD byte 185 */
	sdcmd2(sd, CMD_MMC_SWITCH, (185<<16)|(1<<8)|(3<<24), NULL, NULL);

	if (sd->sd_nomultiwrite) SDPOKE8(sd, SDSTAT2, 0x28);
	else SDPOKE8(sd, SDSTAT2, 0x38);

	sd->sd_state |= DATSSP_4BIT|SD_HISPEED|SD_HC;
	sd->sd_state &= ~SD_LOSPEED;

	sdcmd2(sd, CMD_SET_BLOCKLEN, 512, NULL, NULL);

	sdcmd2(sd, CMD_MMC_SEND_EXT_CSD, 0, NULL, NULL);
	do {
		if (timeout(sd)) break;
		SDPOKE8(sd, SDGPIO, 0xdf);
		s = SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xff);
	} while ((s & 0xf) == 0xf);

	if (sd->os_dmaprep && sd->os_dmastream)
	  sd->os_dmaprep(sd->os_arg, dat, 512);

	SDPOKE8(sd, SDGPIO, 0xdf);
	datptr = dat;
	datssp_stream2(sd, &datptr, 512);

	sd->sd_state &= ~SDDAT_RX;
	sd->sd_sz = dat[212] | (dat[213]<<8) | (dat[214]<<16) | (dat[215]<<24);

	if (timeout(sd)) return 0;
	else {
		reset_timeout(sd);
		if (sd->sd_sz) sd->sd_type = 1; /* eMMC, not SD */
		return sdsize(sd);
	}
}
#endif


static int sdreset2(struct sdcore *sd) {
	unsigned int rca, s, i, x;
	unsigned int resp[17];

#ifndef SD_NOMMC
	if (sd->sd_type == 1) return mmcreset2(sd); /* eMMC chip, not SD */
#endif
	reset_common(sd);
	if (sd->hw_version == 0) return 0;

	s = sdcmd2(sd, CMD_SEND_IF_COND, 0x1aa, resp, NULL);
	if (s) { 
		reset_timeout(sd); 
		x = 0x00ff8000;
	} else {
		x = 0x50ff8000;
	}

	i = 0;
	do {
		sdcmd2(sd, CMD_APP_CMD, 0, NULL, NULL);
		sdcmd2(sd, ACMD_SD_SEND_OP_COND, x, resp, NULL);
		if (i > 3000) sd->sd_timeout = 1000001;
		if (timeout(sd)) break;
		i++;
	} while (((resp[1] & 0x80) == 0x0));

	if ((x & 0x40000000) && (resp[1] & 0x40)) {
		sd->sd_state |= SD_HC;
	}

	sdcmd2(sd, CMD_ALL_SEND_CID, 0, resp, NULL);
	sdcmd2(sd, CMD_SEND_RELATIVE_ADDR, 0, resp, NULL);
	rca = resp[1] << 8 | resp[2];
	sd->sd_rcaarg = (rca & 0xff00) << 16 | (rca & 0xff) << 16;
	sd->sdboot_token = ~sd->sd_rcaarg;

	sdcmd2(sd, CMD_SEND_CSD, sd->sd_rcaarg, sd->sd_csd, NULL);
	sdcmd2(sd, CMD_SELECT_CARD, sd->sd_rcaarg, resp, NULL);

	if ((resp[1] & 0x2)) {
		sd->sd_locked = 1;
#ifndef SD_NOLOCKSUPPORT
		if (sd->sd_pwd) 
	 	  sdlockctl2(sd, SDLOCK_UNLOCK, sd->sd_pwd, NULL);
#endif
	} else sd->sd_locked = 0;

	sdcmd2(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
	sdcmd2(sd, ACMD_SET_CLR_CARD_DETECT, 0, NULL, NULL);
	sdcmd2(sd, CMD_SET_BLOCKLEN, 512, NULL, NULL);
	sdcmd2(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
	sdcmd2(sd, ACMD_SET_BUS_WIDTH, 2, resp, NULL);
	sd->sd_state |= DATSSP_4BIT;
	sd->sd_state &= ~SD_LOSPEED;

	sdcmd2(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
	sdcmd2(sd, ACMD_SEND_SCR2, 0, NULL, NULL);
	do {
		if (timeout(sd)) break;
		SDPOKE8(sd, SDGPIO, 0xdf);
		SDPEEK8(sd, SDGPIO);
		s = SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xff);
		SDPEEK8(sd, SDGPIO);
	} while ((s & 0xf) != 0x0);
	for (i = 0; i < 16; i++) {
		SDPOKE8(sd, SDGPIO, 0xdf);
		SDPEEK8(sd, SDGPIO);
		s = (SDPEEK8(sd, SDGPIO) & 0xf) << 4;
		SDPOKE8(sd, SDGPIO, 0xff);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xdf);
		SDPEEK8(sd, SDGPIO);
		s |= (SDPEEK8(sd, SDGPIO) & 0xf);
		SDPOKE8(sd, SDGPIO, 0xff);
		SDPEEK8(sd, SDGPIO);
		if (i < 8) sd->sd_scr[i] = s;
	}
	for (i = 0; i < 8; i++) {
		SDPOKE8(sd, SDGPIO, 0xdf);
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xff);
		SDPEEK8(sd, SDGPIO);
	}
	sd->sd_state &= ~SDDAT_RX;

#ifndef SD_NOHIGHSPEED
	if ((sd->sd_scr[0] & 0xf) >= 1) { // SD version >= 1.10
		unsigned char dat[64];
		sdcmd2(sd, CMD_SWITCH_FUNC2, 0x80fffff1, NULL, NULL);
		do {
			if (timeout(sd)) break;
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);
			s = SDPEEK8(sd, SDGPIO);
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);
		} while ((s & 0xf) != 0x0);
		for (i = 0; i < 72; i++) {
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);
			s = (SDPEEK8(sd, SDGPIO) & 0xf) << 4;
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);
			s |= (SDPEEK8(sd, SDGPIO) & 0xf);
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);
			if (i < 64) dat[i] = s;
		}
		for (i = 0; i < 8; i++) {
			SDPOKE8(sd, SDGPIO, 0xdf);
			SDPEEK8(sd, SDGPIO);
			SDPEEK8(sd, SDGPIO);
			SDPOKE8(sd, SDGPIO, 0xff);
			SDPEEK8(sd, SDGPIO);
		}
		sd->sd_state &= ~SDDAT_RX;
		if (dat[0] | dat[1]) {
			if (sd->sd_nomultiwrite) SDPOKE8(sd, SDSTAT2, 0x28);
			else SDPOKE8(sd, SDSTAT2, 0x38);
			sd->sd_state |= SD_HISPEED;
		}
	}
#endif

#ifdef BIGENDIAN
	if ((sd->sd_csd[15] & 0x30) || (SDPEEK16(sd, SDGPIO) & 0x2))
#else
	if ((sd->sd_csd[15] & 0x30) || (SDPEEK16(sd, SDGPIO) & 0x200))
#endif
		sd->sd_wprot = 1;
	sd->sd_blocksize = 1 << ((sd->sd_csd[6] & 0xf));
#ifndef SD_NOAUTOMMC
	if (timeout(sd) && (sd->sd_type == 0)) {
		/* if mmcreset2() works, sd_type is rewritten */
		sd->sd_type = 2;  /* SD, no eMMC */
		return mmcreset2(sd);
	}
#else
	if (timeout(sd)) return 0;
#endif
	else {
		reset_timeout(sd);
		return sdsize(sd);
	}
}


/*
 * return 0 : 8 bit TS-SDCORE v1
 * return 1 : 8 bit 4x8 TS-SDCORE v2
 * return 2 : 32 bit 4x32 TS-SDCORE v2
 * return 3 : 16 bit 4x32 TS-SDCORE v2
 * return 4 : 8 bit 4x32 TS-SDCORE v2
 */
static int version(struct sdcore *sd) {
	int a, b, i;


#ifdef SD_FORCEVERSION
	return SD_FORCEVERSION;
#endif
	for (i = 0; i < (sizeof(sdcores)/sizeof(sdcores[0])); i++) {
		if (sdcores[i] == NULL) break;
		if (sdcores[i]->sd_regstart == sd->sd_regstart) 
		  return sdcores[i]->hw_version;
	}

	a = SDPEEK8(sd, 3);
	SDPOKE8(sd, 3, (a ^ 0x40));
	b = SDPEEK8(sd, 3);
	SDPOKE8(sd, 3, a);
	if ((a & 0x40) ^ (b & 0x40)) return 0;
	else if (a & 0x40) return 1;
	/* either 2, 3, or 4 */
	a = SDPEEK32(sd, 12);
	b = SDPEEK16(sd, 12);
#ifdef BIGENDIAN
	if ((a & 0x40000000) && (b & 0x4000)) return 2;
#else
	if ((a & 0x40) && (b & 0x40)) return 2;
#endif
	a = SDPEEK8(sd, 12);
	if (a & 0x40) return 3;
	else return 4;
}


int sdreset(struct sdcore *sd) {
	unsigned int rca, s, x;
	unsigned int resp[17];
	int i;
	sd->refcnt++;
	BUG_ON(sd->refcnt != 1);

	reset_timeout(sd);
	sd_initcrc(sd);
	sd->parked_sector = 0;
	sd->sd_wprot = 0;
	sd->sd_blocksize = 0;
	sd->sd_sz = 0;

	sd->hw_version = version(sd);
	if (sd->hw_version >= 2) {
		sd->refcnt--;
		return sdreset2(sd);
	}

	// check for no SD card present
	if (SDPEEK8(sd, SDCTRL) & 0x8){
		sd->refcnt--;
		return 0;
	}

	if (sd->sdboot_token) {
		int ret = sdfastinit(sd);
		sd->sdboot_token = 0;
		if (ret){
			sd->refcnt--;
			return ret;
		}
	}

	// set controller for 1-bit mode, slow clock
	SDPOKE8(sd, SDCTRL, 0x20);

	SDPOKE8(sd, SDSTATE, S_DUMMY_CLK);
	sd->sd_state = SDCMD_RX|SDDAT_RX;
	s = SDPEEK8(sd, SDSTATE);
	while ((s & 0x7) != S_SEND_CMD) {
		// If we timeout here, it would be VERY BAD as we have no
		// further recourse to set things right if we can't turn
		// the SD off.
		if (timeout(sd)){
			sd->refcnt--;
			return 0;
		}
		sd->os_delay(sd->os_arg, 10000);
		sd->sd_timeout += 10000;

		// We won't be able to change state until both SSPs are empty
		s = tend_ssp(sd, NULL, NULL);
	}
	SDPOKE8(sd, SDSTATE, S_OFF);
	sd->sd_state = 0;

	sd->os_delay(sd->os_arg, 50000);

	SDPOKE8(sd, SDSTATE, S_DUMMY_CLK);
	sd->os_delay(sd->os_arg, 100000);
	if ((SDPEEK8(sd, SDSTATE) & 0x7) == S_OFF) {
		// No card present
		sd->refcnt--;
		return 0;
	}

	SDPOKE8(sd, SDSTATE, S_WAIT_RESP);
	// clock will freerun waiting for a response that will never come
	sd->os_delay(sd->os_arg, 50000);

	SDPOKE8(sd, SDSTATE, S_DUMMY_CLK);

	s = sdcmd(sd, CMD_SEND_IF_COND, 0x1aa, resp, NULL);
	if (s) { 
		reset_timeout(sd); 
		SDPOKE8(sd, SDSTATE, S_DUMMY_CLK);
		x = 0x00ff0000;
	} else {
		x = 0x50ff0000;
	}

	i = 0;
	do {
		sdcmd(sd, CMD_APP_CMD, 0, NULL, NULL);
		sdcmd(sd, ACMD_SD_SEND_OP_COND, x, resp, NULL);
		if (i > 3000) sd->sd_timeout = 1000001;
		if (timeout(sd)) break;
		i++;
	} while (((resp[1] & 0x80) == 0x0));

	if ((x & 0x40000000) && (resp[1] & 0x40)) sd->sd_state |= SD_HC;

	sdcmd(sd, CMD_ALL_SEND_CID, 0, resp, NULL);
	sdcmd(sd, CMD_SEND_RELATIVE_ADDR, 0, resp, NULL);
	rca = resp[1] << 8 | resp[2];
	sd->sd_rcaarg = (rca & 0xff00) << 16 | (rca & 0xff) << 16;
	sd->sdboot_token = ~sd->sd_rcaarg;

	sdcmd(sd, CMD_SEND_CSD, sd->sd_rcaarg, sd->sd_csd, NULL);
	sdcmd(sd, CMD_SELECT_CARD, sd->sd_rcaarg, resp, NULL);
	
	if ((resp[1] & 0x2)) {
		unsigned int ret = 1;
		sd->sd_locked = 1;
#ifndef SD_NOLOCKSUPPORT
		if (sd->sd_pwd) 
	 	  ret = sdlockctl(sd, SDLOCK_UNLOCK, sd->sd_pwd, NULL);
#endif
		if (ret != 0){
			sd->refcnt--;
			return 0;
		}
	} else sd->sd_locked = 0;

	sdcmd(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
	sdcmd(sd, ACMD_SET_CLR_CARD_DETECT, 0, NULL, NULL);
	/*
	sdcmd(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
	sdcmd(sd, ACMD_SEND_SCR, 0, NULL, &datptr);
	while ((datptr - sd->sd_scr) != 8) {
		if (timeout(sd)) return 1;
		tend_ssp(sd, NULL, &datptr);
	}
	datssp_stream(sd, NULL, 3);
	SDPOKE8(sd, SDSTATE, (TYPE_ABORT << 5) | S_SEND_CMD);
	sd->sd_state |= SDCMD_RX|SDDAT_RX;
	while ((SDPEEK8(sd, SDSTATE) & 0x17) != S_SEND_CMD) {
		if (timeout(sd)) break;
		tend_ssp(sd, NULL, NULL);
	}
	sd->sd_state &= ~(SDCMD_RX|SDDAT_RX);
	if ((sd->sd_scr[0] & 0xf) >= 1) { // SD version >= 1.10
		unsigned char dat[64];
		datptr = dat;
		sdcmd(sd, CMD_SWITCH_FUNC, 0x80fffff1, NULL, &datptr);
		while ((datptr - dat) != 64) {
			if (timeout(sd)) break;
			tend_ssp(sd, NULL, &datptr);
		}
		datssp_stream(sd, NULL, 3);
		SDPOKE8(sd, SDSTATE, (TYPE_ABORT << 5) | S_SEND_CMD);
		sd->sd_state |= SDCMD_RX|SDDAT_RX;
		while ((SDPEEK8(sd, SDSTATE) & 0x7) != S_SEND_CMD) {
			if (timeout(sd)) break;
			tend_ssp(sd, NULL, NULL);
		}
		sd->sd_state &= ~(SDCMD_RX|SDDAT_RX);
	}
	*/

	sdcmd(sd, CMD_SET_BLOCKLEN, 512, NULL, NULL);
	sdcmd(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
	sdcmd(sd, ACMD_SET_BUS_WIDTH, 2, resp, NULL);

	// set controller for 4-bit mode, fast clock
	SDPOKE8(sd, SDCTRL, (0x40 | (sd->os_dmastream ? 0x2 : 0x0)));
	sd->sd_state |= DATSSP_4BIT;
	
	/*
	sdcmd(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
	sdcmd(sd, ACMD_SEND_SCR, 0, NULL, &datptr);
	while ((datptr - sd->sd_scr) != 8) {
		if (timeout(sd)) break;
		tend_ssp(sd, NULL, &datptr);
	}
	datssp_stream(sd, NULL, 6);
	SDPOKE8(sd, SDSTATE, S_DUMMY_CLK | (TYPE_SHORTRESP << 5));
	bzero(resp, 6 * 4);
	sdcmd(sd, CMD_SEND_STATUS, 0, resp, NULL);
	*/

	if ((SDPEEK8(sd, SDCTRL) & 0x80) || (sd->sd_csd[15] & 0x30))
		sd->sd_wprot = 1;
	sd->sd_blocksize = 1 << ((sd->sd_csd[6] & 0xf));
	if (timeout(sd) || error(sd, resp, ACMD_SET_BUS_WIDTH)){
		sd->refcnt--;
		return 0;
	} else { 
		sd->refcnt--;
		return sdsize(sd);
	}
}


static
int sdread2(struct sdcore *sd, unsigned int sector, unsigned char *dat, 
  int nsectors) {
	struct sdiov iov;
	int ret;

	iov.sdiov_base = dat;
	iov.sdiov_nsect = nsectors;
	ret = do_read2(sd, sector, &iov, 1);
	return ret;
}


int sdread(struct sdcore *sd, unsigned int sector, unsigned char *dat, 
  int nsectors) {
	struct sdiov iov;
	int ret;
	sd->refcnt++;
	BUG_ON(sd->refcnt != 1);

	iov.sdiov_base = dat;
	iov.sdiov_nsect = nsectors;
	if (sd->hw_version == 0) ret = do_read(sd, sector, &iov, 1);
	else ret = do_read2(sd, sector, &iov, 1);
	sd->refcnt--;
	return ret;
}


int sdwrite(struct sdcore *sd, unsigned int sector, unsigned char *dat, 
  int nsectors) {
	struct sdiov iov;
	unsigned int ret;

	sd->refcnt++;
	BUG_ON(sd->refcnt != 1);

	iov.sdiov_base = dat;
	iov.sdiov_nsect = nsectors;
	if (sd->hw_version == 0) ret = do_write(sd, sector, &iov, 1);
	else ret = do_write2(sd, sector, &iov, 1);

	sd->refcnt--;
	return ret;
}


int sdreadv(struct sdcore *sd, unsigned int sector, struct sdiov *iov,
  int niov) {
	if (sd->hw_version == 0) return do_read(sd, sector, iov, niov);
	else return do_read2(sd, sector, iov, niov);
}


int sdwritev(struct sdcore *sd, unsigned int sector, struct sdiov *iov,
  int niov) {
	if (sd->hw_version == 0) return do_write(sd, sector, iov, niov);
	else return do_write2(sd, sector, iov, niov);
}


static int sdsetwprot2(struct sdcore *sd, unsigned int perm) {
	int i, ret, s;
	unsigned int csd[16], resp[6];
	unsigned char csdchars[16];
	unsigned char *csdptr = csdchars;

	stop2(sd);

	perm = perm ? 0x3 : 0x1;
	for (i = 0; i < 16; i++) csd[i] = sd->sd_csd[i + 1];
	csd[14] &= ~(0x3 << 4);
	csd[14] |= (perm << 4);
	csd[15] = 0x1 | crc7(0, csd, 15) << 1;
	for (i = 0; i < 16; i++) csdchars[i] = csd[i];

	ret = sdcmd2(sd, CMD_PROGRAM_CSD, 0, resp, NULL);
	if (ret || error(sd, resp, CMD_PROGRAM_CSD)) return 1;
	for (i = 0; i < 16; i++) {
		s = *csdptr++;
		sd_4bit_feedcrc(sd, s);
		SDPOKE8(sd, SDGPIO, (0x10|((s & 0xf0) >> 4))); 
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, (0x30|((s & 0xf0) >> 4)));
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, (0x10|(s & 0xf))); 
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, (0x30|(s & 0xf))); 
		SDPEEK8(sd, SDGPIO);
	}
	for (i = 0; i < 8; i++) {
		s = sd_4bit_getcrc(sd);
		SDPOKE8(sd, SDGPIO, (0x10|((s & 0xf0) >> 4))); 
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, (0x30|((s & 0xf0) >> 4)));
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, (0x10|(s & 0xf))); 
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, (0x30|(s & 0xf))); 
		SDPEEK8(sd, SDGPIO);
	}
	// End bit
	SDPOKE8(sd, SDGPIO, 0x1f);
	SDPEEK8(sd, SDGPIO);
	SDPEEK8(sd, SDGPIO);
	SDPOKE8(sd, SDGPIO, 0x3f);
	SDPEEK8(sd, SDGPIO);
	SDPOKE8(sd, SDGPIO, 0xbf);  //  tristate dat
	// CRC ack
	s = 0;
	for (i = 0; i < 7; i++) { 
		SDPOKE8(sd, SDGPIO, 0x9f);  // clk negedge
		SDPEEK8(sd, SDGPIO);        // delay
		s = s << 1;
		s |= (SDPEEK8(sd, SDGPIO) & 0x1);
		SDPOKE8(sd, SDGPIO, 0xbf);  // clk posedge
	}
	if ((s & 0xf) != 0x5) return 1;
	// wait for unbusy
	s = 0;
	while ((s & 0x7) != 0x7) {
		if (timeout(sd)) break;
		SDPOKE8(sd, SDGPIO, 0x9f);  // clk negedge
		SDPEEK8(sd, SDGPIO);        // delay
		s = s << 1;
		s |= SDPEEK8(sd, SDGPIO) & 0x1;
		SDPOKE8(sd, SDGPIO, 0xbf);
	}
	for (i = 0; i < 8; i++) {
		SDPOKE8(sd, SDGPIO, 0x9f);
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xbf);
		SDPEEK8(sd, SDGPIO);
	}
	sd->sd_state &= ~SDDAT_TX;

	sdcmd2(sd, CMD_DESELECT_CARD, ~sd->sd_rcaarg, NULL, NULL);
	ret = sdcmd2(sd, CMD_SEND_CSD, sd->sd_rcaarg, sd->sd_csd, NULL);
	if (ret || sd->sd_csd[15] != csd[14]) {
		return 1;
	}
	sdcmd2(sd, CMD_SELECT_CARD, sd->sd_rcaarg, resp, NULL);

	sd->sd_wprot = 1;	
	return 0;
}


int sdsetwprot(struct sdcore *sd, unsigned int perm) {
	int i, ret;
	unsigned int csd[16], resp[6];
	unsigned char csdchars[16];
	unsigned char *csdptr = csdchars;

	if (sd->hw_version) return sdsetwprot2(sd, perm);

	if (stop(sd)) return 1;

	perm = perm ? 0x3 : 0x1;
	for (i = 0; i < 16; i++) csd[i] = sd->sd_csd[i + 1];
	csd[14] &= ~(0x3 << 4);
	csd[14] |= (perm << 4);
	csd[15] = 0x1 | crc7(0, csd, 15) << 1;
	for (i = 0; i < 16; i++) csdchars[i] = csd[i];

	ret = sdcmd(sd, CMD_PROGRAM_CSD, 0, resp, NULL);
	if (ret || error(sd, resp, CMD_PROGRAM_CSD)) return 1;
	datssp_stream(sd, &csdptr, 16);
	datssp_stream(sd, NULL, 8);
	SDPOKE8(sd, SDSTATE, S_CRC_CHECK | (TYPE_BSYRESP << 5));
	sd->sd_state &= ~SDDAT_TX;

	sdcmd(sd, CMD_DESELECT_CARD, ~sd->sd_rcaarg, NULL, NULL);
	ret = sdcmd(sd, CMD_SEND_CSD, sd->sd_rcaarg, sd->sd_csd, NULL);
	if (ret || sd->sd_csd[15] != csd[14]) {
		return 1;
	}
	sdcmd(sd, CMD_SELECT_CARD, sd->sd_rcaarg, resp, NULL);

	sd->sd_wprot = 1;	
	return 0;
}


#ifndef SD_NOLOCKSUPPORT
int sdlockctl(struct sdcore *sd, unsigned int cmd, unsigned char *pwd, 
  unsigned char *sdbootdat) {
	unsigned char pwddat[18];
	unsigned char *pwdptr = pwddat;
	unsigned int resp[6];
	int ret, i, len;
	int ccc = (sd->sd_csd[5] << 4) | (sd->sd_csd[6] >> 4);

	if (sd->hw_version) return sdlockctl2(sd, cmd, pwd, sdbootdat);
	
	if (!(ccc & 0x80)) return 1; // Class 7 is lock-unlock commands
	
	if (pwd == NULL && cmd != SDLOCK_ERASE) return 1;
	
	if (stop(sd)) return 1;

	if (sd->sd_state & DATSSP_4BIT) {
		int oldctrl = SDPEEK8(sd, SDCTRL);
		int ret;

		sdcmd(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
		sdcmd(sd, ACMD_SET_BUS_WIDTH, 0, NULL, NULL);
		SDPOKE8(sd, SDCTRL, 0x20);
		sd->sd_state &= ~DATSSP_4BIT;
		ret = sdlockctl(sd, cmd, pwd, sdbootdat);
		sdcmd(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
		sdcmd(sd, ACMD_SET_BUS_WIDTH, 2, NULL, NULL);
		sd->sd_state |= DATSSP_4BIT;
		SDPOKE8(sd, SDCTRL, oldctrl);
		return ret;
	}
	
	pwddat[0] = cmd; 
	if (cmd != SDLOCK_ERASE) {
		pwddat[1] = 16; // length
		for (i = 0; i < 16; i++) {
			pwddat[2 + i] = pwd[i];
		}
	}

	if (cmd == SDLOCK_ERASE) len = 1; else len = 18;
	ret = sdcmd(sd, CMD_SET_BLOCKLEN, len, resp, NULL);
	if (ret || error(sd, resp, CMD_SET_BLOCKLEN)) return 1;
	ret = sdcmd(sd, CMD_LOCK_UNLOCK, 0, resp, NULL);
	if (ret || error(sd, resp, CMD_LOCK_UNLOCK)) return 1;

	while ((pwdptr - pwddat) != len) {
		if (timeout(sd)) return 1;
		tend_ssp(sd, NULL, &pwdptr);
	}

	if (sd->sd_state & DATSSP_4BIT) datssp_stream(sd, NULL, 8);
	else datssp_stream(sd, NULL, 2);

	SDPOKE8(sd, SDSTATE, S_CRC_CHECK | (TYPE_BSYRESP << 5));
	sd->sd_state &= ~SDDAT_TX;
	ret = sdcmd(sd, CMD_SET_BLOCKLEN, 512, resp, NULL);
	if (ret || error(sd, resp, CMD_SET_BLOCKLEN)) return 1;
	ret = sdcmd(sd, CMD_SEND_STATUS, sd->sd_rcaarg, resp, NULL);
	if (ret || error(sd, resp, CMD_SEND_STATUS)) return 1;
	
	if ((cmd == SDLOCK_ERASE || cmd == SDLOCK_UNLOCK ||
	  cmd == SDLOCK_CLRPWD) && (resp[1] & 0x2)) {
		return 1;
	}

	if (sdbootdat) {
		sdbootdat[0] = SDLOCK_UNLOCK;
		for (i = 1; i < 18; i++) {
			sdbootdat[i] = pwddat[i];
			sd_1bit_feedcrc(sd, pwddat[i]);
		}
		sdbootdat[18] = sd_1bit_getcrc(sd);
		sdbootdat[19] = sd_1bit_getcrc(sd);
	}
	
	return 0;
}


static
int sdlockctl2(struct sdcore *sd, unsigned int cmd, unsigned char *pwd, 
  unsigned char *sdbootdat) {
	unsigned char pwddat[18];
	unsigned char *pwdptr = pwddat;
	unsigned int resp[6];
	int ret, i, j, len, s;
	int ccc = (sd->sd_csd[5] << 4) | (sd->sd_csd[6] >> 4);
	
	if (!(ccc & 0x80)) return 1; // Class 7 is lock-unlock commands
	
	if (pwd == NULL && cmd != SDLOCK_ERASE) return 1;
	
	stop2(sd);

	if (sd->sd_state & DATSSP_4BIT) {
		int ret;

		sdcmd2(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
		sdcmd2(sd, ACMD_SET_BUS_WIDTH, 0, NULL, NULL);
		sd->sd_state &= ~DATSSP_4BIT;
		ret = sdlockctl2(sd, cmd, pwd, sdbootdat);
		sdcmd2(sd, CMD_APP_CMD, sd->sd_rcaarg, NULL, NULL);
		sdcmd2(sd, ACMD_SET_BUS_WIDTH, 2, NULL, NULL);
		sd->sd_state |= DATSSP_4BIT;
		return ret;
	}
	
	pwddat[0] = cmd; 
	if (cmd != SDLOCK_ERASE) {
		pwddat[1] = 16; // length
		for (i = 0; i < 16; i++) {
			pwddat[2 + i] = pwd[i];
		}
	}

	if (cmd == SDLOCK_ERASE) len = 1; else len = 18;
	ret = sdcmd2(sd, CMD_SET_BLOCKLEN, len, resp, NULL);
	if (ret || error(sd, resp, CMD_SET_BLOCKLEN)) return 1;
	ret = sdcmd2(sd, CMD_LOCK_UNLOCK, 0, resp, NULL);
	if (ret || error(sd, resp, CMD_LOCK_UNLOCK)) return 1;

	for (i = 0; i < len; i++) {
		unsigned int b = *pwdptr++;
		unsigned int x;

		sd_1bit_feedcrc(sd, b);
		for (j = 0; j < 8; j++) {
			x = 0x1e | ((b >> 7) & 0x1);
			b = b << 1;
			SDPOKE8(sd, SDGPIO, x);  // clk negedge
			SDPEEK8(sd, SDGPIO);
			SDPEEK8(sd, SDGPIO);
			x |= 0x20;
			SDPOKE8(sd, SDGPIO, x);  // clk posedge
			SDPEEK8(sd, SDGPIO);
		}
	}
	for (i = 0; i < 2; i++) {
		unsigned int b = sd_1bit_getcrc(sd); 
		unsigned int x;

		for (j = 0; j < 8; j++) {
			x = 0x1e | ((b >> 7) & 0x1);
			b = b << 1;
			SDPOKE8(sd, SDGPIO, x);  // clk negedge
			SDPEEK8(sd, SDGPIO);
			SDPEEK8(sd, SDGPIO);
			x |= 0x20;
			SDPOKE8(sd, SDGPIO, x);  // clk posedge
			SDPEEK8(sd, SDGPIO);
		}
	}
	// End bit 
	SDPOKE8(sd, SDGPIO, 0x1f);  // clk negedge
	SDPEEK8(sd, SDGPIO);
	SDPOKE8(sd, SDGPIO, 0xbf);  // clk posedge, tristate dat
	// CRC ack
	s = 0;
	for (i = 0; i < 7; i++) { 
		SDPOKE8(sd, SDGPIO, 0x9f);  // clk negedge
		SDPEEK8(sd, SDGPIO);        // delay
		s = s << 1;
		s |= SDPEEK8(sd, SDGPIO) & 0x1;
		SDPOKE8(sd, SDGPIO, 0xbf);  // clk posedge
		SDPEEK8(sd, SDGPIO);
	}
	if ((s & 0xf) != 0x5) return 1;

	// wait for unbusy
	s = 0;
	while ((s & 0x7) != 0x7) {
		if (timeout(sd)) break;
		SDPOKE8(sd, SDGPIO, 0x9f);  // clk negedge
		SDPEEK8(sd, SDGPIO);        // delay
		s = s << 1;
		s |= SDPEEK8(sd, SDGPIO) & 0x1;
		SDPOKE8(sd, SDGPIO, 0xbf);
		SDPEEK8(sd, SDGPIO);
	}
	for (i = 0; i < 8; i++) {
		SDPOKE8(sd, SDGPIO, 0x9f);
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xbf);
		SDPEEK8(sd, SDGPIO);
	}

	sd->sd_state &= ~SDDAT_TX;
	ret = sdcmd2(sd, CMD_SET_BLOCKLEN, 512, resp, NULL);
	if (ret || error(sd, resp, CMD_SET_BLOCKLEN)) {
		return 1;
	}
	ret = sdcmd2(sd, CMD_SEND_STATUS, sd->sd_rcaarg, resp, NULL);
	if (ret || error(sd, resp, CMD_SEND_STATUS)) {
		return 1;
	}
	
	if ((cmd == SDLOCK_ERASE || cmd == SDLOCK_UNLOCK ||
	  cmd == SDLOCK_CLRPWD) && (resp[1] & 0x2)) {
		return 1;
	}

	if (sdbootdat) {
		sdbootdat[0] = SDLOCK_UNLOCK;
		for (i = 1; i < 18; i++) {
			sdbootdat[i] = pwddat[i];
			sd_1bit_feedcrc(sd, pwddat[i]);
		}
		sdbootdat[18] = sd_1bit_getcrc(sd);
		sdbootdat[19] = sd_1bit_getcrc(sd);
	}

	for (i = 0; i < 8; i++) {
		SDPOKE8(sd, SDGPIO, 0x9f);
		SDPEEK8(sd, SDGPIO);
		SDPEEK8(sd, SDGPIO);
		SDPOKE8(sd, SDGPIO, 0xbf);
		SDPEEK8(sd, SDGPIO);
	}
	return 0;
}
#endif
#undef DBG
