// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX585 camera.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/* --------------------------------------------------------------------------
 * Driver-local custom controls
 * --------------------------------------------------------------------------
 */

#ifndef V4L2_CID_USER_IMX585_BASE
#define V4L2_CID_USER_IMX585_BASE (V4L2_CID_USER_BASE + 0x2000)
#endif

#define V4L2_CID_IMX585_HDR_DATASEL_TH  (V4L2_CID_USER_IMX585_BASE + 0)
#define V4L2_CID_IMX585_HDR_DATASEL_BK  (V4L2_CID_USER_IMX585_BASE + 1)
#define V4L2_CID_IMX585_HDR_GRAD_TH     (V4L2_CID_USER_IMX585_BASE + 2)
#define V4L2_CID_IMX585_HDR_GRAD_COMP_L (V4L2_CID_USER_IMX585_BASE + 3)
#define V4L2_CID_IMX585_HDR_GRAD_COMP_H (V4L2_CID_USER_IMX585_BASE + 4)
#define V4L2_CID_IMX585_HDR_GAIN        (V4L2_CID_USER_IMX585_BASE + 5)
#define V4L2_CID_IMX585_HCG_GAIN        (V4L2_CID_USER_IMX585_BASE + 6)
#define V4L2_CID_IMX585_VMAX            (V4L2_CID_USER_IMX585_BASE + 7)
#define V4L2_CID_IMX585_HMAX            (V4L2_CID_USER_IMX585_BASE + 8)
#define V4L2_CID_IMX585_SHR             (V4L2_CID_USER_IMX585_BASE + 9)

/* --------------------------------------------------------------------------
 * Registers / limits
 * --------------------------------------------------------------------------
 */

/* Standby or streaming mode */
#define IMX585_REG_MODE_SELECT          CCI_REG8(0x3000)
#define IMX585_MODE_STANDBY             0x01
#define IMX585_MODE_STREAMING           0x00
#define IMX585_STREAM_DELAY_US          25000
#define IMX585_STREAM_DELAY_RANGE_US    1000

/* Initialisation delay between XCLR low->high and the moment sensor is ready */
#define IMX585_XCLR_MIN_DELAY_US        500000
#define IMX585_XCLR_DELAY_RANGE_US      1000

/* Leader mode and XVS/XHS direction */
#define IMX585_REG_XMSTA                CCI_REG8(0x3002)
#define IMX585_REG_XXS_DRV              CCI_REG8(0x30a6)
#define IMX585_REG_EXTMODE              CCI_REG8(0x30ce)
#define IMX585_REG_XXS_OUTSEL           CCI_REG8(0x30a4)
#define IMX585_REG_MDBIT                CCI_REG8(0x3023)

/* XVS pulse length, 2^n H with n=0~3 */
#define IMX585_REG_XVSLNG               CCI_REG8(0x30cc)
/* XHS pulse length, 16*(2^n) Clock with n=0~3 */
#define IMX585_REG_XHSLNG               CCI_REG8(0x30cd)

/* Clock selection */
#define IMX585_INCK_SEL                 CCI_REG8(0x3014)

/* Link speed selector */
#define IMX585_DATARATE_SEL             CCI_REG8(0x3015)

/* BIN mode: 0x01 mono bin, 0x00 color */
#define IMX585_BIN_MODE                 CCI_REG8(0x3019)
/*
 * Window cropping (SRM page table — "WINMODE" register & "Restrictions on
 * Window cropping mode"). Setting WINMODE [4:0] = 14h enables a
 * pixel-array readout window defined by PIX_HST/PIX_HWIDTH/PIX_VST/
 * PIX_VWIDTH; the sensor then outputs ONLY those rows/cols, skipping the
 * 8-col + 20-row optical-black overhead the all-pixel readout includes.
 * Without this, the OB rows land at the top of the CFE buffer and the
 * downstream BE crop (which is geometric / aspect-ratio centred) leaks
 * the OB into the JPEG output as a black bar at the top of the frame.
 *
 * Register-set restrictions per SRM:
 *   PIX_HST      multiple of 2  (>0)
 *   PIX_HWIDTH   multiple of 16 (≥64)
 *   PIX_VST      multiple of 4  (>0)
 *   PIX_VWIDTH   multiple of 4  (≥239)
 *   VMAX         ≥ PIX_VWIDTH + 70
 */
#define IMX585_REG_WINMODE              CCI_REG8(0x3018)
#define IMX585_WINMODE_ALLPIXEL         0x10  /* initial value */
#define IMX585_WINMODE_CROP             0x14
#define IMX585_REG_PIX_HST              CCI_REG16_LE(0x303c)
#define IMX585_REG_PIX_HWIDTH           CCI_REG16_LE(0x303e)
#define IMX585_REG_PIX_VST              CCI_REG16_LE(0x3044)
#define IMX585_REG_PIX_VWIDTH           CCI_REG16_LE(0x3046)

/* Lane Count */
#define IMX585_LANEMODE                 CCI_REG8(0x3040)

/* VMAX internal VBLANK */
#define IMX585_REG_VMAX                 CCI_REG24_LE(0x3028)
#define IMX585_VMAX_MAX                 0xfffff
#define IMX585_VMAX_DEFAULT             2250
/* Min HMAX (4-lane) for the Clear HDR dual HG+LG read; see imx585_update_hmax */
#define IMX585_HMAX_MIN_CLEARHDR        550

/* HMAX internal HBLANK */
#define IMX585_REG_HMAX                 CCI_REG16_LE(0x302c)
#define IMX585_HMAX_MAX                 0xffff

/*
 * SHR0 (3050h) — coarse shutter sweep time (lines).
 *
 * AppNote page 5 "List of Setting Register": SHR0 minimum is "More than
 * 8h" in Normal mode and "More than 10h" in Clear HDR mode (= 16
 * decimal). Driver previously used 10 decimal in HDR which is below
 * spec.
 */
#define IMX585_REG_SHR                  CCI_REG24_LE(0x3050)
#define IMX585_SHR_MIN                  8
#define IMX585_SHR_MIN_HDR              16   /* AppNote §5 page 5 */
#define IMX585_SHR_MAX                  0xfffff

/* Exposure control (lines) */
#define IMX585_EXPOSURE_MIN             2
#define IMX585_EXPOSURE_STEP            1
#define IMX585_EXPOSURE_DEFAULT         1000
#define IMX585_EXPOSURE_MAX             49865

/* HDR threshold / blending / compression */
#define IMX585_REG_EXP_TH_H             CCI_REG16_LE(0x36d0)
#define IMX585_REG_EXP_TH_L             CCI_REG16_LE(0x36d4)
#define IMX585_REG_EXP_BK               CCI_REG8(0x36e2)
#define IMX585_REG_CCMP_EN              CCI_REG8(0x36ef)
#define IMX585_REG_CCMP1_EXP            CCI_REG24_LE(0x36e8)
#define IMX585_REG_CCMP2_EXP            CCI_REG24_LE(0x36e4)
#define IMX585_REG_ACMP1_EXP            CCI_REG8(0x36ee)
#define IMX585_REG_ACMP2_EXP            CCI_REG8(0x36ec)
#define IMX585_REG_EXP_GAIN             CCI_REG8(0x3081)

/* Black level control */
#define IMX585_REG_BLKLEVEL             CCI_REG16_LE(0x30dc)
#define IMX585_BLKLEVEL_DEFAULT         50


/* Digital Clamp */
#define IMX585_REG_DIGITAL_CLAMP        CCI_REG8(0x3458)

/* Analog gain control */
#define IMX585_REG_ANALOG_GAIN          CCI_REG16_LE(0x306c)
#define IMX585_REG_FDG_SEL0             CCI_REG8(0x3030)
#define IMX585_ANA_GAIN_MIN_NORMAL      0
#define IMX585_ANA_GAIN_MIN_HCG         34
#define IMX585_ANA_GAIN_MAX_NORMAL      240
/*
 * AppNote page 5 "List of Setting Register": GAIN range is 00h..50h
 * (0..80 decimal) — covers all modes including Clear HDR. The §5 page
 * 18 sum constraint `9.6dB ≤ GAIN + EXP_GAIN ≤ 29.1dB` for built-in
 * combination, with EXP_GAIN=12dB default, gives GAIN ≤ 17.1dB =
 * register 57. Use 80 here as the absolute register cap (the IPA owns
 * the per-mode tuning of the actual usable range above 57 if it cares).
 */
#define IMX585_ANA_GAIN_MAX_HDR         80
#define IMX585_ANA_GAIN_STEP            1
#define IMX585_ANA_GAIN_DEFAULT         0

/* Flip */
#define IMX585_FLIP_WINMODEH            CCI_REG8(0x3020)
#define IMX585_FLIP_WINMODEV            CCI_REG8(0x3021)

/* Test pattern generator */
#define IMX585_REG_TPG_EN_DUOUT		CCI_REG8(0x30E0)
#define IMX585_REG_TPG_TESTCLKEN		CCI_REG8(0x5300)
#define   IMX585_TPG_TESTCLKEN		BIT(3)

#define IMX585_REG_TPG_PATSEL		CCI_REG8(0x30E2)
#define   IMX585_TPG_PAT_ALL_000	0x00
#define   IMX585_TPG_PAT_ALL_FFF	0x01
#define   IMX585_TPG_PAT_ALL_555	0x02
#define   IMX585_TPG_PAT_ALL_AAA	0x03
#define   IMX585_TPG_PAT_H_COLOR_BARS	0x0a
#define   IMX585_TPG_PAT_V_COLOR_BARS	0x0b

static const char * const imx585_tpg_menu[] = {
	"Disabled",
	"All 000h",
	"All FFFh",
	"All 555h",
	"All AAAh",
	"Horizontal color bars",
	"Vertical color bars",
};

static const int imx585_tpg_val[] = {
	IMX585_TPG_PAT_ALL_000,
	IMX585_TPG_PAT_ALL_000,
	IMX585_TPG_PAT_ALL_FFF,
	IMX585_TPG_PAT_ALL_555,
	IMX585_TPG_PAT_ALL_AAA,
	IMX585_TPG_PAT_H_COLOR_BARS,
	IMX585_TPG_PAT_V_COLOR_BARS,
};

/* Pixel rate helper (sensor line clock proxy used below) */
#define IMX585_PIXEL_RATE               74250000U

/* Native and active array */
#define IMX585_NATIVE_WIDTH             3856U
#define IMX585_NATIVE_HEIGHT            2180U
#define IMX585_PIXEL_ARRAY_LEFT         8U
#define IMX585_PIXEL_ARRAY_WIDTH        3840U
#define IMX585_PIXEL_ARRAY_HEIGHT       2160U
/*
 * AppNote §3.1 page 8 ("Image Data output format") puts the OB region
 * at the top of the visible buffer:
 *   All-pixel:   H4 (Ignored OB) = 10 + H5 (Vertical effective OB) = 10
 *                → 20 rows of OB before the recording area.
 *   Binning:     H4 = 5 + H5 = 5 → 10 rows of OB.
 *
 * In Clear HDR, the OB rows contain stuck pixels that latch at the
 * HG saturation value (~35968) and would render as a speckle band /
 * black bar at the top of the JPEG if included in the active crop.
 * In SDR the same rows blend into the scene and aren't visible. The
 * crop offsets below skip both kinds of OB out of the active area.
 */
/*
 * Per-mode crop top: skip the OB rows. AppNote §3.1 page 8 lists 20
 * OB rows for All-pixel (H4=10 + H5=10) and 10 for binning (H4=H5=5)
 * at the top of the visible buffer. Use the OB count directly — the
 * buffer height equals per-mode top + recording area exactly (4K:
 * 20 + 2160 = 2180, binned: 10 + 1080 = 1090), so the BE's ScalerCrop
 * fits with no extra padding above or below.
 */
#define IMX585_PIXEL_ARRAY_TOP_4K       20U
#define IMX585_PIXEL_ARRAY_TOP_BIN      10U

/* Link frequency setup */
enum {
	IMX585_LINK_FREQ_297MHZ,   /* 594 Mbps/lane  */
	IMX585_LINK_FREQ_360MHZ,   /* 720 Mbps/lane  */
	IMX585_LINK_FREQ_445MHZ,   /* 891 Mbps/lane  */
	IMX585_LINK_FREQ_594MHZ,   /* 1188 Mbps/lane */
	IMX585_LINK_FREQ_720MHZ,   /* 1440 Mbps/lane */
	IMX585_LINK_FREQ_891MHZ,   /* 1782 Mbps/lane */
	IMX585_LINK_FREQ_1039MHZ,  /* 2079 Mbps/lane */
	IMX585_LINK_FREQ_1188MHZ,  /* 2376 Mbps/lane */
};

static const u8 link_freqs_reg_value[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 0x07,
	[IMX585_LINK_FREQ_360MHZ]  = 0x06,
	[IMX585_LINK_FREQ_445MHZ]  = 0x05,
	[IMX585_LINK_FREQ_594MHZ]  = 0x04,
	[IMX585_LINK_FREQ_720MHZ]  = 0x03,
	[IMX585_LINK_FREQ_891MHZ]  = 0x02,
	[IMX585_LINK_FREQ_1039MHZ] = 0x01,
	[IMX585_LINK_FREQ_1188MHZ] = 0x00,
};

static const u64 link_freqs[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 297000000ULL,
	[IMX585_LINK_FREQ_360MHZ]  = 360000000ULL,
	[IMX585_LINK_FREQ_445MHZ]  = 445500000ULL,
	[IMX585_LINK_FREQ_594MHZ]  = 594000000ULL,
	[IMX585_LINK_FREQ_720MHZ]  = 720000000ULL,
	[IMX585_LINK_FREQ_891MHZ]  = 891000000ULL,
	[IMX585_LINK_FREQ_1039MHZ] = 1039500000ULL,
	[IMX585_LINK_FREQ_1188MHZ] = 1188000000ULL,
};

/* min HMAX for 4-lane 4K full res mode, x2 for 2-lane */
static const u16 HMAX_table_4lane_4K_12bit[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 1584,
	[IMX585_LINK_FREQ_360MHZ]  = 1320,
	[IMX585_LINK_FREQ_445MHZ]  = 1100,
	[IMX585_LINK_FREQ_594MHZ]  = 792,
	[IMX585_LINK_FREQ_720MHZ]  = 660,
	[IMX585_LINK_FREQ_891MHZ]  = 550,
	/*
	 * 2079 Mbps/lane RAW12 is not listed in the standard all-pixel table.
	 * HMAX=440 advertises roughly 75 fps but produced broken low-signal
	 * frames on Pi 5/RP1. HMAX=472 is the verified-clean timing used for
	 * RAW12 reference captures.
	 */
	[IMX585_LINK_FREQ_1039MHZ] = 472,
	[IMX585_LINK_FREQ_1188MHZ] = 472,
};

static const u16 HMAX_table_4lane_4K_10bit[] = {
	[IMX585_LINK_FREQ_297MHZ]  = 1320,
	[IMX585_LINK_FREQ_360MHZ]  = 1100,
	[IMX585_LINK_FREQ_445MHZ]  = 1100,
	[IMX585_LINK_FREQ_594MHZ]  = 660,
	[IMX585_LINK_FREQ_720MHZ]  = 550,
	[IMX585_LINK_FREQ_891MHZ]  = 550,
	/*
	 * Sony's 2079 Mbps/lane RAW10 row lists HMAX=366, VMAX=2250 for the
	 * 90.1 fps all-pixel mode. On the Pi 5/RP1 path, same-scene RAW10 vs
	 * RAW12 QA showed deterministic artifacts at HMAX=366. HMAX=375 with
	 * VMAX=2200 is the validated 90 fps clean timing on this DUT.
	 */
	[IMX585_LINK_FREQ_1039MHZ] = 375,
	[IMX585_LINK_FREQ_1188MHZ] = 376,
};

struct imx585_inck_cfg {
	u32 xclk_hz;
	u8  inck_sel;
};

static const struct imx585_inck_cfg imx585_inck_table[] = {
	{ 74250000, 0x00 },
	{ 37125000, 0x01 },
	{ 72000000, 0x02 },
	{ 27000000, 0x03 },
	{ 24000000, 0x04 },
};

static const char * const hdr_gain_adder_menu[] = {
	"+0dB", "+6dB", "+12dB", "+18dB", "+24dB", "+29.1dB",
};

/*
 * EXP_BK register values per AppNote §4.2 page 15. Indices 0-7 are valid;
 * higher values are "Setting Prohibited". Spec lists two 50/50 entries
 * (indices 0 and 4) — keep both, label them clearly.
 */
static const char * const hdr_data_blender_menu[] = {
	"HG 1/2, LG 1/2",                /* 0h */
	"HG 3/4, LG 1/4",                /* 1h */
	"HG 7/8, LG 1/8",                /* 2h */
	"HG 15/16, LG 1/16",             /* 3h */
	"HG 1/2, LG 1/2 (alt)",          /* 4h */
	"HG 1/16, LG 15/16",             /* 5h */
	"HG 1/8, LG 7/8",                /* 6h */
	"HG 1/4, LG 3/4",                /* 7h */
};

static const char * const grad_compression_slope_menu[] = {
	"1/1", "1/2", "1/4",  "1/8",   "1/16", "1/32",
	"1/64", "1/128", "1/256", "1/512", "1/1024", "1/2048",
};

enum {
	SYNC_INT_LEADER,
	SYNC_INT_FOLLOWER,
	SYNC_EXTERNAL,
};

static const char * const sync_mode_menu[] = {
	"Internal Sync Leader Mode",
	"External Sync Leader Mode",
	"Follower Mode",
};

/* Mode description */
struct imx585_mode {
	unsigned int width;
	unsigned int height;

	u8  hmax_div;       /* per-mode scaling of min HMAX */
	const u16 *hmax_table;
	u16 min_hmax;       /* computed at runtime */
	u32 min_vmax;       /* computed at runtime (fits 20-bit) */

	struct v4l2_rect crop;

	struct {
		unsigned int num_of_regs;
		const struct cci_reg_sequence *regs;
	} reg_list;
};

/* --------------------------------------------------------------------------
 * Register tables
 * --------------------------------------------------------------------------
 */

static const struct cci_reg_sequence common_regs[] = {
	{ CCI_REG8(0x3002), 0x01 },
	{ CCI_REG8(0x3069), 0x00 },
	{ CCI_REG8(0x3074), 0x64 },
	{ CCI_REG8(0x30d5), 0x04 }, /* DIG_CLP_VSTART */
	{ CCI_REG8(0x3030), 0x00 }, /* FDG_SEL0 LCG (HCG=0x01) */
	{ CCI_REG8(0x30a6), 0x00 }, /* XVS_DRV [1:0] Hi-Z */
	{ CCI_REG8(0x3081), 0x00 }, /* EXP_GAIN reset */
	{ CCI_REG8(0x303a), 0x03 }, /* Disable embedded data */

	/* The remaining blocks are datasheet-recommended settings */
	{ CCI_REG8(0x3460), 0x21 }, { CCI_REG8(0x3478), 0xa1 },
	{ CCI_REG8(0x347c), 0x01 }, { CCI_REG8(0x3480), 0x01 },
	{ CCI_REG8(0x3a4e), 0x14 }, { CCI_REG8(0x3a52), 0x14 },
	{ CCI_REG8(0x3a56), 0x00 }, { CCI_REG8(0x3a5a), 0x00 },
	{ CCI_REG8(0x3a5e), 0x00 }, { CCI_REG8(0x3a62), 0x00 },
	{ CCI_REG8(0x3a6a), 0x20 }, { CCI_REG8(0x3a6c), 0x42 },
	{ CCI_REG8(0x3a6e), 0xa0 }, { CCI_REG8(0x3b2c), 0x0c },
	{ CCI_REG8(0x3b30), 0x1c }, { CCI_REG8(0x3b34), 0x0c },
	{ CCI_REG8(0x3b38), 0x1c }, { CCI_REG8(0x3ba0), 0x0c },
	{ CCI_REG8(0x3ba4), 0x1c }, { CCI_REG8(0x3ba8), 0x0c },
	{ CCI_REG8(0x3bac), 0x1c }, { CCI_REG8(0x3d3c), 0x11 },
	{ CCI_REG8(0x3d46), 0x0b }, { CCI_REG8(0x3de0), 0x3f },
	{ CCI_REG8(0x3de1), 0x08 }, { CCI_REG8(0x3e14), 0x87 },
	{ CCI_REG8(0x3e16), 0x91 }, { CCI_REG8(0x3e18), 0x91 },
	{ CCI_REG8(0x3e1a), 0x87 }, { CCI_REG8(0x3e1c), 0x78 },
	{ CCI_REG8(0x3e1e), 0x50 }, { CCI_REG8(0x3e20), 0x50 },
	{ CCI_REG8(0x3e22), 0x50 }, { CCI_REG8(0x3e24), 0x87 },
	{ CCI_REG8(0x3e26), 0x91 }, { CCI_REG8(0x3e28), 0x91 },
	{ CCI_REG8(0x3e2a), 0x87 }, { CCI_REG8(0x3e2c), 0x78 },
	{ CCI_REG8(0x3e2e), 0x50 }, { CCI_REG8(0x3e30), 0x50 },
	{ CCI_REG8(0x3e32), 0x50 }, { CCI_REG8(0x3e34), 0x87 },
	{ CCI_REG8(0x3e36), 0x91 }, { CCI_REG8(0x3e38), 0x91 },
	{ CCI_REG8(0x3e3a), 0x87 }, { CCI_REG8(0x3e3c), 0x78 },
	{ CCI_REG8(0x3e3e), 0x50 }, { CCI_REG8(0x3e40), 0x50 },
	{ CCI_REG8(0x3e42), 0x50 }, { CCI_REG8(0x4054), 0x64 },
	{ CCI_REG8(0x4148), 0xfe }, { CCI_REG8(0x4149), 0x05 },
	{ CCI_REG8(0x414a), 0xff }, { CCI_REG8(0x414b), 0x05 },
	{ CCI_REG8(0x420a), 0x03 }, { CCI_REG8(0x4231), 0x08 },
	{ CCI_REG8(0x423d), 0x9c }, { CCI_REG8(0x4242), 0xb4 },
	{ CCI_REG8(0x4246), 0xb4 }, { CCI_REG8(0x424e), 0xb4 },
	{ CCI_REG8(0x425c), 0xb4 }, { CCI_REG8(0x425e), 0xb6 },
	{ CCI_REG8(0x426c), 0xb4 }, { CCI_REG8(0x426e), 0xb6 },
	{ CCI_REG8(0x428c), 0xb4 }, { CCI_REG8(0x428e), 0xb6 },
	{ CCI_REG8(0x4708), 0x00 }, { CCI_REG8(0x4709), 0x00 },
	{ CCI_REG8(0x470a), 0xff }, { CCI_REG8(0x470b), 0x03 },
	{ CCI_REG8(0x470c), 0x00 }, { CCI_REG8(0x470d), 0x00 },
	{ CCI_REG8(0x470e), 0xff }, { CCI_REG8(0x470f), 0x03 },
	{ CCI_REG8(0x47eb), 0x1c }, { CCI_REG8(0x47f0), 0xa6 },
	{ CCI_REG8(0x47f2), 0xa6 }, { CCI_REG8(0x47f4), 0xa0 },
	{ CCI_REG8(0x47f6), 0x96 }, { CCI_REG8(0x4808), 0xa6 },
	{ CCI_REG8(0x480a), 0xa6 }, { CCI_REG8(0x480c), 0xa0 },
	{ CCI_REG8(0x480e), 0x96 }, { CCI_REG8(0x492c), 0xb2 },
	{ CCI_REG8(0x4930), 0x03 }, { CCI_REG8(0x4932), 0x03 },
	{ CCI_REG8(0x4936), 0x5b }, { CCI_REG8(0x4938), 0x82 },
	{ CCI_REG8(0x493e), 0x23 }, { CCI_REG8(0x4ba8), 0x1c },
	{ CCI_REG8(0x4ba9), 0x03 }, { CCI_REG8(0x4bac), 0x1c },
	{ CCI_REG8(0x4bad), 0x1c }, { CCI_REG8(0x4bae), 0x1c },
	{ CCI_REG8(0x4baf), 0x1c }, { CCI_REG8(0x4bb0), 0x1c },
	{ CCI_REG8(0x4bb1), 0x1c }, { CCI_REG8(0x4bb2), 0x1c },
	{ CCI_REG8(0x4bb3), 0x1c }, { CCI_REG8(0x4bb4), 0x1c },
	{ CCI_REG8(0x4bb8), 0x03 }, { CCI_REG8(0x4bb9), 0x03 },
	{ CCI_REG8(0x4bba), 0x03 }, { CCI_REG8(0x4bbb), 0x03 },
	{ CCI_REG8(0x4bbc), 0x03 }, { CCI_REG8(0x4bbd), 0x03 },
	{ CCI_REG8(0x4bbe), 0x03 }, { CCI_REG8(0x4bbf), 0x03 },
	{ CCI_REG8(0x4bc0), 0x03 }, { CCI_REG8(0x4c14), 0x87 },
	{ CCI_REG8(0x4c16), 0x91 }, { CCI_REG8(0x4c18), 0x91 },
	{ CCI_REG8(0x4c1a), 0x87 }, { CCI_REG8(0x4c1c), 0x78 },
	{ CCI_REG8(0x4c1e), 0x50 }, { CCI_REG8(0x4c20), 0x50 },
	{ CCI_REG8(0x4c22), 0x50 }, { CCI_REG8(0x4c24), 0x87 },
	{ CCI_REG8(0x4c26), 0x91 }, { CCI_REG8(0x4c28), 0x91 },
	{ CCI_REG8(0x4c2a), 0x87 }, { CCI_REG8(0x4c2c), 0x78 },
	{ CCI_REG8(0x4c2e), 0x50 }, { CCI_REG8(0x4c30), 0x50 },
	{ CCI_REG8(0x4c32), 0x50 }, { CCI_REG8(0x4c34), 0x87 },
	{ CCI_REG8(0x4c36), 0x91 }, { CCI_REG8(0x4c38), 0x91 },
	{ CCI_REG8(0x4c3a), 0x87 }, { CCI_REG8(0x4c3c), 0x78 },
	{ CCI_REG8(0x4c3e), 0x50 }, { CCI_REG8(0x4c40), 0x50 },
	{ CCI_REG8(0x4c42), 0x50 }, { CCI_REG8(0x4d12), 0x1f },
	{ CCI_REG8(0x4d13), 0x1e }, { CCI_REG8(0x4d26), 0x33 },
	{ CCI_REG8(0x4e0e), 0x59 }, { CCI_REG8(0x4e14), 0x55 },
	{ CCI_REG8(0x4e16), 0x59 }, { CCI_REG8(0x4e1e), 0x3b },
	{ CCI_REG8(0x4e20), 0x47 }, { CCI_REG8(0x4e22), 0x54 },
	{ CCI_REG8(0x4e26), 0x81 }, { CCI_REG8(0x4e2c), 0x7d },
	{ CCI_REG8(0x4e2e), 0x81 }, { CCI_REG8(0x4e36), 0x63 },
	{ CCI_REG8(0x4e38), 0x6f }, { CCI_REG8(0x4e3a), 0x7c },
	{ CCI_REG8(0x4f3a), 0x3c }, { CCI_REG8(0x4f3c), 0x46 },
	{ CCI_REG8(0x4f3e), 0x59 }, { CCI_REG8(0x4f42), 0x64 },
	{ CCI_REG8(0x4f44), 0x6e }, { CCI_REG8(0x4f46), 0x81 },
	{ CCI_REG8(0x4f4a), 0x82 }, { CCI_REG8(0x4f5a), 0x81 },
	{ CCI_REG8(0x4f62), 0xaa }, { CCI_REG8(0x4f72), 0xa9 },
	{ CCI_REG8(0x4f78), 0x36 }, { CCI_REG8(0x4f7a), 0x41 },
	{ CCI_REG8(0x4f7c), 0x61 }, { CCI_REG8(0x4f7d), 0x01 },
	{ CCI_REG8(0x4f7e), 0x7c }, { CCI_REG8(0x4f7f), 0x01 },
	{ CCI_REG8(0x4f80), 0x77 }, { CCI_REG8(0x4f82), 0x7b },
	{ CCI_REG8(0x4f88), 0x37 }, { CCI_REG8(0x4f8a), 0x40 },
	{ CCI_REG8(0x4f8c), 0x62 }, { CCI_REG8(0x4f8d), 0x01 },
	{ CCI_REG8(0x4f8e), 0x76 }, { CCI_REG8(0x4f8f), 0x01 },
	{ CCI_REG8(0x4f90), 0x5e }, { CCI_REG8(0x4f91), 0x02 },
	{ CCI_REG8(0x4f92), 0x69 }, { CCI_REG8(0x4f93), 0x02 },
	{ CCI_REG8(0x4f94), 0x89 }, { CCI_REG8(0x4f95), 0x02 },
	{ CCI_REG8(0x4f96), 0xa4 }, { CCI_REG8(0x4f97), 0x02 },
	{ CCI_REG8(0x4f98), 0x9f }, { CCI_REG8(0x4f99), 0x02 },
	{ CCI_REG8(0x4f9a), 0xa3 }, { CCI_REG8(0x4f9b), 0x02 },
	{ CCI_REG8(0x4fa0), 0x5f }, { CCI_REG8(0x4fa1), 0x02 },
	{ CCI_REG8(0x4fa2), 0x68 }, { CCI_REG8(0x4fa3), 0x02 },
	{ CCI_REG8(0x4fa4), 0x8a }, { CCI_REG8(0x4fa5), 0x02 },
	{ CCI_REG8(0x4fa6), 0x9e }, { CCI_REG8(0x4fa7), 0x02 },
	{ CCI_REG8(0x519e), 0x79 }, { CCI_REG8(0x51a6), 0xa1 },
	{ CCI_REG8(0x51f0), 0xac }, { CCI_REG8(0x51f2), 0xaa },
	{ CCI_REG8(0x51f4), 0xa5 }, { CCI_REG8(0x51f6), 0xa0 },
	{ CCI_REG8(0x5200), 0x9b }, { CCI_REG8(0x5202), 0x91 },
	{ CCI_REG8(0x5204), 0x87 }, { CCI_REG8(0x5206), 0x82 },
	{ CCI_REG8(0x5208), 0xac }, { CCI_REG8(0x520a), 0xaa },
	{ CCI_REG8(0x520c), 0xa5 }, { CCI_REG8(0x520e), 0xa0 },
	{ CCI_REG8(0x5210), 0x9b }, { CCI_REG8(0x5212), 0x91 },
	{ CCI_REG8(0x5214), 0x87 }, { CCI_REG8(0x5216), 0x82 },
	{ CCI_REG8(0x5218), 0xac }, { CCI_REG8(0x521a), 0xaa },
	{ CCI_REG8(0x521c), 0xa5 }, { CCI_REG8(0x521e), 0xa0 },
	{ CCI_REG8(0x5220), 0x9b }, { CCI_REG8(0x5222), 0x91 },
	{ CCI_REG8(0x5224), 0x87 }, { CCI_REG8(0x5226), 0x82 },
};

static const struct cci_reg_sequence common_clearHDR_mode[] = {
	{ CCI_REG8(0x301a), 0x10 }, /* WDMODE: Clear HDR */
	{ CCI_REG8(0x3024), 0x02 }, /* COMBI_EN: with built-in combination */
	{ CCI_REG8(0x3069), 0x02 },
	{ CCI_REG8(0x3074), 0x63 },
	{ CCI_REG8(0x3930), 0xe6 }, /* DUR[15:8] (12-bit) */
	{ CCI_REG8(0x3931), 0x00 }, /* DUR[7:0]  (12-bit) */
	{ CCI_REG8(0x3a4c), 0x61 }, { CCI_REG8(0x3a4d), 0x02 },
	{ CCI_REG8(0x3a50), 0x70 }, { CCI_REG8(0x3a51), 0x02 },
	{ CCI_REG8(0x3e10), 0x17 }, /* ADTHEN */
	{ CCI_REG8(0x493c), 0x41 }, /* 10-bit HDR */
	{ CCI_REG8(0x4940), 0x41 }, /* 12-bit HDR */
	{ CCI_REG8(0x3081), 0x02 }, /* EXP_GAIN: +12 dB default */
	/*
	 * HG/LG selection thresholds (§4.2, page 15).
	 *
	 * The AppNote's "initial value" of EXP_TH_H = EXP_TH_L = 0x1000
	 * documents a fallback to the EXP_BK weighted blend, but empirically
	 * that path leaves the combiner output clamped near BLC for typical
	 * scenes (verified at LED 5500K @ 80% on this rig: HDR-16 DNG max
	 * stays at ~4200 with TH_H=TH_L=0x1000, vs ~36000 with TH_H=0xFFF /
	 * TH_L=0). Use the rule-based selection range instead so HG drives
	 * the bulk of the output and LG only takes over once HG saturates.
	 */
	{ CCI_REG8(0x36d0), 0xFF }, { CCI_REG8(0x36d1), 0x0F }, /* EXP_TH_H = 0x0FFF (HG saturation cutoff) */
	{ CCI_REG8(0x36d4), 0x00 }, { CCI_REG8(0x36d5), 0x00 }, /* EXP_TH_L = 0x0000 (no low cutoff) */
	{ CCI_REG8(0x36e2), 0x00 },                              /* EXP_BK   = HG 1/2, LG 1/2 (only used in overlap) */
	/*
	 * Spec-valid CCMP gradation-compression slopes (§4.3, page 16). These
	 * must land in their register's allowed range or the sensor output
	 * clamps at BLC. ACMP1 (middle segment) must be 06h..0Bh; ACMP2 (high
	 * segment) must be 00h..05h.
	 */
	{ CCI_REG8(0x36ec), 0x04 }, /* ACMP2_EXP = 1/16  (high slope; natural inverse spans 16-bit, no LUT stretch needed) */
	{ CCI_REG8(0x36ee), 0x06 }, /* ACMP1_EXP = 1/64  (middle slope) */
};

static const struct cci_reg_sequence common_normal_mode[] = {
	{ CCI_REG8(0x301a), 0x00 }, /* WDMODE: Normal */
	{ CCI_REG8(0x3024), 0x00 }, /* COMBI_EN */
	{ CCI_REG8(0x3069), 0x00 },
	{ CCI_REG8(0x3074), 0x64 },
	{ CCI_REG8(0x3930), 0x0c }, /* DUR[15:8] (12-bit) */
	{ CCI_REG8(0x3931), 0x01 }, /* DUR[7:0]  (12-bit) */
	{ CCI_REG8(0x3a4c), 0x39 }, { CCI_REG8(0x3a4d), 0x01 },
	{ CCI_REG8(0x3a50), 0x48 }, { CCI_REG8(0x3a51), 0x01 },
	{ CCI_REG8(0x3e10), 0x10 }, /* ADTHEN */
	{ CCI_REG8(0x493c), 0x23 }, /* 10-bit Normal */
	{ CCI_REG8(0x4940), 0x23 }, /* 12-bit Normal */
};

/*
 * Window-crop registers — PIX_VST=12 lands the cropping window at the H8
 * recording top (skipping H6 ignored=4 + H7 margin=8 = 12 lines per
 * AppNote ClearHDR §3). PIX_HST=8 is the equivalent horizontal margin.
 *
 * Two PIX_VWIDTH variants exist:
 *
 *  - 12-bit mode (SDR + ClearHDR-12 CCMP): PIX_VWIDTH = 2160 = active
 *    recording height. Sensor outputs PIX_VWIDTH rows (no OB), buffer
 *    is exactly the recording area — pisp.cpp's centered-aspect crop
 *    falls at offset 0 with no OB to leak.
 *
 *  - 16-bit ClearHDR: the sensor PRE-pends 20 OB rows in this format
 *    regardless of WINMODE (the cropping suppresses OB for COMP1/RAW12
 *    but not for RAW16 — likely because the CFE accepts every CSI2 DT
 *    when csi_dt=0 and the sensor uses a different DT for OB). To make
 *    pisp.cpp's centered crop land cleanly on the recording area, we
 *    extend PIX_VWIDTH to 2180 so the sensor emits 20 OB + 2180 = 2200
 *    rows, the buffer dim is advertised as 2200, and the centered
 *    aspect crop offset is (2200-2160)/2 = 20 — exactly the OB count.
 *    The extra 20 PIX_VWIDTH rows past H8 read into H9 margin (8) +
 *    H10 (1) + start of vertical blanking, which the BE crop discards
 *    along with the OB.
 */
#define IMX585_WIN_CROP_REGS_COMMON \
	{ IMX585_REG_WINMODE,    IMX585_WINMODE_CROP }, \
	{ IMX585_REG_PIX_HST,    8    }, /* skip H-margin   */ \
	{ IMX585_REG_PIX_HWIDTH, 3840 }, /* active width    */ \
	{ IMX585_REG_PIX_VST,    12   }  /* H6 + H7         */
#define IMX585_WIN_CROP_REGS_12BIT \
	IMX585_WIN_CROP_REGS_COMMON, \
	{ IMX585_REG_PIX_VWIDTH, 2160 }  /* active height   */
#define IMX585_WIN_CROP_REGS_16BIT \
	IMX585_WIN_CROP_REGS_COMMON, \
	{ IMX585_REG_PIX_VWIDTH, 2180 }  /* active + 20 to compensate for OB prepend */

/* All-pixel 4K, 12-bit */
static const struct cci_reg_sequence mode_4k_regs_12bit[] = {
	{ CCI_REG8(0x301b), 0x00 }, /* ADDMODE non-binning */
	{ CCI_REG8(0x3022), 0x02 }, /* ADBIT 12-bit */
	{ IMX585_REG_MDBIT, 0x01 }, /* MDBIT 12-bit */
	{ CCI_REG8(0x30d5), 0x04 }, /* DIG_CLP_VSTART non-binning */
	IMX585_WIN_CROP_REGS_12BIT,
};

/* All-pixel 4K, 10-bit */
static const struct cci_reg_sequence mode_4k_regs_10bit[] = {
	{ CCI_REG8(0x301b), 0x00 }, /* ADDMODE non-binning */
	{ CCI_REG8(0x3022), 0x00 }, /* ADBIT 10-bit */
	{ IMX585_REG_MDBIT, 0x00 }, /* MDBIT RAW10 */
	{ CCI_REG8(0x30d5), 0x04 }, /* DIG_CLP_VSTART non-binning */
	{ CCI_REG8(0x3930), 0x66 }, /* DUR[15:8] (10-bit) */
	{ CCI_REG8(0x3931), 0x00 }, /* DUR[7:0]  (10-bit) */
	IMX585_WIN_CROP_REGS_12BIT,
};

/* 2x2 binned 1080p, 12-bit */
static const struct cci_reg_sequence mode_1080_regs_12bit[] = {
	{ CCI_REG8(0x301b), 0x01 }, /* ADDMODE binning */
	{ CCI_REG8(0x3022), 0x02 }, /* ADBIT 12-bit */
	{ IMX585_REG_MDBIT, 0x01 }, /* MDBIT 12-bit */
	{ CCI_REG8(0x30d5), 0x02 }, /* DIG_CLP_VSTART binning */
	IMX585_WIN_CROP_REGS_12BIT,
};

/*
 * All-pixel 4K, 16-bit ClearHDR. Identical to the 12-bit table except
 * PIX_VWIDTH is bumped to 2180 — see comment on IMX585_WIN_CROP_REGS_16BIT
 * for the rationale (compensates for the 20 OB rows the sensor prepends
 * in 16-bit RAW16 output, so pisp.cpp's centered crop lands at offset 20
 * and skips them cleanly). MDBIT is overridden to 0x03 (RAW16) at runtime
 * in start_streaming.
 */
static const struct cci_reg_sequence mode_4k_regs_16bit[] = {
	{ CCI_REG8(0x301b), 0x00 }, /* ADDMODE non-binning */
	{ CCI_REG8(0x3022), 0x02 }, /* ADBIT 12-bit */
	{ IMX585_REG_MDBIT, 0x01 }, /* MDBIT 12-bit (overridden to 0x03 at runtime) */
	{ CCI_REG8(0x30d5), 0x04 }, /* DIG_CLP_VSTART non-binning */
	IMX585_WIN_CROP_REGS_16BIT,
};

/* --------------------------------------------------------------------------
 * Mode list
 * --------------------------------------------------------------------------
 * Default:
 *   12Bit - FHD, 4K
 * ClearHDR Enabled:
 *   12bit + Gradation compression
 *   16bit - FHD, 4K
 *
 * Gradation compression is available on 12bit
 * With Default option, only 12bit mode is exposed
 * With ClearHDR enabled via parameters,
 *   12bit will be with Gradation compression enabled
 *   16bit mode exposed
 *
 * Technically, because the sensor is actually binning
 * in digital domain, its readout speed is the same
 * between 4K and FHD. However, through testing it is
 * possible to "overclock" the FHD mode, thus leaving the
 * hmax_div option for those who want to try.
 * Also, note that FHD and 4K mode shared the same VMAX.
 */

/*
 * Mode array layout:
 *   [0] 1080p binned (12-bit; ClearHDR FHD binning is unusable)
 *   [1] 4K all-pixel for 12-bit formats (SDR + ClearHDR-12 CCMP).
 *       Sensor-side WINMODE crop strips the OB region — buffer = active.
 *   [2] 4K all-pixel for 16-bit ClearHDR. The sensor still emits 20 OB
 *       rows at the top of the buffer in this format (CFE accepts every
 *       CSI2 packet type because csi_dt=0 for RAW16, and no IMX585
 *       register suppresses the H4+H5 OB-row output). Advertise height
 *       = active + 20, set crop.top = 20 so libcamera/BE skip the OB.
 *
 * get_mode_table() routes 12-bit → modes [0..1], 16-bit → mode [2].
 */
enum imx585_mode_id {
	IMX585_MODE_1080P_12BIT,
	IMX585_MODE_4K_12BIT,
	IMX585_MODE_4K_16BIT_HDR,
};

static struct imx585_mode supported_modes[] = {
	{
		/* 1080p60 2x2 binning, 12-bit */
		.width = IMX585_PIXEL_ARRAY_WIDTH / 2,   /* 1920 */
		.height = IMX585_PIXEL_ARRAY_HEIGHT / 2, /* 1080 */
		.hmax_div = 1,
		.hmax_table = HMAX_table_4lane_4K_12bit,
		.min_hmax = 366,            /* overwritten at runtime */
		.min_vmax = IMX585_VMAX_DEFAULT,
		.crop = {
			.left = 0,
			.top = 0,
			.width = IMX585_PIXEL_ARRAY_WIDTH / 2,
			.height = IMX585_PIXEL_ARRAY_HEIGHT / 2,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1080_regs_12bit),
			.regs = mode_1080_regs_12bit,
		},
	},
	{
		/* 4K60 all-pixel, 12-bit (SDR + ClearHDR-12 CCMP) */
		.width = IMX585_PIXEL_ARRAY_WIDTH,   /* 3840 */
		.height = IMX585_PIXEL_ARRAY_HEIGHT, /* 2160 */
		.hmax_div = 1,
		.hmax_table = HMAX_table_4lane_4K_12bit,
		.min_hmax = 550,            /* overwritten at runtime */
		.min_vmax = IMX585_VMAX_DEFAULT,
		.crop = {
			.left = 0,
			.top = 0,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_regs_12bit),
			.regs = mode_4k_regs_12bit,
		},
	},
	{
		/*
		 * 4K60 all-pixel, 16-bit ClearHDR. Buffer height = active
		 * 2160 + 2*20 padding so pisp.cpp's centered aspect crop
		 * lands at offset 20 and skips both the 20-row OB prepend
		 * and the equal margin below. The 16-bit reg sequence sets
		 * PIX_VWIDTH=2180 so the sensor emits exactly 2200 rows
		 * (20 OB + 2180 cropped recording-extended into H9 margin).
		 */
		.width = IMX585_PIXEL_ARRAY_WIDTH,                                  /* 3840 */
		.height = IMX585_PIXEL_ARRAY_HEIGHT + 2 * IMX585_PIXEL_ARRAY_TOP_4K,/* 2200 */
		.hmax_div = 1,
		.hmax_table = HMAX_table_4lane_4K_12bit,
		.min_hmax = 550,
		.min_vmax = IMX585_VMAX_DEFAULT,
		.crop = {
			.left = 0,
			.top = 0,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_regs_16bit),
			.regs = mode_4k_regs_16bit,
		},
	},
};

static struct imx585_mode supported_10bit_modes[] = {
	{
		/* Cropped UHD RAW10 at 90 fps when the 2079 Mbps/lane link is selected */
		.width = IMX585_PIXEL_ARRAY_WIDTH,   /* 3840 */
		.height = IMX585_PIXEL_ARRAY_HEIGHT, /* 2160 */
		.hmax_div = 1,
		.hmax_table = HMAX_table_4lane_4K_10bit,
		.min_hmax = 366,            /* overwritten at runtime */
		.min_vmax = IMX585_VMAX_DEFAULT,
		.crop = {
			.left = 0,
			.top = 0,
			.width = IMX585_PIXEL_ARRAY_WIDTH,
			.height = IMX585_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_regs_10bit),
			.regs = mode_4k_regs_10bit,
		},
	},
};

/* Formats exposed per mode/bit depth */
static const u32 codes_normal[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const u32 codes_clearhdr[] = {
	/* 16-bit first */
	MEDIA_BUS_FMT_SRGGB16_1X16,
	MEDIA_BUS_FMT_SGRBG16_1X16,
	MEDIA_BUS_FMT_SGBRG16_1X16,
	MEDIA_BUS_FMT_SBGGR16_1X16,
	/* then 12-bit */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

static const u32 mono_codes[] = {
	MEDIA_BUS_FMT_Y16_1X16,
	MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_Y10_1X10,
};

/* Regulators */
static const char * const imx585_supply_name[] = {
	"vana", /* 3.3V analog */
	"vdig", /* 1.1V core   */
	"vddl", /* 1.8V I/O    */
};

#define IMX585_NUM_SUPPLIES ARRAY_SIZE(imx585_supply_name)

/* --------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------
 */

struct imx585 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *clientdev;
	struct regmap *regmap;

	struct clk *xclk;
	u32 xclk_freq;
	u8  inck_sel_val;

	unsigned int lane_count;
	unsigned int link_freq_idx;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX585_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;

	/* Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *hcg_ctrl;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *blacklevel;

	/* RAW Controls */
	struct v4l2_ctrl *vmax_ctrl;
	struct v4l2_ctrl *hmax_ctrl;
	struct v4l2_ctrl *shr_ctrl;

	/* HDR controls */
	struct v4l2_ctrl *hdr_mode;
	struct v4l2_ctrl *datasel_th_ctrl;
	struct v4l2_ctrl *datasel_bk_ctrl;
	struct v4l2_ctrl *gdc_th_ctrl;
	struct v4l2_ctrl *gdc_exp_ctrl_l;
	struct v4l2_ctrl *gdc_exp_ctrl_h;
	struct v4l2_ctrl *hdr_gain_ctrl;

	/* Flags/params */
	bool hcg;
	bool mono;
	bool clearhdr_ccmp;
	bool clear_hdr;

	/*
	 * Sync Mode
	 * 0 = Internal Sync Leader Mode
	 * 1 = External Sync Leader Mode
	 * 2 = Follower Mode
	 * The datasheet wording is very confusing but basically:
	 * Leader Mode = Sensor using internal clock to drive the sensor
	 * But with external sync mode you can send a XVS input so the sensor
	 * will try to align with it.
	 * For Follower mode it is purely driven by external clock.
	 * In this case you need to drive both XVS and XHS.
	 */
	u8   sync_mode;

	u16  hmax;
	u32  vmax;

	bool streaming;
	bool common_regs_written;
};

/* Helpers */

static inline struct imx585 *to_imx585(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx585, sd);
}

static int imx585_update_test_pattern(struct imx585 *imx585, u32 pattern_index)
{
	int ret;

	if (pattern_index >= ARRAY_SIZE(imx585_tpg_val))
		return -EINVAL;

	if (!pattern_index) {
		ret = cci_write(imx585->regmap, IMX585_REG_TPG_EN_DUOUT, 0x00, NULL);
		if (ret)
			return ret;

		return cci_write(imx585->regmap, IMX585_REG_TPG_TESTCLKEN, 0x02, NULL);
	}

	ret = cci_write(imx585->regmap, IMX585_REG_TPG_PATSEL,
			imx585_tpg_val[pattern_index], NULL);
	if (ret)
		return ret;

	ret = cci_write(imx585->regmap, IMX585_REG_TPG_EN_DUOUT, 0x01, NULL);
	if (ret)
		return ret;

	return cci_write(imx585->regmap, IMX585_REG_TPG_TESTCLKEN, 0x0A, NULL);
}

static bool imx585_is_clearhdr_16bit_code(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_Y16_1X16:
		return true;
	default:
		return false;
	}
}

static bool imx585_is_clearhdr_12bit_code(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		return true;
	default:
		return false;
	}
}

static void imx585_activate_hdr_controls(struct imx585 *imx585)
{
	const bool clear_hdr = imx585->clear_hdr;
	const bool ccmp = clear_hdr && imx585->clearhdr_ccmp;

	v4l2_ctrl_activate(imx585->datasel_th_ctrl, clear_hdr);
	v4l2_ctrl_activate(imx585->datasel_bk_ctrl, clear_hdr);
	v4l2_ctrl_activate(imx585->gdc_th_ctrl, ccmp);
	v4l2_ctrl_activate(imx585->gdc_exp_ctrl_h, ccmp);
	v4l2_ctrl_activate(imx585->gdc_exp_ctrl_l, ccmp);
	v4l2_ctrl_activate(imx585->hdr_gain_ctrl, clear_hdr);
	v4l2_ctrl_activate(imx585->hcg_ctrl, !clear_hdr);
}

static inline void get_mode_table(struct imx585 *imx585, unsigned int code,
				  const struct imx585_mode **mode_list,
				  unsigned int *num_modes)
{
	*mode_list = NULL;
	*num_modes = 0;

	if (imx585->mono) {
		/* --- Mono paths ---
		 * Y16 only valid in Clear HDR. 4K-only (binning unusable).
		 * Use the 16-bit-specific mode entry for the buffer-with-OB
		 * layout. Y12 routes to the 12-bit modes.
		 */
		if (code == MEDIA_BUS_FMT_Y16_1X16 && imx585->clear_hdr) {
			*mode_list = &supported_modes[IMX585_MODE_4K_16BIT_HDR];
			*num_modes = 1;
		} else if (code == MEDIA_BUS_FMT_Y12_1X12) {
			if (imx585->clear_hdr) {
				if (imx585->clearhdr_ccmp) {
					*mode_list = &supported_modes[IMX585_MODE_4K_12BIT];
					*num_modes = 1;
				}
			} else {
				*mode_list = supported_modes;     /* binned + 4K 12-bit */
				*num_modes = 2;
			}
		} else if (code == MEDIA_BUS_FMT_Y10_1X10 && !imx585->clear_hdr) {
			*mode_list = supported_10bit_modes;   /* 4K 10-bit */
			*num_modes = ARRAY_SIZE(supported_10bit_modes);
		}
	} else {
		/* --- Color paths --- */
		switch (code) {
		/* 16-bit (Clear HDR linear, only valid when WDR=1).
		 *
		 * 4K-only — binned Clear HDR is unusable. Routes to mode [2]
		 * which advertises height = active + 20 OB rows so the buffer
		 * covers the OB region the sensor still emits in this format.
		 */
		case MEDIA_BUS_FMT_SRGGB16_1X16:
		case MEDIA_BUS_FMT_SGRBG16_1X16:
		case MEDIA_BUS_FMT_SGBRG16_1X16:
		case MEDIA_BUS_FMT_SBGGR16_1X16:
			*mode_list = &supported_modes[IMX585_MODE_4K_16BIT_HDR];
			*num_modes = 1;
			break;

		/* 12-bit. Per AppNote §2 page 6, the 1920×1080 binning mode in
		 * Clear HDR only supports 16-bit output — 12-bit binned HDR is
		 * not a valid sensor configuration and the part returns BLC if
		 * asked. Skip the binning entry (index 0) when WDR=1, leaving
		 * only the 4K all-pixel mode at index 1. */
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
		case MEDIA_BUS_FMT_SBGGR12_1X12:
			if (imx585->clear_hdr) {
				if (imx585->clearhdr_ccmp) {
					*mode_list = &supported_modes[IMX585_MODE_4K_12BIT];
					*num_modes = 1;
				}
			} else {
				*mode_list = supported_modes;         /* binned + 4K */
				*num_modes = 2;                       /* exclude 16-bit entry */
			}
			break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
		case MEDIA_BUS_FMT_SBGGR10_1X10:
			if (!imx585->clear_hdr) {
				*mode_list = supported_10bit_modes;
				*num_modes = ARRAY_SIZE(supported_10bit_modes);
			}
			break;
		default:
			*mode_list = NULL;
			*num_modes = 0;
		}
	}
}

static bool imx585_code_in_table(const u32 *table, unsigned int num_entries,
				 u32 code)
{
	unsigned int i;

	for (i = 0; i < num_entries; i++)
		if (table[i] == code)
			return true;

	return false;
}

static u32 imx585_get_format_code(struct imx585 *imx585, u32 code)
{
	u32 mapped_code = 0;
	unsigned int i;

	if (imx585->mono) {
		if (imx585->clear_hdr) {
			if (code == MEDIA_BUS_FMT_Y16_1X16)
				return code;
			if (imx585->clearhdr_ccmp &&
			    code == MEDIA_BUS_FMT_Y12_1X12)
				return code;
			return MEDIA_BUS_FMT_Y16_1X16;
		}

		if (code == MEDIA_BUS_FMT_Y12_1X12 ||
		    code == MEDIA_BUS_FMT_Y10_1X10)
			return code;
		return MEDIA_BUS_FMT_Y12_1X12;
	}

	if (imx585->clear_hdr) {
		for (i = 0; i < ARRAY_SIZE(codes_clearhdr); i++)
			if (codes_clearhdr[i] == code &&
			    (imx585_is_clearhdr_16bit_code(code) ||
			     imx585->clearhdr_ccmp))
				return codes_clearhdr[i];
		return codes_clearhdr[0];
	}

	for (i = 0; i < ARRAY_SIZE(codes_normal); i++)
		if (codes_normal[i] == code)
			return codes_normal[i];

	switch (code) {
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		mapped_code = MEDIA_BUS_FMT_SRGGB12_1X12;
		break;
	case MEDIA_BUS_FMT_SGRBG16_1X16:
		mapped_code = MEDIA_BUS_FMT_SGRBG12_1X12;
		break;
	case MEDIA_BUS_FMT_SGBRG16_1X16:
		mapped_code = MEDIA_BUS_FMT_SGBRG12_1X12;
		break;
	case MEDIA_BUS_FMT_SBGGR16_1X16:
		mapped_code = MEDIA_BUS_FMT_SBGGR12_1X12;
		break;
	default:
		break;
	}

	if (mapped_code &&
	    imx585_code_in_table(codes_normal, ARRAY_SIZE(codes_normal),
				 mapped_code))
		return mapped_code;

	return codes_normal[0];
}

static u32 imx585_default_format_code(struct imx585 *imx585)
{
	if (imx585->mono)
		return imx585->clear_hdr ? MEDIA_BUS_FMT_Y16_1X16 :
					   MEDIA_BUS_FMT_Y12_1X12;

	return imx585->clear_hdr ? MEDIA_BUS_FMT_SRGGB16_1X16 :
				   MEDIA_BUS_FMT_SRGGB12_1X12;
}

static const struct imx585_mode *imx585_default_mode(struct imx585 *imx585)
{
	return imx585->clear_hdr ?
	       &supported_modes[IMX585_MODE_4K_16BIT_HDR] :
	       &supported_modes[IMX585_MODE_1080P_12BIT];
}

static const struct imx585_mode *
imx585_find_mode(struct imx585 *imx585, u32 code, u32 req_width,
		 u32 req_height)
{
	const struct imx585_mode *mode_list;
	unsigned int num_modes;

	get_mode_table(imx585, code, &mode_list, &num_modes);
	if (!mode_list || !num_modes)
		return imx585_default_mode(imx585);

	return v4l2_find_nearest_size(mode_list, num_modes, width, height,
				      req_width, req_height);
}

static const struct imx585_mode *
imx585_state_get_mode(struct imx585 *imx585, struct v4l2_subdev_state *state,
		      u32 *code)
{
	struct v4l2_mbus_framefmt *fmt;
	u32 fmt_code = imx585_default_format_code(imx585);

	if (WARN_ON(!state)) {
		*code = fmt_code;
		return imx585_default_mode(imx585);
	}

	fmt = v4l2_subdev_state_get_format(state, 0);
	if (WARN_ON(!fmt)) {
		*code = fmt_code;
		return imx585_default_mode(imx585);
	}

	fmt_code = imx585_get_format_code(imx585, fmt->code);
	*code = fmt_code;

	return imx585_find_mode(imx585, fmt_code, fmt->width, fmt->height);
}

static void imx585_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

/* Update analogue gain limits based on mode/HDR/HCG */
static void imx585_update_gain_limits(struct imx585 *imx585)
{
	const bool hcg_on = imx585->hcg;
	const bool clear_hdr = imx585->clear_hdr;
	const u32 min = hcg_on ? IMX585_ANA_GAIN_MIN_HCG : IMX585_ANA_GAIN_MIN_NORMAL;
	const u32 max = clear_hdr ? IMX585_ANA_GAIN_MAX_HDR : IMX585_ANA_GAIN_MAX_NORMAL;
	u32 cur = imx585->gain->val;

	__v4l2_ctrl_modify_range(imx585->gain, min, max, IMX585_ANA_GAIN_STEP,
				 clamp(cur, min, max));

	if (cur < min || cur > max)
		__v4l2_ctrl_s_ctrl(imx585->gain, clamp(cur, min, max));
}

/* Recompute per-mode timing limits (HMAX/VMAX) from link/lanes/HDR */
static void imx585_update_hmax(struct imx585 *imx585)
{
	const u32 lane_scale = (imx585->lane_count == 2) ? 2 : 1;
	const u32 hdr_scale  = imx585->clear_hdr ? 2 : 1;
	unsigned int i;

	dev_info(imx585->clientdev, "Update minimum HMAX: link_freq=%llu lane_scale=%u hdr_scale=%u\n",
		 link_freqs[imx585->link_freq_idx], lane_scale, hdr_scale);

	for (i = 0; i < ARRAY_SIZE(supported_modes); ++i) {
		u32 h = supported_modes[i].hmax_table[imx585->link_freq_idx] *
			lane_scale / supported_modes[i].hmax_div;
		u32 v = IMX585_VMAX_DEFAULT * hdr_scale;

		/*
		 * Clear HDR always does the dual HG+LG read, so the line readout
		 * has a higher minimum (~HMAX 550 at 4-lane) than single-read
		 * SDR — independent of output depth (12-bit CCMP or 16-bit). The
		 * 12-bit/link-bandwidth tables drop below that at >=2079 Mbps/lane
		 * (HMAX 472), where the HDR frame loses ~94% of its rows. Floor it
		 * so Clear HDR runs on the high-speed link without a slower
		 * dedicated hdr-link-frequency.
		 */
		if (imx585->clear_hdr)
			h = max_t(u32, h, IMX585_HMAX_MIN_CLEARHDR * lane_scale);

		supported_modes[i].min_hmax = h;
		supported_modes[i].min_vmax = v;

		dev_info(imx585->clientdev, " mode %ux%u -> VMAX=%u HMAX=%u\n",
			 supported_modes[i].width, supported_modes[i].height, v, h);
	}

	for (i = 0; i < ARRAY_SIZE(supported_10bit_modes); ++i) {
		u32 h = supported_10bit_modes[i].hmax_table[imx585->link_freq_idx] *
			lane_scale / supported_10bit_modes[i].hmax_div;
		u32 v = IMX585_VMAX_DEFAULT * hdr_scale;

		if (imx585->link_freq_idx == IMX585_LINK_FREQ_1039MHZ)
			v = 2200 * hdr_scale;

		supported_10bit_modes[i].min_hmax = h;
		supported_10bit_modes[i].min_vmax = v;

		dev_info(imx585->clientdev, " mode %ux%u -> VMAX=%u HMAX=%u\n",
			 supported_10bit_modes[i].width,
			 supported_10bit_modes[i].height, v, h);
	}
}

static void imx585_set_framing_limits(struct imx585 *imx585,
				      const struct imx585_mode *mode)
{
	u64 pixel_rate;
	u64 max_hblank;

	imx585_update_hmax(imx585);

	imx585->vmax = mode->min_vmax;
	imx585->hmax = mode->min_hmax;

	/* Pixel rate proxy: width * clock / min_hmax */
	pixel_rate = (u64)mode->width * IMX585_PIXEL_RATE;
	do_div(pixel_rate, mode->min_hmax);
	__v4l2_ctrl_modify_range(imx585->pixel_rate, pixel_rate, pixel_rate, 1,
				 pixel_rate);

	max_hblank = (u64)IMX585_HMAX_MAX * pixel_rate;
	do_div(max_hblank, IMX585_PIXEL_RATE);
	max_hblank -= mode->width;

	__v4l2_ctrl_modify_range(imx585->hblank, 0, max_hblank, 1, 0);
	__v4l2_ctrl_s_ctrl(imx585->hblank, 0);

	__v4l2_ctrl_modify_range(imx585->vblank,
				 mode->min_vmax - mode->height,
				 IMX585_VMAX_MAX - mode->height,
				 1, mode->min_vmax - mode->height);
	__v4l2_ctrl_s_ctrl(imx585->vblank, mode->min_vmax - mode->height);

	__v4l2_ctrl_modify_range(imx585->exposure, IMX585_EXPOSURE_MIN,
				 imx585->vmax - IMX585_SHR_MIN_HDR, 1,
				 IMX585_EXPOSURE_DEFAULT);

	dev_info(imx585->clientdev, "Framing: VMAX=%u HMAX=%u pixel_rate=%llu\n",
		 imx585->vmax, imx585->hmax, pixel_rate);
}

/* --------------------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------------------
 */

static int imx585_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx585 *imx585 = container_of(ctrl->handler, struct imx585, ctrl_handler);
	const struct imx585_mode *mode;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 fmt_code;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&imx585->sd);
	mode = imx585_state_get_mode(imx585, state, &fmt_code);

	switch (ctrl->id) {
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		if (imx585->clear_hdr != ctrl->val) {
			imx585->clear_hdr = ctrl->val;

			imx585_activate_hdr_controls(imx585);

			/* Disable HCG in ClearHDR mode */
			imx585->hcg = imx585->clear_hdr ? 0 : imx585->hcg;
			__v4l2_ctrl_s_ctrl(imx585->hcg_ctrl, imx585->hcg);
			imx585_update_gain_limits(imx585);
			dev_info(imx585->clientdev, "HDR=%u, HCG=%u\n", ctrl->val, imx585->hcg);

			if (state) {
				struct v4l2_rect *crop;

				fmt = v4l2_subdev_state_get_format(state, 0);
				fmt_code = imx585_get_format_code(imx585, fmt->code);
				mode = imx585_find_mode(imx585, fmt_code,
							fmt->width, fmt->height);

				fmt->code = fmt_code;
				fmt->width = mode->width;
				fmt->height = mode->height;
				fmt->field = V4L2_FIELD_NONE;
				imx585_reset_colorspace(fmt);

				crop = v4l2_subdev_state_get_crop(state, 0);
				*crop = mode->crop;
			} else {
				mode = imx585_default_mode(imx585);
			}

			imx585_set_framing_limits(imx585, mode);
		}
		break;
	case V4L2_CID_IMX585_HCG_GAIN:
		if (!imx585->clear_hdr) {
			imx585->hcg = ctrl->val;
			imx585_update_gain_limits(imx585);
			dev_info(imx585->clientdev, "HCG=%u\n", ctrl->val);
		}
		break;
	default:
		break;
	}

	/* Apply control only when powered (runtime active). */
	if (!pm_runtime_get_if_active(imx585->clientdev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE: {
		u32 shr = (imx585->vmax - ctrl->val) & ~1U; /* SHR always a multiple of 2 */

		dev_dbg(imx585->clientdev, "EXPOSURE=%u -> SHR=%u (VMAX=%u HMAX=%u)\n",
			ctrl->val, shr, imx585->vmax, imx585->hmax);

		ret = cci_write(imx585->regmap, IMX585_REG_SHR, shr, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "SHR write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_IMX585_HCG_GAIN:
		if (!imx585->clear_hdr) {
			ret = cci_write(imx585->regmap, IMX585_REG_FDG_SEL0, ctrl->val, NULL);
			if (ret)
				dev_err_ratelimited(imx585->clientdev,
						    "FDG_SEL0 write failed (%d)\n", ret);
			dev_info(imx585->clientdev, "HCG write reg=%u\n", ctrl->val);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_info(imx585->clientdev, "ANALOG_GAIN=%u (%s)\n",
			ctrl->val, imx585->hcg ? "HCG" : "LCG");

		ret = cci_write(imx585->regmap, IMX585_REG_ANALOG_GAIN, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "Gain write failed (%d)\n", ret);
		break;
	case V4L2_CID_VBLANK: {
		u32 current_exposure = imx585->exposure->cur.val;
		const u32 min_shr = imx585->clear_hdr ? IMX585_SHR_MIN_HDR : IMX585_SHR_MIN;

		imx585->vmax = (mode->height + ctrl->val) & ~1U;

		current_exposure = clamp_t(u32, current_exposure,
					   IMX585_EXPOSURE_MIN, imx585->vmax - min_shr);
		__v4l2_ctrl_modify_range(imx585->exposure,
					 IMX585_EXPOSURE_MIN, imx585->vmax - min_shr, 1,
					 current_exposure);

		dev_info(imx585->clientdev, "VBLANK=%u -> VMAX=%u\n", ctrl->val, imx585->vmax);

		ret = cci_write(imx585->regmap, IMX585_REG_VMAX, imx585->vmax, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "VMAX write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_HBLANK: {
		u32 width   = mode->width;
		u32 hblank  = (u32)ctrl->val;
		u64 num;
		u32 hmax_new;

		num = (u64)mode->min_hmax * (width + hblank);
		hmax_new = div_u64(num, width);

		imx585->hmax = hmax_new;

		dev_info(imx585->clientdev,
			 "HBLANK=%u -> HMAX=%u (min_hmax=%u, width=%u)\n",
			 hblank, imx585->hmax, mode->min_hmax, width);

		ret = cci_write(imx585->regmap, IMX585_REG_HMAX, imx585->hmax, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "HMAX write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_HFLIP:
		ret = cci_write(imx585->regmap, IMX585_FLIP_WINMODEH, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "HFLIP write failed (%d)\n", ret);
		break;
	case V4L2_CID_VFLIP:
		ret = cci_write(imx585->regmap, IMX585_FLIP_WINMODEV, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "VFLIP write failed (%d)\n", ret);
		break;
	case V4L2_CID_BRIGHTNESS: {
		u16 blacklevel = min_t(u32, ctrl->val, 4095);

		ret = cci_write(imx585->regmap, IMX585_REG_BLKLEVEL, blacklevel, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "BLKLEVEL write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_IMX585_SHR:
		dev_info(imx585->clientdev, "SHR=%u\n", ctrl->val);
		if (ctrl->val == 0)
            break; 
		ret = cci_write(imx585->regmap, IMX585_REG_SHR, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "SHR write failed (%d)\n", ret);
		break;
	case V4L2_CID_IMX585_VMAX:
		dev_info(imx585->clientdev, "VMAX=%u\n", ctrl->val);
		if (ctrl->val == 0)
            break; 
		ret = cci_write(imx585->regmap, IMX585_REG_VMAX, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "VMAX write failed (%d)\n", ret);
		break;
	case V4L2_CID_IMX585_HMAX:
		dev_info(imx585->clientdev, "HMAX=%u\n", ctrl->val);
		if (ctrl->val == 0)
            break; 
		ret = cci_write(imx585->regmap, IMX585_REG_HMAX, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "HMAX write failed (%d)\n", ret);
		break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE: /* Handled above */
		break;
	case V4L2_CID_IMX585_HDR_DATASEL_TH: {
		const u16 *th = (const u16 *)ctrl->p_new.p;

		ret = cci_write(imx585->regmap, IMX585_REG_EXP_TH_H, th[0], NULL);
		if (!ret)
			ret = cci_write(imx585->regmap, IMX585_REG_EXP_TH_L, th[1], NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "HDR TH write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_IMX585_HDR_DATASEL_BK:
		ret = cci_write(imx585->regmap, IMX585_REG_EXP_BK, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev, "HDR BK write failed (%d)\n", ret);
		break;
	case V4L2_CID_IMX585_HDR_GRAD_TH: {
		const u32 *thr = (const u32 *)ctrl->p_new.p;

		ret = cci_write(imx585->regmap, IMX585_REG_CCMP1_EXP, thr[0], NULL);
		if (!ret)
			ret = cci_write(imx585->regmap, IMX585_REG_CCMP2_EXP, thr[1], NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "HDR grad TH write failed (%d)\n", ret);
		break;
	}
	case V4L2_CID_IMX585_HDR_GRAD_COMP_L:
		ret = cci_write(imx585->regmap, IMX585_REG_ACMP1_EXP, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "HDR grad low write failed (%d)\n", ret);
		break;
	case V4L2_CID_IMX585_HDR_GRAD_COMP_H:
		ret = cci_write(imx585->regmap, IMX585_REG_ACMP2_EXP, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "HDR grad high write failed (%d)\n", ret);
		break;
	case V4L2_CID_IMX585_HDR_GAIN:
		ret = cci_write(imx585->regmap, IMX585_REG_EXP_GAIN, ctrl->val, NULL);
		if (ret)
			dev_err_ratelimited(imx585->clientdev,
					    "HDR gain write failed (%d)\n", ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx585_update_test_pattern(imx585, ctrl->val);
		break;
	default:
		dev_dbg(imx585->clientdev, "Unhandled ctrl %s: id=0x%x, val=0x%x\n",
			 ctrl->name, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(imx585->clientdev);
	return ret;
}

static const struct v4l2_ctrl_ops imx585_ctrl_ops = {
	.s_ctrl = imx585_set_ctrl,
};

/*
 * ClearHDR threshold register order (per IMX585 AppNote, §4.2):
 *   th[0] -> EXP_TH_H (0x36D0): high-gain saturation cutoff
 *   th[1] -> EXP_TH_L (0x36D4): high-gain "low" cutoff
 * Constraint: EXP_TH_H >= EXP_TH_L (the spec marks EXP_TH_H < EXP_TH_L
 * as "Prohibited" — the sensor enters an invalid state and only outputs
 * the BLC pedestal).
 *
 * The AppNote's "initial value" of 0x1000 each (= thresholds equal,
 * EXP_BK weighted blend) clamps the combiner output near BLC on
 * typical scenes — empirically verified on this rig. Default to a wide
 * rule-based selection range so HG drives normal exposure values and
 * LG only kicks in once HG saturates near 0x0FFF.
 */
static const u16 hdr_thresh_def[2] = { 0x0FFF, 0x0000 };
static const struct v4l2_ctrl_config imx585_cfg_datasel_th = {
	.ops       = &imx585_ctrl_ops,
	.id        = V4L2_CID_IMX585_HDR_DATASEL_TH,
	.name      = "HDR Data Selection Threshold",
	.type      = V4L2_CTRL_TYPE_U16,
	.min       = 0,
	.max       = 0x0FFF,
	.step      = 1,
	.def       = 0,
	.dims      = { 2 },
	.elem_size = sizeof(u16),
};

static const struct v4l2_ctrl_config imx585_cfg_datasel_bk = {
	.ops   = &imx585_ctrl_ops,
	.id    = V4L2_CID_IMX585_HDR_DATASEL_BK,
	.name  = "HDR Data Blending Mode",
	.type  = V4L2_CTRL_TYPE_MENU,
	.max   = ARRAY_SIZE(hdr_data_blender_menu) - 1,
	.def   = 0,
	.qmenu = hdr_data_blender_menu,
};

static const u32 grad_thresh_def[2] = { 500, 11500 };
static const struct v4l2_ctrl_config imx585_cfg_grad_th = {
	.ops       = &imx585_ctrl_ops,
	.id        = V4L2_CID_IMX585_HDR_GRAD_TH,
	.name      = "HDR Gradient Compression Threshold (16-bit)",
	.type      = V4L2_CTRL_TYPE_U32,
	.min       = 0,
	.max       = 0x1FFFF,
	.step      = 1,
	.def       = 0,
	.dims      = { 2 },
	.elem_size = sizeof(u32),
};

/*
 * Per IMX585 AppNote §4.3 / Rev1.0 page 16:
 *
 *   ACMP1_EXP @ 0x36EE controls the MIDDLE compression segment (between
 *   CCMP1_EXP and CCMP2_EXP). Allowed values: 06h..0Bh (1/64..1/2048).
 *   ACMP2_EXP @ 0x36EC controls the HIGH segment (above CCMP2_EXP).
 *   Allowed values: 00h..05h (1/1..1/32).
 *
 * Writing a value outside the allowed range puts the sensor into a degenerate
 * state and the output ends up clamped at BLC. The original driver defaults
 * had these the wrong way round (idx 2 = "1/4" written to ACMP1 — prohibited),
 * which produced all-BLC frames in 12-bit ClearHDR mode.
 *
 * GRAD_COMP_L writes ACMP1_EXP (middle slope, aggressive ratios).
 * GRAD_COMP_H writes ACMP2_EXP (high slope, mild ratios).
 */
static const struct v4l2_ctrl_config imx585_cfg_grad_exp_l = {
	.ops   = &imx585_ctrl_ops,
	.id    = V4L2_CID_IMX585_HDR_GRAD_COMP_L,
	.name  = "HDR Gradient Compression Ratio Middle (ACMP1)",
	.type  = V4L2_CTRL_TYPE_MENU,
	.min   = 6,                                          /* spec lower bound for ACMP1 */
	.max   = ARRAY_SIZE(grad_compression_slope_menu) - 1,
	.def   = 6,                                          /* 1/64 */
	.qmenu = grad_compression_slope_menu,
};

static const struct v4l2_ctrl_config imx585_cfg_grad_exp_h = {
	.ops   = &imx585_ctrl_ops,
	.id    = V4L2_CID_IMX585_HDR_GRAD_COMP_H,
	.name  = "HDR Gradient Compression Ratio High (ACMP2)",
	.type  = V4L2_CTRL_TYPE_MENU,
	.min   = 0,
	.max   = 5,                                          /* spec upper bound for ACMP2 */
	.def   = 4,                                          /* 1/16 — natural inverse spans 16-bit cleanly */
	.qmenu = grad_compression_slope_menu,
};

static const struct v4l2_ctrl_config imx585_cfg_hdr_gain = {
	.ops   = &imx585_ctrl_ops,
	.id    = V4L2_CID_IMX585_HDR_GAIN,
	.name  = "HDR Gain Adder (dB)",
	.type  = V4L2_CTRL_TYPE_MENU,
	.min   = 0,
	.max   = ARRAY_SIZE(hdr_gain_adder_menu) - 1,
	.def   = 2,
	.qmenu = hdr_gain_adder_menu,
};

static const struct v4l2_ctrl_config imx585_cfg_hcg = {
	.ops  = &imx585_ctrl_ops,
	.id   = V4L2_CID_IMX585_HCG_GAIN,
	.name = "HCG Enable",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = 0,
	.max  = 1,
	.step = 1,
	.def  = 0,
};

static const struct v4l2_ctrl_config imx585_cfg_hmax = {
	.ops  = &imx585_ctrl_ops,
	.id   = V4L2_CID_IMX585_HMAX,
	.name = "HMAX",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = IMX585_HMAX_MAX,
	.step = 1,
};

static const struct v4l2_ctrl_config imx585_cfg_vmax = {
	.ops  = &imx585_ctrl_ops,
	.id   = V4L2_CID_IMX585_VMAX,
	.name = "VMAX",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = IMX585_VMAX_MAX,
	.step = 1,
};

static const struct v4l2_ctrl_config imx585_cfg_shr = {
	.ops  = &imx585_ctrl_ops,
	.id   = V4L2_CID_IMX585_SHR,
	.name = "SHR",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = IMX585_SHR_MAX,
	.step = 1,
};

static int imx585_init_controls(struct imx585 *imx585)
{
	struct v4l2_ctrl_handler *hdl = &imx585->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 32);

	/* Read-only, updated per mode */
	imx585->pixel_rate = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       1, UINT_MAX, 1, 1);

	imx585->link_freq =
		v4l2_ctrl_new_int_menu(hdl, &imx585_ctrl_ops, V4L2_CID_LINK_FREQ,
				       0, 0, &link_freqs[imx585->link_freq_idx]);
	if (imx585->link_freq)
		imx585->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx585->vblank = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xFFFFF, 1, 0);
	imx585->hblank = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xFFFF, 1, 0);
	imx585->blacklevel = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					       V4L2_CID_BRIGHTNESS, 0, 0xFFFF, 1,
					       IMX585_BLKLEVEL_DEFAULT);

	imx585->exposure = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX585_EXPOSURE_MIN, IMX585_EXPOSURE_MAX,
					     IMX585_EXPOSURE_STEP, IMX585_EXPOSURE_DEFAULT);

	imx585->gain = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
					 IMX585_ANA_GAIN_MIN_NORMAL, IMX585_ANA_GAIN_MAX_NORMAL,
					 IMX585_ANA_GAIN_STEP, IMX585_ANA_GAIN_DEFAULT);

	imx585->hflip = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx585->vflip = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	imx585->hdr_mode = v4l2_ctrl_new_std(hdl, &imx585_ctrl_ops,
					     V4L2_CID_WIDE_DYNAMIC_RANGE, 0, 1, 1, 0);
	imx585->datasel_th_ctrl = v4l2_ctrl_new_custom(hdl, &imx585_cfg_datasel_th, NULL);
	imx585->datasel_bk_ctrl = v4l2_ctrl_new_custom(hdl, &imx585_cfg_datasel_bk, NULL);
	imx585->gdc_th_ctrl     = v4l2_ctrl_new_custom(hdl, &imx585_cfg_grad_th, NULL);
	imx585->gdc_exp_ctrl_l  = v4l2_ctrl_new_custom(hdl, &imx585_cfg_grad_exp_l, NULL);
	imx585->gdc_exp_ctrl_h  = v4l2_ctrl_new_custom(hdl, &imx585_cfg_grad_exp_h, NULL);
	imx585->hdr_gain_ctrl   = v4l2_ctrl_new_custom(hdl, &imx585_cfg_hdr_gain, NULL);
	imx585->hcg_ctrl        = v4l2_ctrl_new_custom(hdl, &imx585_cfg_hcg, NULL);

	imx585->vmax_ctrl        = v4l2_ctrl_new_custom(hdl, &imx585_cfg_vmax, NULL);
	imx585->hmax_ctrl        = v4l2_ctrl_new_custom(hdl, &imx585_cfg_hmax, NULL);
	imx585->shr_ctrl        = v4l2_ctrl_new_custom(hdl, &imx585_cfg_shr, NULL);

	v4l2_ctrl_new_std_menu_items(hdl, &imx585_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx585_tpg_menu) - 1,
				     0, 0, imx585_tpg_menu);

	imx585_activate_hdr_controls(imx585);

	if (hdl->error) {
		ret = hdl->error;
		dev_err(imx585->clientdev, "control init failed (%d)\n", ret);
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(imx585->clientdev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &imx585_ctrl_ops, &props);
	if (ret)
		goto err_free;

	/* Set the default value for ClearHDR thresholds */
	memcpy(imx585->datasel_th_ctrl->p_cur.p, hdr_thresh_def, sizeof(hdr_thresh_def));
	memcpy(imx585->datasel_th_ctrl->p_new.p, hdr_thresh_def, sizeof(hdr_thresh_def));
	memcpy(imx585->gdc_th_ctrl->p_cur.p, grad_thresh_def, sizeof(grad_thresh_def));
	memcpy(imx585->gdc_th_ctrl->p_new.p, grad_thresh_def, sizeof(grad_thresh_def));

	imx585->hdr_mode->flags |= V4L2_CTRL_FLAG_UPDATE | V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx585->sd.ctrl_handler = hdl;
	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void imx585_free_controls(struct imx585 *imx585)
{
	v4l2_ctrl_handler_free(imx585->sd.ctrl_handler);
}

/* --------------------------------------------------------------------------
 * Pad ops / formats
 * --------------------------------------------------------------------------
 */

static int imx585_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx585 *imx585 = to_imx585(sd);
	unsigned int entries;
	const u32 *tbl;

	if (code->pad)
		return -EINVAL;

	if (imx585->mono) {
		if (imx585->clear_hdr) {
			if (code->index >= (imx585->clearhdr_ccmp ? 2 : 1))
				return -EINVAL;
			code->code = mono_codes[code->index];
			return 0;
		}
		if (code->index > 1)
			return -EINVAL;
		code->code = code->index ? MEDIA_BUS_FMT_Y10_1X10
					  : MEDIA_BUS_FMT_Y12_1X12;
		return 0;
	}

	if (imx585->clear_hdr) {
		tbl = codes_clearhdr;
		entries = imx585->clearhdr_ccmp ?
			  ARRAY_SIZE(codes_clearhdr) / 4 : 1;
	} else {
		tbl = codes_normal;
		entries = ARRAY_SIZE(codes_normal) / 4;
	}

	if (code->index >= entries)
		return -EINVAL;

	code->code = imx585_get_format_code(imx585, tbl[code->index * 4]);
	return 0;
}

static int imx585_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx585 *imx585 = to_imx585(sd);
	const struct imx585_mode *mode_list;
	unsigned int num_modes;

	if (fse->pad)
		return -EINVAL;

	get_mode_table(imx585, fse->code, &mode_list, &num_modes);
	if (fse->index >= num_modes)
		return -EINVAL;
	if (fse->code != imx585_get_format_code(imx585, fse->code))
		return -EINVAL;

	fse->min_width  = mode_list[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->min_height = mode_list[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx585_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx585 *imx585 = to_imx585(sd);
	const struct imx585_mode *mode;
	struct v4l2_mbus_framefmt *format;

	if (fmt->pad)
		return -EINVAL;

	fmt->format.code = imx585_get_format_code(imx585, fmt->format.code);
	mode = imx585_find_mode(imx585, fmt->format.code,
				fmt->format.width, fmt->format.height);

	fmt->format.width        = mode->width;
	fmt->format.height       = mode->height;
	fmt->format.field        = V4L2_FIELD_NONE;
	imx585_reset_colorspace(&fmt->format);

	format = v4l2_subdev_state_get_format(sd_state, fmt->pad);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		imx585_set_framing_limits(imx585, mode);

	*format = fmt->format;

	/*
	 * Sync the per-mode crop into the subdev state so libcamera reads
	 * the right active area for ScalerCrop bounds when the mode changes
	 * (otherwise the crop stays at whatever init_state set, which is
	 * mode 0). Per-mode crop matters because the OB offsets at the top
	 * of the visible buffer differ between binning (10 rows) and 4K
	 * all-pixel (20 rows) — see IMX585_PIXEL_ARRAY_TOP_BIN/4K.
	 */
	*v4l2_subdev_state_get_crop(sd_state, 0) = mode->crop;
	return 0;
}

/* --------------------------------------------------------------------------
 * Stream on/off
 * --------------------------------------------------------------------------
 */

static int imx585_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx585 *imx585 = to_imx585(sd);
	const struct imx585_mode *mode;
	u32 fmt_code;
	int ret;

	if (pad || streams_mask != BIT_ULL(0))
		return -EINVAL;

	/* Single source-pad stream: repeated enable has nothing to reapply. */
	if (v4l2_subdev_is_streaming(sd))
		return 0;

	ret = pm_runtime_get_sync(imx585->clientdev);
	if (ret < 0) {
		pm_runtime_put_noidle(imx585->clientdev);
		return ret;
	}

	ret = cci_multi_reg_write(imx585->regmap, common_regs,
				  ARRAY_SIZE(common_regs), NULL);
	if (ret) {
		dev_err(imx585->clientdev, "Failed to write common settings\n");
		goto err_rpm_put;
	}

	ret = cci_write(imx585->regmap, IMX585_INCK_SEL, imx585->inck_sel_val, NULL);
	if (!ret)
		ret = cci_write(imx585->regmap, IMX585_REG_BLKLEVEL, IMX585_BLKLEVEL_DEFAULT, NULL);
	if (!ret)
		ret = cci_write(imx585->regmap, IMX585_DATARATE_SEL,
				link_freqs_reg_value[imx585->link_freq_idx], NULL);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(imx585->regmap, IMX585_LANEMODE,
			(imx585->lane_count == 2) ? 0x01 : 0x03, NULL);
	if (ret)
		goto err_rpm_put;

	/* Mono bin flag (datasheet: 0x01 mono, 0x00 color) */
	ret = cci_write(imx585->regmap, IMX585_BIN_MODE, imx585->mono ? 0x01 : 0x00, NULL);
	if (ret)
		goto err_rpm_put;

	/* Sync configuration */
	if (imx585->sync_mode == SYNC_INT_FOLLOWER) {
		dev_info(imx585->clientdev, "Internal sync follower: XVS input\n");
		ret = cci_write(imx585->regmap, IMX585_REG_EXTMODE, 0x01, NULL);
		if (!ret)
			ret = cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x03, NULL); /* XHS out, XVS in */
		if (!ret)
			ret = cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x08, NULL); /* disable XVS OUT */
	} else if (imx585->sync_mode == SYNC_INT_LEADER) {
		dev_info(imx585->clientdev, "Internal sync leader: XVS/XHS output\n");
		ret = cci_write(imx585->regmap, IMX585_REG_EXTMODE, 0x00, NULL);
		if (!ret)
			ret = cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x00, NULL); /* XHS/XVS out */
		if (!ret)
			ret = cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x0A, NULL);
	} else {
		dev_info(imx585->clientdev, "Follower: XVS/XHS input\n");
		ret = cci_write(imx585->regmap, IMX585_REG_XXS_DRV, 0x0F, NULL); /* inputs */
		if (!ret)
			ret = cci_write(imx585->regmap, IMX585_REG_XXS_OUTSEL, 0x00, NULL);
	}
	if (ret)
		goto err_rpm_put;

	imx585->common_regs_written = true;

	/* Select mode */
	mode = imx585_state_get_mode(imx585, state, &fmt_code);

	ret = cci_multi_reg_write(imx585->regmap, mode->reg_list.regs,
				  mode->reg_list.num_of_regs, NULL);
	if (ret) {
		dev_err(imx585->clientdev, "Failed to write mode registers\n");
		goto err_rpm_put;
	}

	if (imx585->clear_hdr) {
		ret = cci_multi_reg_write(imx585->regmap, common_clearHDR_mode,
					  ARRAY_SIZE(common_clearHDR_mode), NULL);
		if (ret) {
			dev_err(imx585->clientdev, "Failed to set ClearHDR regs\n");
			goto err_rpm_put;
		}

		/*
		 * Known issue: ClearHDR mode leaves ~19 OB rows at the top of
		 * the cropped buffer, regardless of PIX_VST value. WINMODE
		 * crop works cleanly for SDR (row 0 = scene) but in HDR the
		 * sensor appears to prepend an HDR-specific OB region that
		 * isn't bypassed by the cropping window. Increasing PIX_VST
		 * shifts the BOTTOM of the buffer (recording window slides
		 * down) but not the top OB count. Needs further datasheet
		 * investigation or empirical pattern testing — for now the
		 * SDR fix is in place and HDR keeps the residual top-bar.
		 */
		/* 16-bit: linear; 12-bit: opt-in gradation compression. */
		if (imx585_is_clearhdr_16bit_code(fmt_code)) {
			ret = cci_write(imx585->regmap, IMX585_REG_CCMP_EN, 0x00, NULL);
			if (!ret)
				ret = cci_write(imx585->regmap, IMX585_REG_MDBIT, 0x03, NULL);
		} else if (imx585->clearhdr_ccmp &&
			   imx585_is_clearhdr_12bit_code(fmt_code)) {
			ret = cci_write(imx585->regmap, IMX585_REG_CCMP_EN, 0x01, NULL);
			if (!ret)
				ret = cci_write(imx585->regmap, IMX585_REG_MDBIT, 0x01, NULL);
		} else {
			dev_err(imx585->clientdev,
				"12-bit ClearHDR CCMP requested without ccmp overlay flag\n");
			ret = -EINVAL;
		}
		if (ret)
			goto err_rpm_put;
	} else {
		ret = cci_multi_reg_write(imx585->regmap, common_normal_mode,
					  ARRAY_SIZE(common_normal_mode), NULL);
		if (ret) {
			dev_err(imx585->clientdev, "Failed to set normal regs\n");
			goto err_rpm_put;
		}
	}

	/* Disable digital clamp */
	ret = cci_write(imx585->regmap, IMX585_REG_DIGITAL_CLAMP, 0x00, NULL);
	if (ret)
		goto err_rpm_put;

	/* Reset manual HMAX/VMAX/SHR control value */
	__v4l2_ctrl_s_ctrl(imx585->vmax_ctrl, 0);
	__v4l2_ctrl_s_ctrl(imx585->hmax_ctrl, 0);
	__v4l2_ctrl_s_ctrl(imx585->shr_ctrl, 0);

	/* Apply user controls after writing the base tables */
	ret = __v4l2_ctrl_handler_setup(imx585->sd.ctrl_handler);
	if (ret) {
		dev_err(imx585->clientdev, "Control handler setup failed\n");
		goto err_rpm_put;
	}

	if (imx585->sync_mode != SYNC_EXTERNAL) {
		ret = cci_write(imx585->regmap, IMX585_REG_XMSTA, 0x00, NULL);
		if (ret)
			goto err_rpm_put;
	}

	ret = cci_write(imx585->regmap, IMX585_REG_MODE_SELECT, IMX585_MODE_STREAMING, NULL);
	if (ret)
		goto err_rpm_put;

	dev_info(imx585->clientdev, "Streaming started\n");
	usleep_range(IMX585_STREAM_DELAY_US,
		     IMX585_STREAM_DELAY_US + IMX585_STREAM_DELAY_RANGE_US);

	/* vflip, hflip and HDR cannot change during streaming */
	__v4l2_ctrl_grab(imx585->vflip, true);
	__v4l2_ctrl_grab(imx585->hflip, true);
	__v4l2_ctrl_grab(imx585->hdr_mode, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(imx585->clientdev);
	return ret;
}

static int imx585_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	if (pad || streams_mask != BIT_ULL(0))
		return -EINVAL;

	if (sd->enabled_pads & ~BIT_ULL(pad))
		return 0;

	ret = cci_write(imx585->regmap, IMX585_REG_MODE_SELECT, IMX585_MODE_STANDBY, NULL);
	if (ret)
		dev_err(imx585->clientdev, "Failed to stop streaming\n");

	__v4l2_ctrl_grab(imx585->vflip, false);
	__v4l2_ctrl_grab(imx585->hflip, false);
	__v4l2_ctrl_grab(imx585->hdr_mode, false);

	pm_runtime_put_autosuspend(imx585->clientdev);

	return ret;
}

/* --------------------------------------------------------------------------
 * Power / runtime PM
 * --------------------------------------------------------------------------
 */

static int imx585_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);
	int ret;

	dev_info(imx585->clientdev, "power_on\n");

	ret = regulator_bulk_enable(IMX585_NUM_SUPPLIES, imx585->supplies);
	if (ret) {
		dev_err(imx585->clientdev, "Failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(imx585->xclk);
	if (ret) {
		dev_err(imx585->clientdev, "Failed to enable clock\n");
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx585->reset_gpio, 1);
	usleep_range(IMX585_XCLR_MIN_DELAY_US,
		     IMX585_XCLR_MIN_DELAY_US + IMX585_XCLR_DELAY_RANGE_US);
	return 0;

reg_off:
	regulator_bulk_disable(IMX585_NUM_SUPPLIES, imx585->supplies);
	return ret;
}

static int imx585_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx585 *imx585 = to_imx585(sd);

	dev_info(imx585->clientdev, "power_off\n");

	gpiod_set_value_cansleep(imx585->reset_gpio, 0);
	regulator_bulk_disable(IMX585_NUM_SUPPLIES, imx585->supplies);
	clk_disable_unprepare(imx585->xclk);

	return 0;
}

/* --------------------------------------------------------------------------
 * Selection / state
 * --------------------------------------------------------------------------
 */

static int imx585_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->pad)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX585_NATIVE_WIDTH;
		sel->r.height = IMX585_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		/*
		 * Active recording area = buffer dimensions, since the sensor
		 * is configured (via WINMODE crop, see IMX585_WIN_CROP_REGS_*)
		 * to skip OB rows/cols at readout. Buffer holds active pixels
		 * only.
		 */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX585_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX585_PIXEL_ARRAY_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx585_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_rect *crop;
	struct imx585 *imx585 = to_imx585(sd);
	struct v4l2_subdev_format fmt = {
		.which  = V4L2_SUBDEV_FORMAT_TRY,
		.pad    = 0,
		.format = {
			.code   = imx585->mono ? MEDIA_BUS_FMT_Y12_1X12
					    : MEDIA_BUS_FMT_SRGGB12_1X12,
			.width  = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	imx585_set_pad_format(sd, state, &fmt);

	crop = v4l2_subdev_state_get_crop(state, 0);
	*crop = supported_modes[0].crop;

	return 0;
}

/* --------------------------------------------------------------------------
 * Subdev ops
 * --------------------------------------------------------------------------
 */

static const struct v4l2_subdev_video_ops imx585_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx585_pad_ops = {
	.enum_mbus_code = imx585_enum_mbus_code,
	.get_fmt        = v4l2_subdev_get_fmt,
	.set_fmt        = imx585_set_pad_format,
	.get_selection  = imx585_get_selection,
	.enum_frame_size = imx585_enum_frame_size,
	.enable_streams  = imx585_enable_streams,
	.disable_streams = imx585_disable_streams,
};

static const struct v4l2_subdev_internal_ops imx585_internal_ops = {
	.init_state = imx585_init_state,
};

static const struct v4l2_subdev_ops imx585_subdev_ops = {
	.video = &imx585_video_ops,
	.pad   = &imx585_pad_ops,
};

/* --------------------------------------------------------------------------
 * Probe / remove
 * --------------------------------------------------------------------------
 */

static int imx585_check_hwcfg(struct device *dev, struct imx585 *imx585)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret = -EINVAL;
	int i;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep)) {
		dev_err(dev, "could not parse endpoint\n");
		goto out_put;
	}

	if (ep.bus.mipi_csi2.num_data_lanes != 2 &&
	    ep.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "only 2 or 4 data lanes supported\n");
		goto out_free;
	}
	imx585->lane_count = ep.bus.mipi_csi2.num_data_lanes;
	dev_info(dev, "Data lanes: %u\n", imx585->lane_count);

	if (!ep.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property missing\n");
		goto out_free;
	}

	for (i = 0; i < ARRAY_SIZE(link_freqs); i++) {
		if (link_freqs[i] == ep.link_frequencies[0]) {
			imx585->link_freq_idx = i;
			break;
		}
	}
	if (i == ARRAY_SIZE(link_freqs)) {
		dev_err(dev, "unsupported link frequency: %llu\n",
			(unsigned long long)ep.link_frequencies[0]);
		goto out_free;
	}

	dev_info(dev, "Link speed: %llu Hz\n",
		 (unsigned long long)ep.link_frequencies[0]);

	ret = 0;

out_free:
	v4l2_fwnode_endpoint_free(&ep);
out_put:
	fwnode_handle_put(endpoint);
	return ret;
}

static int imx585_get_regulators(struct imx585 *imx585)
{
	unsigned int i;

	for (i = 0; i < IMX585_NUM_SUPPLIES; i++)
		imx585->supplies[i].supply = imx585_supply_name[i];

	return devm_regulator_bulk_get(imx585->clientdev,
				       IMX585_NUM_SUPPLIES, imx585->supplies);
}

static int imx585_check_module_exists(struct imx585 *imx585)
{
	int ret;
	u64 val;

	/* No chip-id register; read a known register as a presence test */
	ret = cci_read(imx585->regmap, IMX585_REG_BLKLEVEL, &val, NULL);
	if (ret) {
		dev_err(imx585->clientdev, "register read failed (%d)\n", ret);
		return ret;
	}

	dev_dbg(imx585->clientdev, "Sensor detected\n");
	return 0;
}

static int imx585_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx585 *imx585;
	const char *sync_mode;
	int ret, i;

	imx585 = devm_kzalloc(dev, sizeof(*imx585), GFP_KERNEL);
	if (!imx585)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx585->sd, client, &imx585_subdev_ops);
	imx585->clientdev = dev;

	dev_info(dev, "Reading dtoverlay config:\n");
	imx585->mono = of_property_read_bool(dev->of_node, "mono-mode");
	if (imx585->mono)
		dev_info(dev, "Mono Mode Selected, make sure you have the correct sensor variant\n");
	imx585->clearhdr_ccmp = of_property_read_bool(dev->of_node,
						      "sony,clearhdr-ccmp");
	dev_info(dev, "ClearHDR 12-bit CCMP: %s\n",
		 imx585->clearhdr_ccmp ? "enabled" : "disabled");

	imx585->sync_mode = SYNC_INT_LEADER;
	if (!device_property_read_string(dev, "sony,sync-mode", &sync_mode)) {
		if (!strcmp(sync_mode, "internal-follower"))
			imx585->sync_mode = SYNC_INT_FOLLOWER;
		else if (!strcmp(sync_mode, "external"))
			imx585->sync_mode = SYNC_EXTERNAL;
	}
	dev_info(dev, "sync-mode: %s\n", sync_mode_menu[imx585->sync_mode]);

	ret = imx585_check_hwcfg(dev, imx585);
	if (ret)
		return ret;

	imx585->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx585->regmap))
		return dev_err_probe(dev, PTR_ERR(imx585->regmap), "CCI init failed\n");

	imx585->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx585->xclk))
		return dev_err_probe(dev, PTR_ERR(imx585->xclk), "xclk missing\n");

	imx585->xclk_freq = clk_get_rate(imx585->xclk);
	for (i = 0; i < ARRAY_SIZE(imx585_inck_table); ++i) {
		if (imx585_inck_table[i].xclk_hz == imx585->xclk_freq) {
			imx585->inck_sel_val = imx585_inck_table[i].inck_sel;
			break;
		}
	}
	if (i == ARRAY_SIZE(imx585_inck_table))
		return dev_err_probe(dev, -EINVAL, "unsupported XCLK %u Hz\n", imx585->xclk_freq);

	dev_info(dev, "XCLK %u Hz -> INCK_SEL 0x%02x\n",
		 imx585->xclk_freq, imx585->inck_sel_val);

	ret = imx585_get_regulators(imx585);
	if (ret)
		return dev_err_probe(dev, ret, "regulators\n");

	imx585->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

	/* Power on to probe the device */
	ret = imx585_power_on(dev);
	if (ret)
		return ret;

	ret = imx585_check_module_exists(imx585);
	if (ret)
		goto err_power_off;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = imx585_init_controls(imx585);
	if (ret)
		goto err_pm;

	imx585->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx585->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx585->sd.internal_ops = &imx585_internal_ops;

	imx585->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx585->sd.entity, 1, &imx585->pad);
	if (ret) {
		dev_err(dev, "entity pads init failed: %d\n", ret);
		goto err_ctrls;
	}

	imx585->sd.state_lock = imx585->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx585->sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init\n");
		goto err_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&imx585->sd);
	if (ret) {
		dev_err(dev, "sensor subdev register failed: %d\n", ret);
		goto err_entity;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return 0;

err_entity:
	media_entity_cleanup(&imx585->sd.entity);
err_ctrls:
	imx585_free_controls(imx585);
err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_power_off:
	imx585_power_off(dev);
	return ret;
}

static void imx585_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx585 *imx585 = to_imx585(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	imx585_free_controls(imx585);

	pm_runtime_disable(imx585->clientdev);
	if (!pm_runtime_status_suspended(imx585->clientdev))
		imx585_power_off(imx585->clientdev);
	pm_runtime_set_suspended(imx585->clientdev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx585_pm_ops, imx585_power_off,
				 imx585_power_on, NULL);

static const struct of_device_id imx585_of_match[] = {
	{ .compatible = "sony,imx585" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx585_of_match);

static struct i2c_driver imx585_i2c_driver = {
	.driver = {
		.name  = "imx585",
		.pm    = pm_ptr(&imx585_pm_ops),
		.of_match_table = imx585_of_match,
	},
	.probe  = imx585_probe,
	.remove = imx585_remove,
};
module_i2c_driver(imx585_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_AUTHOR("Tetsuya Nomura <tetsuya.nomura@soho-enterprise.com>");
MODULE_DESCRIPTION("Sony IMX585 sensor driver");
MODULE_LICENSE("GPL");
