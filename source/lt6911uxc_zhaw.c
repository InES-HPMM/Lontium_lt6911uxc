/*
 * lt6911uxc.c - Lontium 4k60 HDMI-CSI bridge driver
 *
 * Copyright (c) 2020, Lukas Neuner <neur@zhaw.ch>
 *
 * This program is based on the tc358840 - Toshiba HDMI to CSI-2 bridge from
 * Armin Weiss
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUG 1
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/version.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/interrupt.h>

#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#ifdef CONFIG_V4L2_FWNODE
#include <media/v4l2-fwnode.h>
#else
#include <media/v4l2-of.h>
#endif
#include "lt6911uxc_regs_zhaw.h"

/* v4l2 debug level */
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

/* custom v4l2 controls */
#define V4L2_CID_USER_LT6911UXC_BASE (V4L2_CID_USER_BASE + 0x1090)
#define LT6911UXC_CID_AUDIO_SAMPLING_RATE (V4L2_CID_USER_LT6911UXC_BASE + 1)
#define LT6911UXC_CID_AUDIO_PRESENT       (V4L2_CID_USER_LT6911UXC_BASE + 2)
#define LT6911UXC_CID_NUM_MIPI_LANES      (V4L2_CID_USER_LT6911UXC_BASE + 3)
#define LT6911UXC_NUM_CTRLS 3

#define LT6911UXC_DEFAULT_MODE	0
#define LT6911UXC_DEFAULT_DATAFMT	MEDIA_BUS_FMT_UYVY8_1X16

/* v4l2 dv timings */
static struct v4l2_dv_timings default_timing = V4L2_DV_BT_CEA_1920X1080P60;

static const struct v4l2_dv_timings_cap lt6911uxc_timings_cap_4kp30 = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(
		160, 3840,				/* min/max width */
		120, 2160,				/* min/max height */
		25000000, 300000000,			/* min/max pixelclock */
		V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
		V4L2_DV_BT_STD_CVT,
		V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_CUSTOM |
		V4L2_DV_BT_CAP_REDUCED_BLANKING)
};


static const int lt6911uxc_60fps[] = {
	60,
};

static const int lt6911uxc_30fps[] = {
	30,
};


static const struct camera_common_frmfmt lt6911uxc_frmfmt[] = {

	{{1920, 1080}, lt6911uxc_60fps, 1, 0, 0},
	{{3840, 2160}, lt6911uxc_30fps, 1, 0, 1},
	{{1920, 2160}, lt6911uxc_60fps, 1, 0, 2},
};

struct lt6911uxc {
	struct v4l2_ctrl_handler ctrl_handler;
	struct i2c_client *i2c_client;
	struct v4l2_subdev *sd;
	struct media_pad pad[1];

	struct camera_common_data *s_data;
	struct camera_common_pdata *pdata;

	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;
	struct v4l2_ctrl *num_mipi_lanes_ctrl;

	struct v4l2_ctrl		*ctrls[LT6911UXC_NUM_CTRLS]; // [0]: same as audio_sampling_rate_ctrl, [1]: same as audio_present_ctrl , [2]: same as num_mipi_lanes_ctrl
	/* timing / media format */
	struct v4l2_dv_timings timings;
	struct v4l2_dv_timings detected_timings;/* timings detected from phy */

	struct mutex lock;

	/* controls */
	u8 bank;		/* active reg-bank for I2C */
	bool enable_i2c;
	bool signal_present;
	/* expose audio capabilties */
	u32 mbus_fmt_code;			/* current media bus format */

	//struct lt6911uxc_platform_data *pdata;
};

static const struct camera_common_colorfmt lt6911uxc_color_fmts[] = {
	{ MEDIA_BUS_FMT_UYVY8_1X16,  V4L2_COLORSPACE_SRGB,  V4L2_PIX_FMT_UYVY},
};

static const struct v4l2_event lt6911uxc_ev_source_change = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static inline struct lt6911uxc *to_state(struct v4l2_subdev *sd)
{
	struct camera_common_data *tmp = container_of(sd, struct camera_common_data, subdev);
	return (struct lt6911uxc *)(tmp->priv);
}

/* ------ I2C --------------------------------------------------------------- */

static void lt6911uxc_reg_bank(struct v4l2_subdev *sd, u8 bank)
{
	struct lt6911uxc *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	struct i2c_msg msg;
	u8 data[2];
	u8 address;

	if(state->bank == bank)
		return;
	dev_dbg(&client->dev, "i2c: change register bank to 0x%02X\n",
		bank);

	address = 0xFF;
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = address;
	data[1] = bank;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		dev_err(&client->dev, "%s: switch to bank 0x%x from 0x%x failed\n",
			__func__, bank, client->addr);
		return;
	}
	state->bank = bank;
}

static void lt6911uxc_i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	struct lt6911uxc *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	struct i2c_msg msg;
	u8 data[2];
	u8 address;
	
	/* write register bank offset */
	u8 bank = (reg >> 8) & 0xFF;
	lt6911uxc_reg_bank(sd, bank);

	address = reg & 0xFF;
	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = address;
	data[1] = val;

	err = i2c_transfer(client->adapter, &msg, 1);

	if (err != 1) {
		dev_err(&client->dev, "%s: write register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
		return;
	}
	dev_dbg(&client->dev, "i2c: write register: 0x%04X = 0x%02X\n",
		reg, val);
}

static void lt6911uxc_i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct lt6911uxc *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 reg_addr[1] = { (u8)(reg & 0xff) };
	u8 bank_addr   = (u8)((reg >> 8) & 0xFF);

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,		/* write */
			.len = 1,
			.buf = reg_addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,	/* read n bytes */
			.len = n,
			.buf = values,
		},
	};	

	/* write register bank offset */
	lt6911uxc_reg_bank(sd, bank_addr);

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: read register 0x%04X from 0x%x failed\n",
			__func__, reg, client->addr);
	}
}

static u8 lt6911uxc_i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	u8 val;

	lt6911uxc_i2c_rd(sd, reg, &val, 1);

	dev_dbg(sd->dev, "i2c: read 0x%04X = 0x%02X\n", reg, val);
	return val;
}

static u16 lt6911uxc_i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val;
	u8 tmp;

	lt6911uxc_i2c_rd(sd, reg, (u8 *)&val, 2);
	/* high byte always at lower address -> swap */
	tmp = val & 0xFF;
	val = (val >> 8) | tmp << 8;

	dev_dbg(sd->dev, "i2c: read 0x%04X = 0x%04X\n", reg, val);
	return val;
}

/* ------ STATUS / CTRL ----------------------------------------------------- */

static inline bool no_signal(struct v4l2_subdev *sd)
{
	struct lt6911uxc *state = to_state(sd);
	return !state->signal_present;
}

static void lt6911uxc_ext_control(struct v4l2_subdev *sd, bool enable)
{
	struct lt6911uxc *state = to_state(sd);

	if(state->enable_i2c == enable)
		return;

	state->enable_i2c = enable;
	if (enable) {
		dev_dbg(sd->dev, "%s(): enable external i2c control\n", __func__);
		lt6911uxc_i2c_wr8(sd, ENABLE_I2C, 0x01);
		lt6911uxc_i2c_wr8(sd, DISABLE_WD, 0x00);
	} else {
		dev_dbg(sd->dev, "%s(): disable external i2c control\n", __func__);
		lt6911uxc_i2c_wr8(sd, ENABLE_I2C, 0x00);
	}
}

static int lt6911uxc_csi_enable(struct v4l2_subdev *sd, bool enable)
{
	dev_dbg(sd->dev, "%s():\n", __func__);

	if (enable) {
		lt6911uxc_i2c_wr8(sd, MIPI_TX_CTRL, 0xFB);
	} else {
		lt6911uxc_i2c_wr8(sd, MIPI_TX_CTRL, 0x00);
	}
	return 0;
}

static int lt6911uxc_get_audio_sampling_rate(struct lt6911uxc* state)
{
	int audio_fs, idx;
	static const int eps = 1500;
	static const int rates_default [] = {
		32000, 44100, 48000, 88200, 96000, 176400, 192000
	};

	audio_fs = lt6911uxc_i2c_rd8(state->sd, AUDIO_SR) * 1000;
	dev_dbg(&state->i2c_client->dev, "%s: Audio sample rate %d [Hz]\n",
		__func__, audio_fs);

	/* audio_fs is an approximation of sample rate - search nearest */
	for(idx = 0; idx < ARRAY_SIZE(rates_default); ++idx)
	{
		if ((rates_default[idx] - eps < audio_fs) &&
		    (rates_default[idx] + eps > audio_fs))
			return rates_default[idx];
	}
	dev_err(&state->i2c_client->dev, "%s: unhandled sampling rate %d [Hz]",
		__func__, audio_fs);
	return 0;
}

/* ------ TIMINGS ----------------------------------------------------------- */

 static int lt6911uxc_detect_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings, u8 lanes)
 {
	struct v4l2_bt_timings *bt = &timings->bt;
	u32 htot, vtot, fps;
	u16 width, height;
	u8 fm2, fm1, fm0, pol;
	int half_pixel_clk, frame_interval;
	
	struct lt6911uxc *state = to_state(sd);
	struct device *dev = &state->i2c_client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (no_signal(sd)) {
		v4l2_err(sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	timings->type  = V4L2_DV_BT_656_1120;
	bt->interlaced = V4L2_DV_PROGRESSIVE;

	/* video frame size */
	width  = lt6911uxc_i2c_rd16(sd, H_ACTIVE_0P5);
	if (lanes <= 4) {
		width = width * 2;
	}
	height = lt6911uxc_i2c_rd16(sd, V_ACTIVE);
	v4l2_dbg(1, debug, sd, "frame active - width %u height %u\n",
		width, height);
	bt->width = width;
	bt->height = height;

	/* front/back porch, sync pulse */
	bt->hfrontporch	= lt6911uxc_i2c_rd16(sd, H_FP_0P5);
	bt->hbackporch	= lt6911uxc_i2c_rd16(sd, H_BP_0P5);
	bt->hsync	= lt6911uxc_i2c_rd16(sd, H_SW_0P5);
	if (lanes <= 4) {
		bt->hfrontporch = bt->hfrontporch * 2;
		bt->hbackporch = bt->hbackporch * 2;
		bt->hsync = bt->hsync * 2;
	}

	bt->vfrontporch	= lt6911uxc_i2c_rd8(sd, V_FP);
	bt->vbackporch	= lt6911uxc_i2c_rd8(sd, V_BP);
	bt->vsync	= lt6911uxc_i2c_rd8(sd, V_SW);

	pol = lt6911uxc_i2c_rd8(sd, SYNC_POL);
	if (pol & MASK_HSYNC_POL)
		bt->polarities |= V4L2_DV_HSYNC_POS_POL;
	if (pol & MASK_VSYNC_POL)
		bt->polarities |= V4L2_DV_VSYNC_POS_POL;

	/* ------  pixelclock ------ */

	/* set frequency meter to half pixel clock */
	lt6911uxc_i2c_wr8(sd, AD_HALF_PCLK, 0x21);
	usleep_range(10000,10100);	/* needed by manufacturer */

	fm2 = lt6911uxc_i2c_rd8(sd, FM1_FREQ_IN2) & MASK_FMI_FREQ2;
	fm1 = lt6911uxc_i2c_rd8(sd, FM1_FREQ_IN1);
	fm0 = lt6911uxc_i2c_rd8(sd, FM1_FREQ_IN0);

	if (lanes <= 4) {
		half_pixel_clk = fm2 << 16 | fm1 << 8 | fm0;
	} else {
		half_pixel_clk = (fm2 << 16 | fm1 << 8 | fm0) / 2;
	}

	v4l2_dbg(1, debug, sd, "pixel clock %d\n", half_pixel_clk*2);

	htot = V4L2_DV_BT_FRAME_WIDTH(bt);
	vtot = V4L2_DV_BT_FRAME_HEIGHT(bt);

	/* frameinterval in ms */
	frame_interval = DIV_ROUND_CLOSEST((htot * vtot), (half_pixel_clk * 2));
	fps = DIV_ROUND_CLOSEST((half_pixel_clk * 2 * 1000 * 100), (htot * vtot));
	v4l2_dbg(1, debug, sd, "frame_interval %d ms, fps %d.%02d\n",
		frame_interval, (int)(fps/100), (int)(fps%100));
	bt->pixelclock = half_pixel_clk * 2 * 1000;

	// Separate <250 MHz from >250 MHz for deskewing calibration
	if (bt->pixelclock >= 250000000) {
		s_data->mode = 1;
	} else {
		s_data->mode = 0;
	}

	/* sanity check */
	if ((bt->width < 640) || (bt->height < 480) ||
	    (htot <= width) || (vtot <= height)) {
		memset(timings, 0, sizeof(struct v4l2_dv_timings));
		return -ENOLCK;
	}
	return 0;
}

/* ------ CORE OPS ---------------------------------------------------------- */

static int lt6911uxc_log_status(struct v4l2_subdev *sd)
{
	struct lt6911uxc *state = to_state(sd);

	v4l2_info(sd, "----- Timings -----\n");
	if (!&state->detected_timings.bt.width) {
		v4l2_info(sd, "no video detected\n");
	} else {
		v4l2_print_dv_timings(sd->name, "detected format: ",
				&state->detected_timings, true);
	}
	v4l2_print_dv_timings(sd->name, "configured format: ", &state->timings,
			true);

	return 0;
}

static int lt6911uxc_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
		struct v4l2_event_subscription *sub)
{
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

/* ------ IRQ --------------------------------------------------------------- */

static void lt6911uxc_hdmi_int_handler(struct lt6911uxc *state,
		bool *handled)
{
	u8 int_event;
	struct v4l2_dv_timings timings = {};
	u8 fm2, fm1, fm0, lanes;
	int byte_clock;
	struct device *dev = &state->i2c_client->dev;

	/* Read interrupt event */
	int_event = lt6911uxc_i2c_rd8(state->sd, INT_HDMI);
	dev_dbg(dev, "%s: HDMI event  =  0x%02X\n", __func__, int_event);

	switch(int_event) {
	case INT_HDMI_DISCONNECT:
		/* stop MIPI output */
		dev_info(dev,"HDMI signal disconnected\n");
		lt6911uxc_csi_enable(state->sd, false);
		v4l2_ctrl_s_ctrl(state->num_mipi_lanes_ctrl, 0);

		if (state->signal_present) {
			state->signal_present = false;
			v4l2_subdev_notify_event(state->sd,
						&lt6911uxc_ev_source_change);
			dev_dbg(dev,"event: no signal\n");

			memset(&state->timings, 0, sizeof(state->timings));
			memset(&state->detected_timings, 0,
			       sizeof(state->detected_timings));
		}
		if (handled)
			*handled = true;
		break;

	case INT_HDMI_STABLE:
		dev_info(dev, "HDMI signal stable\n");

		/* at each HDMI-stable event renew timings */
		state->signal_present = true;
		lanes = lt6911uxc_i2c_rd8(state->sd, MIPI_LANES);
		dev_dbg(dev, "MIPI lanes %d\n", lanes);
		v4l2_ctrl_s_ctrl(state->num_mipi_lanes_ctrl, lanes);
		lt6911uxc_detect_timings(state->sd, &timings, lanes);
		
		/* byte clock / MIPI clock */
		lt6911uxc_i2c_wr8(state->sd, AD_HALF_PCLK, 0x1B);
		usleep_range(10000,10100);
		fm2 = lt6911uxc_i2c_rd8(state->sd, FM1_FREQ_IN2) & MASK_FMI_FREQ2;
		fm1 = lt6911uxc_i2c_rd8(state->sd, FM1_FREQ_IN1);
		fm0 = lt6911uxc_i2c_rd8(state->sd, FM1_FREQ_IN0);

		byte_clock = fm2 << 16 | fm1 << 8 | fm0;
		dev_dbg(dev, "byte clock %d [kHz], MIPI clock %d [kHz]\n",
			byte_clock, byte_clock*4);

		/* MIPI */ 
		//lanes = lt6911uxc_i2c_rd8(state->sd, MIPI_LANES);
		//dev_dbg(dev, "MIPI lanes %d\n", lanes);

		lt6911uxc_csi_enable(state->sd, true);

		/* store newly detected timings (if any) if those are
		 * detected for the first time */
		if (!state->detected_timings.bt.width) {
			state->detected_timings = timings;
			dev_dbg(dev,"store new timings");
		} else if (tegra_v4l2_match_dv_timings(&timings, 
				&state->detected_timings, 250000, false)) {
			dev_dbg(dev,"ignore timings change");
		} else {
			state->detected_timings = timings;
			dev_dbg(dev,"detected timings updated");
		}


		if (handled)
			*handled = true;
		break;
	default:
		dev_err(dev, "%s: unhandled  = 0x%02X\n", __func__, int_event);
		return;
	}
}

static void lt6911uxc_audio_int_handler(struct lt6911uxc *state, 
		bool *handled)
{
	u8 int_event;
	int audio_fs = 0;
	struct device *dev = &state->i2c_client->dev;
	
	/* read interrupt event */
	int_event = lt6911uxc_i2c_rd8(state->sd, INT_AUDIO);
	dev_dbg(dev,"%s: Audio event  =  0x%02X\n", __func__, int_event);

	switch(int_event) {
	case INT_AUDIO_DISCONNECT:
		dev_info(dev,"Audio signal disconnected");
		audio_fs = 0;
		break;
	case INT_AUDIO_SR_HIGH:
	case INT_AUDIO_SR_LOW:
		if (state->signal_present) {
			dev_info(dev,"Audio sampling rate changed");
			audio_fs = lt6911uxc_get_audio_sampling_rate(state);
		}
		else
			audio_fs = 0;
		break;
	default:
		dev_err(dev,"%s: unhandled = 0x%02X\n", __func__, int_event);
		return;
	}

	v4l2_ctrl_s_ctrl(state->audio_present_ctrl, (audio_fs != 0));
	v4l2_ctrl_s_ctrl(state->audio_sampling_rate_ctrl, audio_fs);

	if (handled)
		*handled = true;
	return;
}

static int lt6911uxc_isr(struct v4l2_subdev *sd, bool *handled)
{
	struct lt6911uxc *state = to_state(sd);

	mutex_lock(&state->lock);
	dev_dbg(sd->dev, "%s in kthread %d\n", __func__, current->pid);

	lt6911uxc_ext_control(sd, true);

	/* Retrieve interrupt event */
	lt6911uxc_hdmi_int_handler(state, handled);

	lt6911uxc_audio_int_handler(state, handled);
	
	lt6911uxc_log_status(sd);

	lt6911uxc_ext_control(sd, false);

	mutex_unlock(&state->lock);
	return 0;
}

static irqreturn_t lt6911uxc_irq_handler(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = dev_id;
	bool handled = false;

	lt6911uxc_isr(sd, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/* ------ VIDEO OPS --------------------------------------------------------- */

static const struct v4l2_dv_timings_cap* lt6911uxc_g_timings_cap(
		struct lt6911uxc *state)
{
	return &lt6911uxc_timings_cap_4kp30;
}

static int lt6911uxc_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);
	return 0;
}

static int lt6911uxc_s_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxc *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (!v4l2_valid_dv_timings(timings, lt6911uxc_g_timings_cap(state),
				   NULL, NULL)) {
		v4l2_err(sd, "%s: timings out of range\n", __func__);
		return -EINVAL;
	}

	/* Fill optional fields .standards and .flags if format is part of
	 * CEA-861 / VESA-DMT timings */
	v4l2_find_dv_timings_cap(timings, lt6911uxc_g_timings_cap(state), 0,
				 NULL, NULL);

	/* Verify if new timings match current timings */
	if (tegra_v4l2_match_dv_timings(timings, &state->timings, 0, false)) {
		v4l2_info(sd, "%s: no change\n", __func__);
		return 0;
	}

	memset(timings->bt.reserved, 0, sizeof(timings->bt.reserved));
	state->timings = *timings;

	if (debug)
		v4l2_print_dv_timings(sd->name, "s_dv_timings: ",
				&state->timings, true);
	return 0;
}

static int lt6911uxc_g_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxc *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	*timings = state->timings;
	return 0;
}

static int lt6911uxc_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct lt6911uxc *state = to_state(sd);

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (no_signal(sd)) {
		v4l2_warn(sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	if (!v4l2_valid_dv_timings(&state->detected_timings,
				lt6911uxc_g_timings_cap(state), NULL, NULL)) {
		v4l2_warn(sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	*timings = state->detected_timings;
	if (debug)
		v4l2_print_dv_timings(sd->name, "query_dv_timings: ",
				timings, true);
	return 0;
}

static int lt6911uxc_s_stream(struct v4l2_subdev *sd, int enable)
{
	v4l2_dbg(3, debug, sd, "%s(): enable %d \n", __func__, enable);

	/* handled by ISR */
	return 0;
}

/* ------ PAD OPS ----------------------------------------------------------- */

static int lt6911uxc_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *cfg,
		struct v4l2_subdev_format *format)
{
	struct lt6911uxc *state = to_state(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int i = 0;

	v4l2_dbg(3, debug, sd,"%s():\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	/* retrieve mbus pixelcode and active video frame size */
	fmt->code = state->mbus_fmt_code;
	fmt->width  = state->timings.bt.width;
	fmt->height = state->timings.bt.height;
	fmt->field  = V4L2_FIELD_NONE;

	for (i = 0; i < ARRAY_SIZE(lt6911uxc_color_fmts); i++) {
		if (lt6911uxc_color_fmts[i].code == fmt->code) {
			fmt->colorspace = lt6911uxc_color_fmts[i].colorspace;
			break;
		}
	}

	switch (fmt->code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
	default:
		fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
		fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		break;
	}
	
	return 0;
}

static int lt6911uxc_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *cfg,
		struct v4l2_subdev_format *format)
{
	struct lt6911uxc *state = to_state(sd);
	u32 code = format->format.code; /* is overwritten by get_fmt */
	int ret;

	v4l2_dbg(2, debug, sd,
		"%s(): query format - width=%d, height=%d, code=0x%08X\n",
		__func__, format->format.width, format->format.height, code);

	/* adjust requested format based on current DV timings */
	ret = lt6911uxc_get_fmt(sd, cfg, format);
	format->format.code = code;

	if (ret)
		return ret;

	switch (code) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
		break;
	default:
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	state->mbus_fmt_code = format->format.code;
	v4l2_dbg(2, debug, sd,
		"%s(): current format - width=%d, height=%d, code=0x%08X\n",
		__func__, format->format.width, format->format.height,
		state->mbus_fmt_code);
	return 0;
}

static int lt6911uxc_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_state *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	v4l2_dbg(3, debug, sd, "%s()\n", __func__);

	if (code->index >= ARRAY_SIZE(lt6911uxc_color_fmts))
		return -EINVAL;

	code->code = lt6911uxc_color_fmts[code->index].code;
	v4l2_dbg(2, debug, sd, "%s(): fmt-code 0x%04X\n", __func__, code->code);

	return 0;
}

static int lt6911uxc_dv_timings_cap(struct v4l2_subdev *sd,
		struct v4l2_dv_timings_cap *cap)
{
	struct lt6911uxc *state = to_state(sd);
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (cap->pad != 0)
		return -EINVAL;

	*cap = *lt6911uxc_g_timings_cap(state);
	return 0;
}

static int lt6911uxc_enum_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_enum_dv_timings *timings)
{
	struct lt6911uxc *state = to_state(sd);
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	if (timings->pad != 0)
		return -EINVAL;

	/* filter non supported DV timings */
	return v4l2_enum_dv_timings_cap(timings,
				lt6911uxc_g_timings_cap(state), NULL, NULL);
}

/* ------ Register OPS ------------------------------------------------------ */

static int lt6911uxc_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops lt6911uxc_subdev_internal_ops = {
	.open = lt6911uxc_open,
};

static struct v4l2_subdev_core_ops lt6911uxc_subdev_core_ops = {
	.log_status 		= lt6911uxc_log_status,
	.subscribe_event 	= lt6911uxc_subscribe_event,
	.unsubscribe_event 	= v4l2_event_subdev_unsubscribe,
};

static struct v4l2_subdev_video_ops lt6911uxc_subdev_video_ops = {
	.g_input_status		= lt6911uxc_g_input_status,	
	.s_dv_timings		= lt6911uxc_s_dv_timings,
	.g_dv_timings		= lt6911uxc_g_dv_timings,
	.query_dv_timings	= lt6911uxc_query_dv_timings,
	.s_stream		= lt6911uxc_s_stream,
};

static const struct v4l2_subdev_pad_ops lt6911uxc_pad_ops = {
	.set_fmt		= lt6911uxc_set_fmt,
	.get_fmt		= lt6911uxc_get_fmt,
	.enum_mbus_code		= lt6911uxc_enum_mbus_code,
	.dv_timings_cap		= lt6911uxc_dv_timings_cap,
	.enum_dv_timings	= lt6911uxc_enum_dv_timings,
};

static struct v4l2_subdev_ops lt6911uxc_ops = {
	.core 	= &lt6911uxc_subdev_core_ops,
	.video 	= &lt6911uxc_subdev_video_ops,
	.pad 	= &lt6911uxc_pad_ops,
};

#ifdef CONFIG_MEDIA_CONTROLLER
static const struct media_entity_operations lt6911uxc_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};
#endif

/* ------ CUSTOM CTRLS ------------------------------------------------------ */

static const struct v4l2_ctrl_config lt6911uxc_ctrl_audio_sampling_rate = {
	.id = LT6911UXC_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio Sampling Rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 192000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt6911uxc_ctrl_audio_present = {
	.id = LT6911UXC_CID_AUDIO_PRESENT,
	.name = "Audio Present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt6911uxc_ctrl_num_mipi_lanes = {
	.id = LT6911UXC_CID_NUM_MIPI_LANES,
	.name = "Number of MIPI Lanes Used",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 8,
	.step = 4,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

/* ------ Driver setup ------------------------------------------------------ */

static void lt6911uxc_initial_setup(struct lt6911uxc *state)
{
	state->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_1X16;
	state->signal_present = false;
	state->enable_i2c = false;
	mutex_init(&state->lock);

	/* Init Timings */
	lt6911uxc_s_dv_timings(state->sd, &default_timing);
}

static const struct of_device_id lt6911uxc_of_match[] = {
	{ .compatible = "lontium,lt6911uxc_zhaw" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt6911uxc_of_match);

static struct camera_common_pdata* lt6911uxc_parse_dt(struct i2c_client *client,
				struct camera_common_data *s_data)
{
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err = 0;
	int gpio;
	const char *str;

	if (!np)
		return NULL;

	match = of_match_device(lt6911uxc_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	err = of_property_read_string(np, "use_sensor_mode_id", &str);
	if (!err) {
		if (!strcmp(str, "true"))
			s_data->use_sensor_mode_id = true;
		else
			s_data->use_sensor_mode_id = false;
	} else {
		dev_info(&client->dev, "use_sensor_mode_id not found, setting to false\n");
		s_data->use_sensor_mode_id = false;
	}
	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);

	gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if(gpio < 0) {
		if(gpio == -EPROBE_DEFER) {
			dev_err(&client->dev, "reset-gpio read failed: (%d)\n",
				gpio);
			goto prop_err;
		}
		dev_info(&client->dev, "reset-gpio not found, ignoring\n");
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;
	return board_priv_pdata;

prop_err:
	dev_err(&client->dev, "Could not parse DT parameters\n");
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static struct camera_common_sensor_ops lt6911uxc_common_ops = {
	.numfrmfmts = ARRAY_SIZE(lt6911uxc_frmfmt),
	.frmfmt_table = lt6911uxc_frmfmt,
};



static int lt6911uxc_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct lt6911uxc *priv;
	struct v4l2_subdev *sd;
	int err = 0;

	dev_info(&client->dev, "Probing lt6911uxc\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;
	
	priv = devm_kzalloc(&client->dev, 
				sizeof(struct lt6911uxc),
			    GFP_KERNEL);
	if (!priv)
	{
		dev_err(&client->dev, "unable to allocate priv memory!\n");
		return -ENOMEM;
	}

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), 
				GFP_KERNEL);

	if (!common_data) {
		dev_err(&client->dev, "unable to allocate common data memory!\n");
		return -ENOMEM;
	}

	if (client->dev.of_node)
		priv->pdata = lt6911uxc_parse_dt(client, common_data);

	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	
	common_data->ops = &lt6911uxc_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &lt6911uxc_frmfmt[0];
	common_data->colorfmt = lt6911uxc_color_fmts;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = LT6911UXC_NUM_CTRLS;
	common_data->numfmts = ARRAY_SIZE(lt6911uxc_frmfmt);
	common_data->def_mode = 0;
	
	common_data->def_width = 1920;
	common_data->def_height = 1080;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;


	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->sd = &common_data->subdev;
	priv->sd->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	sd = priv->sd;


	/* initial setup */
	lt6911uxc_initial_setup(priv);

	err = camera_common_initialize(common_data, "lt6911uxc_zhaw");
	if (err) {
		dev_err(&client->dev, "Failed to initialize lt6911uxc.\n");
		return err;
	}

	v4l2_i2c_subdev_init(sd, client, &lt6911uxc_ops);

	dev_info(&client->dev, "Chip found @ 7h%02X (%s)\n", client->addr,
		 client->adapter->name);

	/* get interrupt */
	if (client->irq) {
		err = devm_request_threaded_irq(&priv->i2c_client->dev,
						client->irq, NULL,
						lt6911uxc_irq_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT, sd->name,
						(void *)sd);
		if (err) {
			dev_err(&client->dev,"Could not request interrupt %d!\n",
				client->irq);
			return err;
		}
	}

	/* custom v4l2 controls */
	v4l2_ctrl_handler_init(&priv->ctrl_handler, LT6911UXC_NUM_CTRLS);

	priv->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(
		&priv->ctrl_handler, &lt6911uxc_ctrl_audio_sampling_rate, NULL);
	priv->ctrls[0] = priv->audio_sampling_rate_ctrl;

	priv->audio_present_ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
		&lt6911uxc_ctrl_audio_present, NULL);
	priv->ctrls[1] = priv->audio_present_ctrl;

	priv->num_mipi_lanes_ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
		&lt6911uxc_ctrl_num_mipi_lanes, NULL);
	priv->ctrls[2] = priv->num_mipi_lanes_ctrl;	

	sd->ctrl_handler = &priv->ctrl_handler;

	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto err_ctrl_handler;
	}

	err = v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto err_ctrl_handler;
	}

	/* register v4l2_subdev device */

	sd->internal_ops = &lt6911uxc_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;



	/* media entitiy: define pad as output -> origins of link */
	if (IS_ENABLED(CONFIG_MEDIA_CONTROLLER)) {
		priv->pad[0].flags = MEDIA_PAD_FL_SOURCE;
		sd->entity.ops = &lt6911uxc_media_ops;

		err = tegra_media_entity_init(&sd->entity, 1,
					priv->pad, true, false);
		if (err < 0) {
			dev_err(&client->dev, "unable to init media entity\n");
			goto err_ctrl_handler;
		}
	}


	err = v4l2_async_register_subdev(sd);
	if (err) {
		dev_err(&client->dev, "lt6911uxc subdev registration failed\n");
		goto err_ctrl_handler;
	}

	dev_info(&client->dev, "Detected lt6911uxc device\n");
	return 0;

 err_ctrl_handler:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static int lt6911uxc_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct lt6911uxc *priv = (struct lt6911uxc *)s_data->priv;

	dev_dbg(&client->dev, "%s \n", __func__);

	v4l2_async_unregister_subdev(priv->sd);

	if (IS_ENABLED(CONFIG_MEDIA_CONTROLLER)) {
		media_entity_cleanup(&priv->sd->entity);
	}

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_cleanup(s_data);

	dev_info(&client->dev, "removed lt6911uxc instance \n");
	return 0;
}

static const struct i2c_device_id lt6911uxc_id[] = {
	{ "lt6911uxc_zhaw", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lt6911uxc_id);

static struct i2c_driver lt6911uxc_driver = {
	.driver = {
		.of_match_table = of_match_ptr(lt6911uxc_of_match),
		.name = "lt6911uxc_zhaw",
		.owner = THIS_MODULE,
	},
	.id_table = lt6911uxc_id,
	.probe 	  = lt6911uxc_probe,
	.remove   = lt6911uxc_remove,
};
module_i2c_driver(lt6911uxc_driver);

MODULE_DESCRIPTION("Driver for Lontium lt6911uxc HDMI to CSI-2 Bridge");
MODULE_AUTHOR("Lukas Neuner <neur@zhaw.ch>");
MODULE_AUTHOR("Alexey Gromov <groo@zhaw.ch>");
MODULE_AUTHOR("Gianluca Pargaetzi <parg@zhaw.ch>");
MODULE_AUTHOR("Michael Wäspe <waep@zhaw.ch>");
MODULE_LICENSE("GPL v2");