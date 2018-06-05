/*
 * hid-unitec.c - HID driver for UNITEC USB Touch
 *
 * Copyright (c) 2011-2012 UNITEC Co.,LTD.
 *
 * This code is partly based on hid-3m-pct.c:
 *
 * Copyright (c) 2009-2010 Stephane Chatty <chatty@enac.fr>
 * Copyright (c) 2010      Henrik Rydberg <rydberg@euromail.se>
 * Copyright (c) 2010      Canonical, Ltd.
 *
 * This code is partly based on hid-multitouch.c:
 *
 * Copyright (c) 2010-2011 Stephane Chatty <chatty@enac.fr>
 * Copyright (c) 2010-2011 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2010-2011 Ecole Nationale de l'Aviation Civile, France
 */

/* 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation; or, when distributed
 * separately from the Linux kernel or incorporated into other
 * software packages, subject to the following license:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <asm/unaligned.h>
#include "usbhid/usbhid.h"

MODULE_AUTHOR("Fumihiro Atsumi <atsumi@pylone.jp>");
MODULE_DESCRIPTION("UNITEC USB Touch");
MODULE_LICENSE("GPL");

#include "hid-ids.h"

#define MAX_FINGERS	10
#define MAX_TRKID	USHRT_MAX
#define MAX_EVENTS	360

#define UNITEC_STAT_REMOVING	1

struct unitec_finger {
	__s32 x, y;
	__s32 contactid;
	__u16 trkid;
	bool touch_state;
	bool prev_touch_state;
	bool valid;
	bool delimiter;
};

struct unitec_data {
	struct unitec_finger curdata;
	__s8 inputmode;
	__u8 num_received;
	__u8 num_expected;
	__u8 maxcontacts;
	__u16 trkid;
	unsigned int status;
	struct unitec_finger f[MAX_FINGERS];
};

static void set_abs(struct input_dev *input, unsigned int code,
		struct hid_field *field, int snratio)
{
	int fmin = field->logical_minimum;
	int fmax = field->logical_maximum;
	int fuzz = snratio ? (fmax - fmin) / snratio : 0;
	input_set_abs_params(input, code, fmin, fmax, fuzz, 0);
}

static int unitec_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	switch (usage->hid & HID_USAGE_PAGE) {

	case HID_UP_BUTTON:
		return -1;

	case HID_UP_GENDESK:
		switch (usage->hid) {
		case HID_GD_X:
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_POSITION_X);
			set_abs(hi->input, ABS_MT_POSITION_X, field, 0);
			set_abs(hi->input, ABS_X, field, 0);
			return 1;
		case HID_GD_Y:
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_POSITION_Y);
			set_abs(hi->input, ABS_MT_POSITION_Y, field, 0);
			set_abs(hi->input, ABS_Y, field, 0);
			return 1;
		}
		return 0;

	case HID_UP_DIGITIZER:
		switch (usage->hid) {
		/* we do not want to map these: no input-oriented meaning */
		case HID_DG_CONFIDENCE:
		case HID_DG_INPUTMODE:
		case HID_DG_DEVICEINDEX:
		case HID_DG_CONTACTCOUNT:
		case HID_DG_CONTACTMAX:
		case HID_DG_INRANGE:
			return -1;
		case HID_DG_TIPSWITCH:
			input_set_capability(hi->input, EV_KEY, BTN_TOUCH);
			return 1;
		case HID_DG_CONTACTID:
			field->logical_maximum = MAX_TRKID;
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_TRACKING_ID);
			set_abs(hi->input, ABS_MT_TRACKING_ID, field, 0);
			input_set_events_per_packet(hi->input, MAX_EVENTS);
			return 1;
		}
		/* let hid-input decide for the others */
		return 0;

	case 0xff000000:
		/* we do not want to map these: no input-oriented meaning */
		return -1;
	}

	return 0;
}

static int unitec_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	if (usage->type == EV_KEY || usage->type == EV_ABS)
		set_bit(usage->type, hi->input->evbit);

	return -1;
}

static void unitec_emit_event(struct unitec_data *ud, struct input_dev *input)
{
	int count = 0;
	struct unitec_finger *oldest = 0;
	int i;

	for (i = 0; i < MAX_FINGERS; i++) {
		struct unitec_finger *f = &ud->f[i];
		if (!f->valid)
			continue;
		if (f->touch_state) {
			if(!f->prev_touch_state)
				f->trkid = ud->trkid++;
			input_event(input, EV_ABS, ABS_MT_TRACKING_ID, i);
			input_event(input, EV_ABS, ABS_MT_POSITION_X, f->x);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y, f->y);
			input_mt_sync(input);
			count++;

			/* touchscreen emulation: pick the oldest contact */
			if (!oldest || ((f->trkid - oldest->trkid) & (SHRT_MAX + 1)))
				oldest = f;
		}
		f->valid = 0;
	}
	/* touchscreen emulation */
	if (oldest) {
		input_event(input, EV_KEY, BTN_TOUCH, 1);
		input_event(input, EV_ABS, ABS_X, oldest->x);
		input_event(input, EV_ABS, ABS_Y, oldest->y);
	} else {
		input_event(input, EV_KEY, BTN_TOUCH, 0);
	}
	if (!count)
		input_mt_sync(input);

	input_sync(input);
	ud->num_received = 0;
}

static __inline__ __u32 extract(__u8 *report, unsigned offset, unsigned n)
{
	u64 x;

	if (n > 32)
		printk(KERN_WARNING "hid-unitec: extract() called with n (%d) > 32! (%s)\n",
				n, current->comm);

	report += offset >> 3;  /* adjust byte index */
	offset &= 7;            /* now only need bit offset into one byte */
	x = get_unaligned_le64(report);
	x = (x >> offset) & ((1ULL << n) - 1);  /* extract bit field */
	return (u32) x;
}

static int unitec_raw_event(struct hid_device *hdev,
			struct hid_report *report, u8 *data, int size)
{
	struct unitec_data *ud = hid_get_drvdata(hdev);
	struct hid_input *hidinput;
	struct input_dev *input;

	if (!(hdev->claimed & HID_CLAIMED_INPUT))
		return 0;

	if (report->id != 0x04 || size != 6)
		return 0;

	hidinput = list_entry(hdev->inputs.next, struct hid_input, list);
	input = hidinput->input;

	data++;
	ud->curdata.prev_touch_state = ud->curdata.touch_state;
	ud->curdata.touch_state = extract(data, 0, 1);
	ud->curdata.contactid = extract(data, 2, 4);
	ud->curdata.delimiter = extract(data, 6, 1);
	ud->curdata.valid = extract(data, 7, 1);
	ud->curdata.x = extract(data, 8, 16);
	ud->curdata.y = extract(data, 24, 16);

	ud->f[ud->curdata.contactid] = ud->curdata;
	ud->num_received++;

	if (ud->num_received > 0 && ud->curdata.delimiter)
		unitec_emit_event(ud, input);

	return 1;
}

static void unitec_set_input_mode(struct hid_device *hdev)
{
	struct unitec_data *ud = hid_get_drvdata(hdev);
	struct hid_report *r;
	struct hid_report_enum *re;

	if (ud->inputmode < 0)
		return;

	re = &(hdev->report_enum[HID_FEATURE_REPORT]);
	r = re->report_id_hash[ud->inputmode];
	if (r) {
		r->field[0]->value[0] = 0x02;
		usbhid_submit_report(hdev, r, USB_DIR_OUT);
	}
}

static int unitec_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct unitec_data *ud;

	hdev->quirks |= HID_QUIRK_NO_INPUT_SYNC;

	ud = kzalloc(sizeof(struct unitec_data), GFP_KERNEL);
	if (!ud) {
		dev_err(&hdev->dev, "cannot allocate UNITEC data\n");
		return -ENOMEM;
	}
	ud->inputmode = 0x05;
	hid_set_drvdata(hdev, ud);

	ret = hid_parse(hdev);
	if (ret)
		goto end;

	hdev->quirks &= ~HID_QUIRK_NOGET;

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		goto end;

	unitec_set_input_mode(hdev);

end:
	if (ret)
		kfree(ud);

	return ret;
}

#ifdef CONFIG_PM
static int unitec_reset_resume(struct hid_device *hdev)
{
	unitec_set_input_mode(hdev);
	return 0;
}
#endif

static void unitec_remove(struct hid_device *hdev)
{
	struct unitec_data *ud = hid_get_drvdata(hdev);

	ud->status |= UNITEC_STAT_REMOVING;
	hid_hw_stop(hdev);
	kfree(ud);
	hid_set_drvdata(hdev, NULL);
}

static const struct hid_device_id unitec_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_DMC, USB_DEVICE_ID_DMC_USB_TOUCH_07d2) },
	{ }
};
MODULE_DEVICE_TABLE(hid, unitec_devices);

static struct hid_driver unitec_driver = {
	.name = "unitec-usbtouch",
	.id_table = unitec_devices,
	.probe = unitec_probe,
	.remove = unitec_remove,
	.raw_event = unitec_raw_event,
	.input_mapping = unitec_input_mapping,
	.input_mapped = unitec_input_mapped,
#ifdef CONFIG_PM
	.reset_resume = unitec_reset_resume,
#endif
};

static int __init unitec_init(void)
{
	return hid_register_driver(&unitec_driver);
}

static void __exit unitec_exit(void)
{
	hid_unregister_driver(&unitec_driver);
}

module_init(unitec_init);
module_exit(unitec_exit);

