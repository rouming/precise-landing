/*! ------------------------------------------------------------------------------------------------------------------
 * @file    test_les.c
 * @brief   Setup the node as a tag and print out the RTLS info.
 *          In this example, the steps to configure the Tag include:
 *          Tag mode, PANID, encryption key, update rate. 
 *          According to the network settings, some of these settings 
 *          are not necessary.
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <string.h>
#include "dwm_api.h"
#include "hal.h"
#include "hal_log.h"
#include "hal_gpio.h"
#include "test_util.h"
#include <sys/time.h>

struct timeval tv;
uint64_t ts_curr = 0;
uint64_t ts_last = 0;
volatile uint8_t data_ready;

static int frst(void)
{
	int rv;
	int err_cnt = 0;
	int delay_ms = 2000;

	HAL_Log("dwm_factory_reset().\n");
	rv = Test_CheckTxRx(dwm_factory_reset());
	HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
	HAL_Delay(delay_ms);
	err_cnt += rv;

	if (rv == RV_OK) {
		dwm_deinit();
		dwm_init();
	}

	printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ", __FUNCTION__, err_cnt);
	return err_cnt;
}

static int get_loc(void)
{
	// ========== dwm_loc_get ==========
	int rv, err_cnt = 0, i;

	dwm_loc_data_t loc;
	dwm_pos_t pos;
	loc.p_pos = &pos;
	HAL_Log("dwm_loc_get(&loc)\n");
	rv = Test_CheckTxRx(dwm_loc_get(&loc));

	gettimeofday(&tv,NULL);

	if (rv == RV_OK) {
		HAL_Log("ts:%ld.%06ld [%d,%d,%d,%u]\n", tv.tv_sec, tv.tv_usec, loc.p_pos->x, loc.p_pos->y, loc.p_pos->z, loc.p_pos->qf);
		printf("ts:%ld.%06ld [%d,%d,%d,%u]", tv.tv_sec, tv.tv_usec, loc.p_pos->x, loc.p_pos->y, loc.p_pos->z, loc.p_pos->qf);

		for (i = 0; i < loc.anchors.dist.cnt; ++i) {
			HAL_Log("#%u)", i);
			printf("#%u)", i);
			HAL_Log("a:0x%08x", loc.anchors.dist.addr[i]);
			printf("a:0x%08x", loc.anchors.dist.addr[i]);
			if (i < loc.anchors.an_pos.cnt) {
				HAL_Log("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
					loc.anchors.an_pos.pos[i].y,
					loc.anchors.an_pos.pos[i].z,
					loc.anchors.an_pos.pos[i].qf);
				printf("[%d,%d,%d,%u]", loc.anchors.an_pos.pos[i].x,
				       loc.anchors.an_pos.pos[i].y,
				       loc.anchors.an_pos.pos[i].z,
				       loc.anchors.an_pos.pos[i].qf);
			}
			HAL_Log("d=%u,qf=%u\n", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
			printf("d=%u,qf=%u", loc.anchors.dist.dist[i], loc.anchors.dist.qf[i]);
		}
		HAL_Log("\n");
		printf("\n");
	}
	fflush(stdout);
	err_cnt += rv;

	return err_cnt;
}

// configure the tag and restart the node
static int setup_tag(void)
{
	int rv, err_cnt = 0;
	int delay_ms = 1500;

	dwm_cfg_tag_t cfg_tag;
	dwm_cfg_t cfg;
	cfg_tag.stnry_en = 1; // XXX accel
	cfg_tag.low_power_en = 0;
	cfg_tag.meas_mode = DWM_MEAS_MODE_TWR; //XXX
	cfg_tag.loc_engine_en = 1; //XXX
	cfg_tag.common.enc_en = 0;
	cfg_tag.common.led_en = 1;
	cfg_tag.common.ble_en = 1; //XXX
	cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
	cfg_tag.common.fw_update_en = 0;

	HAL_Log("dwm_cfg_get(&cfg)\n");
	err_cnt += Test_CheckTxRx(dwm_cfg_get(&cfg));

	while((cfg_tag.low_power_en!= cfg.low_power_en)
	      || (cfg_tag.meas_mode      != cfg.meas_mode)
	      || (cfg_tag.loc_engine_en  != cfg.loc_engine_en)
	      || (cfg_tag.stnry_en       != cfg.stnry_en)
	      || (cfg_tag.common.enc_en  != cfg.common.enc_en)
	      || (cfg_tag.common.led_en  != cfg.common.led_en)
	      || (cfg_tag.common.ble_en  != cfg.common.ble_en)
	      || (cfg_tag.common.uwb_mode != cfg.common.uwb_mode)
	      || (cfg_tag.common.fw_update_en != cfg.common.fw_update_en)) {
		printf("Comparing set vs. get.\n");
		if (cfg.mode != DWM_MODE_TAG)
			printf("mode: get = %d, set = %d\n",
			       cfg.mode, DWM_MODE_TAG);
		if (cfg.stnry_en != cfg_tag.stnry_en)
			printf("acce: get = %d, set = %d\n",
			       cfg.stnry_en, cfg_tag.stnry_en);
		if (cfg.loc_engine_en != cfg_tag.loc_engine_en)
			printf("le  : get = %d, set = %d\n",
			       cfg.loc_engine_en, cfg_tag.loc_engine_en);
		if (cfg.low_power_en != cfg_tag.low_power_en)
			printf("lp  : get = %d, set = %d\n",
			       cfg.low_power_en, cfg_tag.low_power_en);
		if (cfg.meas_mode != cfg_tag.meas_mode)
			printf("meas: get = %d, set = %d\n",
			       cfg.meas_mode, cfg_tag.meas_mode);
		if (cfg.common.fw_update_en != cfg_tag.common.fw_update_en)
			printf("fwup: get = %d, set = %d\n",
			       cfg.common.fw_update_en, cfg_tag.common.fw_update_en);
		if (cfg.common.uwb_mode != cfg_tag.common.uwb_mode)
			printf("uwb : get = %d, set = %d\n",
			       cfg.common.uwb_mode, cfg_tag.common.uwb_mode);
		if (cfg.common.ble_en != cfg_tag.common.ble_en)
			printf("ble : get = %d, set = %d\n",
			       cfg.common.ble_en, cfg_tag.common.ble_en);
		if (cfg.common.led_en != cfg_tag.common.led_en)
			printf("led : get = %d, set = %d\n",
			       cfg.common.led_en, cfg_tag.common.led_en);

		HAL_Log("dwm_cfg_tag_set(&cfg_tag)\n");
		rv = Test_CheckTxRx(dwm_cfg_tag_set(&cfg_tag));
		err_cnt += rv;

		HAL_Log("dwm_reset()\n");
		rv = Test_CheckTxRx(dwm_reset());
		HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
		err_cnt += rv;
		HAL_Delay(delay_ms);

		HAL_Log("dwm_cfg_get(&cfg)\n");
		rv = Test_CheckTxRx(dwm_cfg_get(&cfg));
		err_cnt += rv;
	}
	HAL_Log("Done.\n");

	printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ",
	       __FUNCTION__, err_cnt);
	return err_cnt;
}

__attribute__((unused))
static int setup_enc(void)
{
	int rv;
	int err_cnt = 0;
	int i = 0;

	// clear the previously used key
	HAL_Log("dwm_enc_key_clear(void)\n");
	rv = Test_CheckTxRx(dwm_enc_key_clear());
	Test_Report("dwm_enc_key_clear(void):\t\t\t%s\n", rv==0 ? "pass":"fail");
	err_cnt += rv;

	dwm_enc_key_t key;

	// the key used in the example network is:
	// 0x 12 34 12 34 12 34 12 34 12 34 12 34 12 34 12 34
	while (i < DWM_ENC_KEY_LEN) {
		key.byte[i++] = 0x12;
		key.byte[i++] = 0x34;
	}

	HAL_Log("dwm_enc_key_set(&key)\n");
	rv = Test_CheckTxRx(dwm_enc_key_set(&key));
	Test_Report("dwm_enc_key_set(&key):\t\t\t%s\n", rv==0 ? "pass":"fail");
	err_cnt += rv;

	HAL_Delay(3000);

	printf("%s %s: err_cnt = %d\n", (err_cnt >0)? "ERR" : "   ",
	       __FUNCTION__, err_cnt);
	return err_cnt;
}

__attribute__((unused))
static void gpio_cb(void)
{
	printf(">> data ready\n");
	data_ready = 1;
}

static void do_data_loop(void)
{
	int err_cnt = 0;
	uint16_t panid;
	uint16_t ur_set;
	uint16_t ur_s_set;

	//init
	printf("Initializing...\n");
	HAL_Log("Initializing...\n");
	dwm_init();
	err_cnt += frst();
	HAL_Log("Done\n");

	/* ========= published APIs =========*/

	// setup tag mode configurations
	setup_tag();

	// setup tag update rate
	ur_s_set = ur_set = 1;
	HAL_Log("dwm_upd_rate_set(ur_set, ur_s_set);\n");
	dwm_upd_rate_set(ur_set, ur_s_set);

	// setup PANID
	panid = 0xbeb2;
	HAL_Log("dwm_panid_set(panid);\n");
	dwm_panid_set(panid);

#if 0
	// setup GPIO interrupt from "LOC_READY" event
	HAL_Log("dwm_int_cfg_set(DWM1001_INTR_LOC_READY);\n");
	dwm_int_cfg_set(DWM1001_INTR_LOC_READY);
	HAL_GPIO_SetupCb(HAL_GPIO_DRDY, HAL_GPIO_INT_EDGE_RISING, &gpio_cb);
#endif

#if 0
	// setup encryption key for network
	setup_enc();
#endif

	while (1) {
#if 0
		if (data_ready == 1) {
#endif
			data_ready = 0;
			get_loc();
#if 0
		}
#endif
		HAL_Delay(200);
	}

	HAL_Log("err_cnt = %d \n", err_cnt);

	Test_End();
}

int main(int argc, char*argv[])
{
	do_data_loop();
	return 0;
}
