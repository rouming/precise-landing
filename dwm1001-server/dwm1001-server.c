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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/time.h>

#include "dwm_api.h"
#include "hal.h"
#include "hal_log.h"
#include "hal_gpio.h"
#include "test_util.h"

struct timeval tv;
uint64_t ts_curr = 0;
uint64_t ts_last = 0;

#define UDP_GROUP "224.1.1.1"
#define UDP_PORT  5555

#define UPD_RATE    1 /* in 100ms */
#define UPD_RATE_MS (UPD_RATE * 100)

//#define LOG
//#define DEV_CONFIG
//#define DEV_FACTORY_RESET
//#define DEV_SETUP_ENC

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wpadded"

struct pos {
	int32_t x;
	int32_t y;
	int32_t z;
	uint16_t qf;
	uint16_t valid;
};

struct dist {
	uint32_t dist;
	uint16_t addr;
	uint16_t qf;
};

struct anchor {
	struct pos  pos;
	struct dist dist;
};

struct ts {
	uint32_t sec;
	uint32_t usec;
};

struct location {
	struct pos     calc_pos;
	struct ts      ts;
	uint32_t       nr_anchors;
	struct anchor  anchors[];
};

#pragma GCC diagnostic pop

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

static int get_loc(struct location *loc)
{
	// ========== dwm_loc_get ==========
	int rv, err_cnt = 0, i;

	dwm_loc_data_t dwm_loc;
	dwm_pos_t dwm_pos;

	dwm_loc.p_pos = &dwm_pos;
	HAL_Log("dwm_loc_get(&dwm_loc)\n");
	rv = Test_CheckTxRx(dwm_loc_get(&dwm_loc));

	gettimeofday(&tv,NULL);

	memset(loc, 0, sizeof(*loc));

	if (rv == RV_OK) {
		HAL_Log("ts:%ld.%06ld [%d,%d,%d,%u]\n",
			tv.tv_sec, tv.tv_usec, dwm_loc.p_pos->x,
			dwm_loc.p_pos->y, dwm_loc.p_pos->z, dwm_loc.p_pos->qf);
#ifdef LOG
		 printf("ts:%ld.%06ld [%d,%d,%d,%u]",
		        tv.tv_sec, tv.tv_usec, dwm_loc.p_pos->x, dwm_loc.p_pos->y,
			dwm_loc.p_pos->z, dwm_loc.p_pos->qf);
#endif

		*loc = (struct location) {
			.calc_pos = {
				.x     = dwm_loc.p_pos->x,
				.y     = dwm_loc.p_pos->y,
				.z     = dwm_loc.p_pos->z,
				.qf    = dwm_loc.p_pos->qf,
				.valid = 1,
			},
			.ts = {
				.sec  = tv.tv_sec,
				.usec = tv.tv_usec,
			},
			.nr_anchors = dwm_loc.anchors.dist.cnt,
		};

		for (i = 0; i < dwm_loc.anchors.dist.cnt; ++i) {
			struct anchor *an = &loc->anchors[i];

			HAL_Log("#%u)", i);
			HAL_Log("a:0x%08x", dwm_loc.anchors.dist.addr[i]);
#ifdef LOG
			printf("#%u)", i);
			printf("a:0x%08x", dwm_loc.anchors.dist.addr[i]);
#endif

			memset(an, 0, sizeof(*an));
			an->dist.dist = dwm_loc.anchors.dist.dist[i];
			an->dist.addr = dwm_loc.anchors.dist.addr[i];
			an->dist.qf   = dwm_loc.anchors.dist.qf[i];

			if (i < dwm_loc.anchors.an_pos.cnt) {
				an->pos.x  = dwm_loc.anchors.an_pos.pos[i].x;
				an->pos.y  = dwm_loc.anchors.an_pos.pos[i].y;
				an->pos.z  = dwm_loc.anchors.an_pos.pos[i].z;
				an->pos.qf = dwm_loc.anchors.an_pos.pos[i].qf;
				an->pos.valid = 1;

				HAL_Log("[%d,%d,%d,%u]", dwm_loc.anchors.an_pos.pos[i].x,
					dwm_loc.anchors.an_pos.pos[i].y,
					dwm_loc.anchors.an_pos.pos[i].z,
					dwm_loc.anchors.an_pos.pos[i].qf);
#ifdef LOG
				printf("[%d,%d,%d,%u]", dwm_loc.anchors.an_pos.pos[i].x,
				       dwm_loc.anchors.an_pos.pos[i].y,
				       dwm_loc.anchors.an_pos.pos[i].z,
				       dwm_loc.anchors.an_pos.pos[i].qf);
#endif
			}
			HAL_Log("d=%u,qf=%u\n", dwm_loc.anchors.dist.dist[i], dwm_loc.anchors.dist.qf[i]);
#ifdef LOG
			printf("d=%u,qf=%u", dwm_loc.anchors.dist.dist[i], dwm_loc.anchors.dist.qf[i]);
#endif
		}
		HAL_Log("\n");
#ifdef LOG
		printf("\n");
#endif
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

	memset(&cfg_tag, 0, sizeof(cfg_tag));
	cfg_tag.stnry_en = 1; // XXX accel
	cfg_tag.low_power_en = 0;
	cfg_tag.meas_mode = DWM_MEAS_MODE_TWR; //XXX
	cfg_tag.loc_engine_en = 1; //XXX
	cfg_tag.common.enc_en = 0;
	cfg_tag.common.led_en = 1;
	cfg_tag.common.ble_en = 1; //XXX
	cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
	cfg_tag.common.fw_update_en = 0;

#if DEV_FACTORY_RESET
	/* Factory reset */
	err_cnt = frst();
	if (err_cnt)
		return err_cnt;
#endif

	HAL_Log("dwm_cfg_get(&cfg)\n");
	err_cnt = Test_CheckTxRx(dwm_cfg_get(&cfg));
	if (err_cnt)
		return err_cnt;

	rv = 0;
	while(rv
	      || cfg_tag.low_power_en   != cfg.low_power_en
	      || cfg_tag.meas_mode      != cfg.meas_mode
	      || cfg_tag.loc_engine_en  != cfg.loc_engine_en
	      || cfg_tag.stnry_en       != cfg.stnry_en
	      || cfg_tag.common.enc_en  != cfg.common.enc_en
	      || cfg_tag.common.led_en  != cfg.common.led_en
	      || cfg_tag.common.ble_en  != cfg.common.ble_en
	      || cfg_tag.common.uwb_mode != cfg.common.uwb_mode
	      || cfg_tag.common.fw_update_en != cfg.common.fw_update_en) {
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

		printf("dwm_cfg_tag_set(&cfg_tag)\n");
		rv = Test_CheckTxRx(dwm_cfg_tag_set(&cfg_tag));
		if (rv)
			continue;
		HAL_Delay(delay_ms);

		printf("dwm_reset()\n");
		rv = Test_CheckTxRx(dwm_reset());
		if (rv)
			continue;
		HAL_Log("Wait %d ms for node to reset.\n", delay_ms);
		err_cnt += rv;
		HAL_Delay(delay_ms);

		printf("dwm_cfg_get(&cfg)\n");
		rv = Test_CheckTxRx(dwm_cfg_get(&cfg));
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

static int setup_sock(struct sockaddr_in *addr)
{
	int rc, sockfd;

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		rc = -errno;
		perror("socket creation failed");
		return rc;
	}

	memset(addr, 0, sizeof(*addr));
	addr->sin_family = AF_INET;
	addr->sin_addr.s_addr = inet_addr(UDP_GROUP);
	addr->sin_port = htons(UDP_PORT);

	return sockfd;
}

static void do_data_loop(void)
{
	int err_cnt = 0;
	uint16_t panid;
	uint16_t ur_set;
	uint16_t ur_s_set;
	int sockfd, n;

	struct sockaddr_in addr;
	struct location *loc = alloca(1024);

	sockfd = setup_sock(&addr);
	if (sockfd < 0)
		return;

	//init
	printf("Initializing...\n");
	HAL_Log("Initializing...\n");

	dwm_init();

#ifdef DEV_CONFIG
	// setup tag mode configurations
	while (setup_tag())
		;

#if DEV_SETUP_ENC
	// setup encryption key for network
	while (setup_enc())
		;
#endif

	// setup PANID
	printf(">> set panid\n");
	panid = 0xbeb2;
	while (dwm_panid_set(panid) != RV_OK)
		printf("panid_set failed\n");

	// setup tag update rate
	printf(">> set rate\n");
	ur_s_set = ur_set = UPD_RATE;
	HAL_Log("dwm_upd_rate_set(ur_set, ur_s_set);\n");
	while (dwm_upd_rate_set(ur_set, ur_s_set) != RV_OK)
		;
#endif

//	HAL_Log("dwm_int_cfg_set(DWM1001_INTR_LOC_READY);\n");
//	uint16_t intr;
//	dwm_int_cfg_set(DWM1001_INTR_LOC_READY);
//	HAL_GPIO_SetupCb(HAL_GPIO_DRDY, HAL_GPIO_INT_EDGE_RISING, &gpio_cb);



	printf("Ready for fetching the data\n");

	while (1) {
		size_t sz;

		err_cnt = get_loc(loc);
		if (!err_cnt) {
			sz = sizeof(*loc) + sizeof(loc->anchors[0]) * loc->nr_anchors;
			n = sendto(sockfd, loc, sz, MSG_CONFIRM,
				   (const struct sockaddr *)&addr, sizeof(addr));
			if (n < 0)
				printf("sendto failed: %d\n", errno);
		} else {
			printf(">> get loc failed\n");
		}

		HAL_Delay(UPD_RATE_MS);
	}

	HAL_Log("err_cnt = %d \n", err_cnt);

	Test_End();
}

int main(int argc, char*argv[])
{
	do_data_loop();
	return 0;
}
