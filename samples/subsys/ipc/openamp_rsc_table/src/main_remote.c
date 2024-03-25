/*
 * Copyright (c) 2020, STMICROELECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/mbox.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#ifdef CONFIG_SHELL_BACKEND_RPMSG
#include <zephyr/shell/shell_rpmsg.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_DBG);

#define SHM_DEVICE_NAME	"shm"

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE		DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR		DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE		DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (1024)

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_rp__client_stack, APP_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_rp__client_data;

static const struct mbox_dt_spec mbox_dev_tx_dt = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), tx);
static const struct mbox_dt_spec mbox_dev_rx_dt = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), rx);

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;

static struct metal_io_region shm_io_data; /* shared memory */
static struct metal_io_region rsc_io_data; /* rsc_table memory */

struct rpmsg_rcv_msg {
	void *data;
	size_t len;
};

static struct metal_io_region *shm_io = &shm_io_data;

static struct metal_io_region *rsc_io = &rsc_io_data;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;
static struct rpmsg_device *rpdev;

static char rx_sc_msg[20];  /* should receive "Hello world!" */
static struct rpmsg_endpoint sc_ept;
static struct rpmsg_rcv_msg sc_msg = {.data = rx_sc_msg};


static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_sc_sem, 0, 1);

static void platform_mbox_callback(const struct device *dev, uint32_t channel,
				  void *user_data, struct mbox_msg *msg)
{
	LOG_DBG("msg received from channel %d, size: %d, content: %x", channel, msg->size, *(uint32_t *)msg->data);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_cs_callback(struct rpmsg_endpoint *ept, void *data,
				  size_t len, uint32_t src, void *priv)
{
	memcpy(sc_msg.data, data, len);
	sc_msg.len = len;
	k_sem_give(&data_sc_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	int status = k_sem_take(&data_sem, K_FOREVER);

	if (status == 0) {
		rproc_virtio_notified(rvdev.vdev, VRING1_ID);
	}
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	LOG_ERR("%s: unexpected ns service receive for name %s",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	LOG_DBG("%s: msg received", __func__);
	mbox_send(mbox_dev_tx_dt.dev, mbox_dev_tx_dt.channel_id, NULL);

	return 0;
}

int platform_init(void)
{
	void *rsc_tab_addr;
	int rsc_size;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_ERR("metal_init: failed: %d", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(shm_io, (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);
	LOG_DBG("SHM_START_ADDR: 0x%llx, SHM_SIZE: %u kB", SHM_START_ADDR, SHM_SIZE);

	/* declare resource table region */
	rsc_table_get(&rsc_tab_addr, &rsc_size);
	rsc_table = (struct st_resource_table *)rsc_tab_addr;

	LOG_DBG("rsc_tab_addr: %p, rsc_size: %u kB", rsc_tab_addr, rsc_size);

	metal_io_init(rsc_io, rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);

	/* setup MBOX device */
	if (!device_is_ready(mbox_dev_tx_dt.dev) || !device_is_ready(mbox_dev_rx_dt.dev)) {
		LOG_ERR("MBOX device is not ready\n");
		return -1;
	}

	// TODO tx callback not used atm
	// mbox_register_callback(mbox_dev_tx_dt.dev, mbox_dev_tx_dt.channel_id, platform_mbox_callback, NULL);
	status = mbox_register_callback(mbox_dev_rx_dt.dev, mbox_dev_rx_dt.channel_id, platform_mbox_callback, NULL);
	if (status < 0) {
		LOG_ERR("Could not register callback (%d)\n", status);
		return -1;
	}

	// TODO tx irq not used atm
	// status = mbox_set_enabled(mbox_dev_tx_dt.dev, mbox_dev_tx_dt.channel_id, 1);
	status = mbox_set_enabled(mbox_dev_rx_dt.dev, mbox_dev_rx_dt.channel_id, 1);
	if (status) {
		LOG_ERR("mbox_set_enabled failed (%d)", status);
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	// TODO tx irq not used atm
	// mbox_set_enabled(mbox_dev_tx_dt.dev, mbox_dev_tx_dt.channel_id, 0);
	mbox_set_enabled(mbox_dev_rx_dt.dev, mbox_dev_rx_dt.channel_id, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}

	ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, NULL);
	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

void app_rpmsg_client_sample(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned int msg_cnt = 0;
	int ret = 0;

	k_sem_take(&data_sc_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux sample client responder started");

	ret = rpmsg_create_ept(&sc_ept, rpdev, "rpmsg-client-sample",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_cs_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux sample client] Could not create endpoint: %d", ret);
		goto task_end;
	}

	while (msg_cnt < 100) {
		k_sem_take(&data_sc_sem,  K_FOREVER);
		msg_cnt++;
		LOG_INF("[Linux sample client] incoming msg %d: %.*s", msg_cnt, sc_msg.len,
			(char *)sc_msg.data);
		rpmsg_send(&sc_ept, sc_msg.data, sc_msg.len);
	}
	rpmsg_destroy_ept(&sc_ept);

task_end:
	LOG_INF("OpenAMP Linux sample client responder ended");
}

void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char *msg;
	unsigned int len;
	int ret = 0;

	LOG_INF("OpenAMP[remote] Linux responder demo started");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform");
		ret = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DEVICE, NULL,
					   new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device");
		ret = -1;
		goto task_end;
	}

// #ifdef CONFIG_SHELL_BACKEND_RPMSG
// 	(void)shell_backend_rpmsg_init_transport(rpdev);
// #endif

	/* start the rpmsg clients */
	k_sem_give(&data_sc_sem);

	while (1) {
		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	LOG_INF("OpenAMP demo ended");
}

int main(void)
{
	LOG_DBG("Hello Zephyr MBOX! board: %s, build time: %s%s", CONFIG_BOARD_TARGET, __DATE__, __TIME__);
	LOG_INF("Starting application threads!");
	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
			rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_create(&thread_rp__client_data, thread_rp__client_stack, APP_TASK_STACK_SIZE,
			app_rpmsg_client_sample,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	return 0;
}
