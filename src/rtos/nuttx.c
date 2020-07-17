/***************************************************************************
 *   Copyright 2016,2017 Sony Video & Sound Products Inc.                  *
 *   Masatoshi Tateishi - Masatoshi.Tateishi@jp.sony.com                   *
 *   Masayuki Ishikawa - Masayuki.Ishikawa@jp.sony.com                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "server/gdb_server.h"

#include "nuttx_header.h"

/* tcb_s */
static int pid_offset = PID;
static int state_offset = STATE;
static int name_offset =  NAME;
static int xcpreg_offset = XCPREG;
static int name_size = NAME_SIZE;

static int g_tasklisttable_size = 0;

static int config_smp = 0;
static int config_disable_signals = 0;
static int config_disable_mqueue = 0;
static int config_paging = 0;
static int config_sig_sigstop_action = 0;

int rtos_thread_packet(struct connection *connection, const char *packet, int packet_size);

/* see nuttx/sched/os_start.c */
static char *nuttx_symbol_list[] = {
	"g_readytorun",            /* 0: must be top of this array */
	"g_tasklisttable",
	NULL
};

/* see nuttx/include/nuttx/sched.h */
struct tcb {
	uint32_t flink;
	uint32_t blink;
	uint8_t  dat[680];
};

struct g_tasklist {
	uint32_t addr;
	uint32_t prio;
};

static struct g_tasklist * g_tasklisttable;
#define NUM_TASK_STATES (g_tasklisttable_size / (sizeof(struct g_tasklist)))

static int g_tasklisttable_size;
static char* task_state_str[12];

static void init_task_state_str()
{
	int i = 0;

	task_state_str[i++] = "INVALID";
	task_state_str[i++] = "PENDING";
	task_state_str[i++] = "READYTORUN";

	if (config_smp)
		task_state_str[i++] = "TASK_ASSIGNED";

	task_state_str[i++] = "RUNNING";
	task_state_str[i++] = "INACTIVE";
	task_state_str[i++] = "WAIT_SEM";

	if (!config_disable_signals)
		task_state_str[i++] = "WAIT_SIG";

	if (!config_disable_mqueue) {
		task_state_str[i++] = "WAIT_MQNOTEMPTY";
		task_state_str[i++] = "WAIT_MQNOTFULL";
	}

	if (config_paging)
		task_state_str[i++] = "WAIT_PAGEFILL";

	if (config_sig_sigstop_action)
		task_state_str[i++] = "TASK_STOPPED";
};

/* see arch/arm/include/armv7-m/irq_cmnvector.h */
static const struct stack_register_offset nuttx_stack_offsets_cortex_m[] = {
	{ ARMV7M_R0,	0xb0, 32 },		/* r0   */
	{ ARMV7M_R1,	0xb4, 32 },		/* r1   */
	{ ARMV7M_R2,	0xb8, 32 },		/* r2   */
	{ ARMV7M_R3,	0xbc, 32 },		/* r3   */
	{ ARMV7M_R4,	0x08, 32 },		/* r4   */
	{ ARMV7M_R5,	0x0c, 32 },		/* r5   */
	{ ARMV7M_R6,	0x10, 32 },		/* r6   */
	{ ARMV7M_R7,	0x14, 32 },		/* r7   */
	{ ARMV7M_R8,	0x18, 32 },		/* r8   */
	{ ARMV7M_R9,	0x1c, 32 },		/* r9   */
	{ ARMV7M_R10,	0x20, 32 },		/* r10  */
	{ ARMV7M_R11,	0x24, 32 },		/* r11  */
	{ ARMV7M_R12,	0xc0, 32 },		/* r12  */
	{ ARMV7M_R13,	0x00, 32 },		/* sp   */
	{ ARMV7M_R14,	0xc4, 32 },		/* lr   */
	{ ARMV7M_PC,	0xc8, 32 },		/* pc   */
	{ ARMV7M_xPSR,	0xcc, 32 },		/* xPSR */
};


static const struct rtos_register_stacking nuttx_stacking_cortex_m = {
	0xd0,                                   /* stack_registers_size */
	-1,                                     /* stack_growth_direction */
	17,                                     /* num_output_registers */
	0,                                      /* stack_alignment */
	nuttx_stack_offsets_cortex_m   /* register_offsets */
};

static int rcmd_offset(const char *cmd, const char *name)
{
	if (strncmp(cmd, name, strlen(name)))
		return -1;

	if (strlen(cmd) <= strlen(name) + 1)
		return -1;

	return atoi(cmd + strlen(name));
}

static int nuttx_thread_packet(struct connection *connection,
	char const *packet, int packet_size)
{
	char cmd[GDB_BUFFER_SIZE / 2] = "";

	if (!strncmp(packet, "qRcmd", 5)) {
		size_t len = unhexify((uint8_t *)cmd, packet + 6, sizeof(cmd));
		int offset;

		if (len <= 0)
			goto pass;

		offset = rcmd_offset(cmd, "nuttx.pid_offset");

		if (offset >= 0) {
			LOG_INFO("pid_offset: %d", offset);
			pid_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.state_offset");

		if (offset >= 0) {
			LOG_INFO("state_offset: %d", offset);
			state_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.name_offset");

		if (offset >= 0) {
			LOG_INFO("name_offset: %d", offset);
			name_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.xcpreg_offset");

		if (offset >= 0) {
			LOG_INFO("xcpreg_offset: %d", offset);
			xcpreg_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.name_size");

		if (offset >= 0) {
			LOG_INFO("name_size: %d", offset);
			name_size = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.g_tasklisttable_size");

		if (offset >= 0) {
			LOG_INFO("g_tasklisttable_size: %d", offset);
			g_tasklisttable_size = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.config_smp");

		if (offset >= 0) {
			LOG_INFO("config_smp: %d", offset);
			config_smp = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.config_disable_signals");

		if (offset >= 0) {
			LOG_INFO("config_disable_signals: %d", offset);
			config_disable_signals = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.config_disable_mqueue");

		if (offset >= 0) {
			LOG_INFO("config_disable_mqueue: %d", offset);
			config_disable_mqueue = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.config_paging");

		if (offset >= 0) {
			LOG_INFO("config_paging: %d", offset);
			config_paging = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.config_sig_sigstop_action");

		if (offset >= 0) {
			LOG_INFO("config_sig_sigstop_action: %d", offset);
			config_sig_sigstop_action = offset;
			goto retok;
		}
	}
pass:
	return rtos_thread_packet(connection, packet, packet_size);
retok:
	gdb_put_packet(connection, "OK", 2);
	return ERROR_OK;
}

static bool nuttx_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
			(target->rtos->symbols[0].address != 0) &&
			(target->rtos->symbols[1].address != 0)) {
		return true;
	}
	return false;
}

static int nuttx_create(struct target *target)
{
	target->rtos->gdb_thread_packet = nuttx_thread_packet;
	LOG_INFO("target type name = %s", target->type->name);
	return 0;
}

static int nuttx_update_threads(struct rtos *rtos)
{
	uint32_t thread_count;
	struct tcb tcb;
	int ret;
	uint32_t head;
	uint32_t tcb_addr;
	uint32_t i;
	uint8_t state;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for NuttX");
		return -3;
	}

	if (g_tasklisttable_size == 0) {
		LOG_ERROR("nuttx_update_threads: Please set up nuttx.g_tasklisttable_size");
		return -3;
	}

	if (g_tasklisttable == NULL) {
		g_tasklisttable = calloc(1, g_tasklisttable_size);

		init_task_state_str();
  }

	/* free previous thread details */
	rtos_free_threadlist(rtos);

	ret = target_read_buffer(rtos->target, rtos->symbols[1].address,
		g_tasklisttable_size, (uint8_t *)g_tasklisttable);
	if (ret) {
		LOG_ERROR("target_read_buffer : ret = %d\n", ret);
		return ERROR_FAIL;
	}

	thread_count = 0;

	for (i = 0; i < NUM_TASK_STATES; i++) {

		if (g_tasklisttable[i].addr == 0 || i == 2 /* g_readytorun appears twice */)
			continue;

		ret = target_read_u32(rtos->target, g_tasklisttable[i].addr,
			&head);

		if (ret) {
			LOG_ERROR("target_read_u32 : ret = %d\n", ret);
			return ERROR_FAIL;
		}

		/* readytorun head is current thread */
		if (g_tasklisttable[i].addr == rtos->symbols[0].address)
			rtos->current_thread = head;

		tcb_addr = head;
		while (tcb_addr) {
			struct thread_detail *thread;
			ret = target_read_buffer(rtos->target, tcb_addr,
				sizeof(tcb), (uint8_t *)&tcb);
			if (ret) {
				LOG_ERROR("target_read_buffer : ret = %d\n",
					ret);
				return ERROR_FAIL;
			}
			thread_count++;

			rtos->thread_details = realloc(rtos->thread_details,
				sizeof(struct thread_detail) * thread_count);
			thread = &rtos->thread_details[thread_count - 1];
			thread->threadid = tcb_addr;
			thread->exists = true;

			state = tcb.dat[state_offset - 8];
			thread->extra_info_str = NULL;
			if (state < NUM_TASK_STATES) {
				int16_t pid = tcb.dat[pid_offset - 8] |
					tcb.dat[pid_offset - 8 + 1] << 8;
				thread->extra_info_str = malloc(256);
				snprintf(thread->extra_info_str, 256, "pid:%d, %s",
					pid, task_state_str[state]);
			}

			if (name_offset) {
				thread->thread_name_str = malloc(name_size + 1);
				snprintf(thread->thread_name_str, name_size,
				    "%s", (char *)&tcb.dat[name_offset - 8]);
				// stupid gdb hates <>
				if (!strncmp(thread->thread_name_str, "<pthread>", strlen("<pthread>")))
					snprintf(thread->thread_name_str, name_size, "%s", "pthread\0");
			} else {
				thread->thread_name_str = malloc(sizeof("None"));
				strcpy(thread->thread_name_str, "None");
			}

			tcb_addr = tcb.flink;
		}
	}
	rtos->thread_count = thread_count;

	return 0;
}


/*
 * thread_id = tcb address;
 */
static int nuttx_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
	struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	int status = rtos_generic_stack_read(rtos->target, &nuttx_stacking_cortex_m,
	    (uint32_t)thread_id + xcpreg_offset, reg_list, num_regs);

#if false
	int num_pc_reg = stacking->num_output_registers - 1 - 1;
	struct rtos_reg *pc_reg = reg_list[num_pc_reg];

	LOG_INFO("thread %lld PC: %xd", thread_id, *(uint32_t*)pc_reg->value);
#endif
	return status;
}

static int nuttx_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;

	*symbol_list = (symbol_table_elem_t *) calloc(1,
		sizeof(symbol_table_elem_t) * ARRAY_SIZE(nuttx_symbol_list));

	for (i = 0; i < ARRAY_SIZE(nuttx_symbol_list); i++)
		(*symbol_list)[i].symbol_name = nuttx_symbol_list[i];

	return 0;
}

struct rtos_type nuttx_rtos = {
	.name = "nuttx",
	.detect_rtos = nuttx_detect_rtos,
	.create = nuttx_create,
	.update_threads = nuttx_update_threads,
	.get_thread_reg_list = nuttx_get_thread_reg_list,
	.get_symbol_list_to_lookup = nuttx_get_symbol_list_to_lookup,
};

