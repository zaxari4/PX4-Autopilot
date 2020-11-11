/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/init.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform/cpuload.h>
#include <uORB/uORB.h>

#include <fcntl.h>

#if defined(GPIO_OTGFS_VBUS) && defined(CONFIG_SYSTEM_CDCACM)
__BEGIN_DECLS
#include <nuttx/wqueue.h>
#include <builtin/builtin.h>

static struct work_s usb_serial_work;

extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);

static void mavlink_usb_start(void *arg)
{
	if (sercon_main(0, NULL) == EXIT_SUCCESS) {
		sched_lock();
		static const char *mavlink_start_argv[] {"mavlink", "start", "-d", "/dev/ttyACM0", NULL};
		exec_builtin("mavlink", (char **)mavlink_start_argv, NULL, 0);
		sched_unlock();
	}
}

static void usb_serial_disconnect(void *arg)
{
	serdis_main(0, NULL);
}

static void mavlink_usb_stop(void *arg)
{
	sched_lock();
	static const char *mavlink_stop_argv[] {"mavlink", "stop", "-d", "/dev/ttyACM0", NULL};
	exec_builtin("mavlink", (char **)mavlink_stop_argv, NULL, 0);
	sched_unlock();

	// serial disconnect
	work_queue(HPWORK, &usb_serial_work, usb_serial_disconnect, NULL, USEC2TICK(200000));
}

static int usb_otgfs_vbus_event(int irq, void *context, void *arg)
{
	int value = px4_arch_gpioread(GPIO_OTGFS_VBUS);

	if (value == 1) {
		// USB connected, start sercon immediately, then mavlink
		work_queue(HPWORK, &usb_serial_work, mavlink_usb_start, NULL, USEC2TICK(200000));

	} else if (value == 0) {
		// USB disconnected, stop mavlink USB immediately, then serdis
		work_queue(HPWORK, &usb_serial_work, mavlink_usb_stop, NULL, 0);
	}

	return 0;
}
__END_DECLS
#endif // GPIO_OTGFS_VBUS && CONFIG_SYSTEM_CDCACM

int px4_platform_console_init(void)
{
#if !defined(CONFIG_DEV_CONSOLE) && defined(CONFIG_DEV_NULL)

	/* Support running nsh on a board with out a console
	 * Without this the assumption that the fd 0..2 are
	 * std{in..err} will be wrong. NSH will read/write to the
	 * fd it opens for the init script or nested scripts assigned
	 * to fd 0..2.
	 *
	 */

	int fd = open("/dev/null", O_RDWR);

	if (fd == 0) {
		/* Successfully opened /dev/null as stdin (fd == 0) */

		(void)fs_dupfd2(0, 1);
		(void)fs_dupfd2(0, 2);
		(void)fs_fdopen(0, O_RDONLY,         NULL, NULL);
		(void)fs_fdopen(1, O_WROK | O_CREAT, NULL, NULL);
		(void)fs_fdopen(2, O_WROK | O_CREAT, NULL, NULL);

	} else {
		/* We failed to open /dev/null OR for some reason, we opened
		 * it and got some file descriptor other than 0.
		 */

		if (fd > 0) {
			(void)close(fd);
		}

		return -ENFILE;

	}

#endif
	return OK;
}

int px4_platform_init(void)
{
	int ret = px4_platform_console_init();

	if (ret < 0) {
		return ret;
	}

	ret = px4_console_buffer_init();

	if (ret < 0) {
		return ret;
	}

	// replace stdout with our buffered console
	int fd_buf = open(CONSOLE_BUFFER_DEVICE, O_WRONLY);

	if (fd_buf >= 0) {
		dup2(fd_buf, 1);
		// keep stderr(2) untouched: the buffered console will use it to output to the original console
		close(fd_buf);
	}

	hrt_init();

	param_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif

	px4::WorkQueueManagerStart();

	uorb_start();

	px4_log_initialize();

#if defined(GPIO_OTGFS_VBUS) && defined(CONFIG_SYSTEM_CDCACM)
	px4_arch_gpiosetevent(GPIO_OTGFS_VBUS, true, true, true, usb_otgfs_vbus_event, NULL);

	// manually start if already connected
	if (px4_arch_gpioread(GPIO_OTGFS_VBUS) == 1) {
		work_queue(HPWORK, &usb_serial_work, mavlink_usb_start, NULL, USEC2TICK(400000));
	}

#endif // GPIO_OTGFS_VBUS && CONFIG_SYSTEM_CDCACM

	return PX4_OK;
}

int px4_platform_configure(void)
{
	return px4_mft_configure(board_get_manifest());

}
