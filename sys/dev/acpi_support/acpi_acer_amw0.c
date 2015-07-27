/*-
 * Copyright (c) 2015 Vladimir Kondratyev <wulf@cicgroup.ru>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * Driver for extra ACPI-controlled features found on ACER laptops that use
 * a WMI enabled BIOS. Allows to control and read status of integrated hardware
 * Inspired by the acer-wmi driver, which implements a subset of these
 * features (hotkeys) on Linux.
 *
 * acer-wmi for Linux:
 *     http://www.kernel.org
 * WMI and ACPI:
 *     http://www.microsoft.com/whdc/system/pnppwr/wmi/wmi-acpi.mspx
 */

#include "opt_acpi.h"
#include <sys/param.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/proc.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/sbuf.h>
#include <sys/module.h>
#include <sys/sysctl.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>
#include "acpi_wmi_if.h"

#define _COMPONENT	ACPI_OEM
ACPI_MODULE_NAME("ACER-WMI-AMW0")

#define ACPI_ACER_AMW0_MGMT_GUID	"67C3371D-95A3-4C37-BB61-DD47B491DAAB"
#define ACPI_ACER_AMW0_EVENT_GUID	"40D1BF71-A82D-4E59-A168-3985E03B2E87"

/* Magic Number required for writing to ACPI for AMW0 */
#define ACER_AMW0_WRITE_MAGIC		0x9610
/* Lowest bytes for the AMW0 interface */
#define ACER_AMW0_LB_WLAN		0x35
#define ACER_AMW0_LB_BLUETOOTH		0x34
#define ACER_AMW0_LB_MAILLED		0x31

/* AMW0 interface capability bits */
#define ACER_AMW0_DEVID_WLAN		0x0001
#define ACER_AMW0_DEVID_BLUETOOTH	0x0002
#define ACER_AMW0_DEVID_MAILLED		0x0004
#define ACER_AMW0_DEVID_BRIGHTNESS	0x0008

struct acpi_acer_amw0_softc {
	device_t	dev;
	device_t	wmi_dev;
	device_t	ec_dev;
	const char	*notify_guid;
	struct sysctl_ctx_list	*sysctl_ctx;
	struct sysctl_oid	*sysctl_tree;
	int		handle_keys;
	int		mailled;
	int		brightness;
	UINT16		devices;
};

static struct {
	char	*name;
	int	dev_id;
	char	*description;
} acpi_acer_amw0_sysctls[] = {
	{
		.name		= "wlan",
		.dev_id		= ACER_AMW0_DEVID_WLAN,
		.description	= "WLAN power control",
	},
	{
		.name		= "bluetooth",
		.dev_id		= ACER_AMW0_DEVID_BLUETOOTH,
		.description	= "Bluetooth power control",
	},
	{
		.name		= "mail_led",
		.dev_id		= ACER_AMW0_DEVID_MAILLED,
		.description	= "Mail LED control",
	},
	{
		.name		= "brightness",
		.dev_id		= ACER_AMW0_DEVID_BRIGHTNESS,
		.description	= "LCD backlight brightness control",
	},
	{ NULL, 0, NULL }
};

/* AMW0 interface */
struct acpi_acer_amw0_params {
	UINT32 eax;
	UINT32 ebx;
	UINT32 ecx;
	UINT32 edx;
};
struct acpi_acer_amw0_result {
	UINT32 eax;
	UINT32 ebx;
	UINT32 ecx;
	UINT32 edx;
	UINT32 flg;
};

ACPI_SERIAL_DECL(acer_amw0, "ACER WMI AMW0 device");

static void	acpi_acer_amw0_identify(driver_t *driver, device_t parent);
static int	acpi_acer_amw0_probe(device_t dev);
static int	acpi_acer_amw0_attach(device_t dev);
static int	acpi_acer_amw0_detach(device_t dev);
static int	acpi_acer_amw0_suspend(device_t dev);
static int	acpi_acer_amw0_resume(device_t dev);

static int	acpi_acer_amw0_probe_devices(struct acpi_acer_amw0_softc *sc);
static int	acpi_acer_amw0_sysctl(SYSCTL_HANDLER_ARGS);
static int	acpi_acer_amw0_sysctl_set(struct acpi_acer_amw0_softc *sc,
		    int dev_id, int arg);
static int	acpi_acer_amw0_sysctl_get(struct acpi_acer_amw0_softc *sc,
		    int dev_id);
static int	acpi_acer_amw0_evaluate(device_t wmi_dev,
		    struct acpi_acer_amw0_params *params,
		    struct acpi_acer_amw0_result *result);
static int	acpi_acer_amw0_get_devstate(struct acpi_acer_amw0_softc *sc,
		    UINT16 dev_id, int *retval);
static int	acpi_acer_amw0_set_devstate(struct acpi_acer_amw0_softc *sc,
		    UINT16 dev_id, int ctrl_param);
static void	acpi_acer_amw0_notify(ACPI_HANDLE h, UINT32 notify,
		    void *context);

static device_method_t acpi_acer_amw0_methods[] = {
	DEVMETHOD(device_identify, acpi_acer_amw0_identify),
	DEVMETHOD(device_probe, acpi_acer_amw0_probe),
	DEVMETHOD(device_attach, acpi_acer_amw0_attach),
	DEVMETHOD(device_detach, acpi_acer_amw0_detach),
	DEVMETHOD(device_suspend, acpi_acer_amw0_suspend),
	DEVMETHOD(device_resume, acpi_acer_amw0_resume),

	DEVMETHOD_END
};

static driver_t	acpi_acer_amw0_driver = {
	"acpi_acer_amw0",
	acpi_acer_amw0_methods,
	sizeof(struct acpi_acer_amw0_softc),
};

static devclass_t acpi_acer_amw0_devclass;

DRIVER_MODULE(acpi_acer_amw0, acpi_wmi, acpi_acer_amw0_driver,
    acpi_acer_amw0_devclass, 0, 0);
MODULE_DEPEND(acpi_acer_amw0, acpi_wmi, 1, 1, 1);
MODULE_DEPEND(acpi_acer_amw0, acpi, 1, 1, 1);

static int
acpi_acer_amw0_probe(device_t dev)
{

	if (ACPI_WMI_PROVIDES_GUID_STRING(device_get_parent(dev),
	    ACPI_ACER_AMW0_MGMT_GUID))
		device_set_desc(dev, "ACER WMI AMW0 device");
	else
		return (EINVAL);

	return (0);
}

static int
acpi_acer_amw0_attach(device_t dev)
{
	struct acpi_acer_amw0_softc *sc;
	int dev_id, i, val;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->wmi_dev = device_get_parent(dev);
	sc->handle_keys = 1;

	/* Check management GUID. */
	if (!ACPI_WMI_PROVIDES_GUID_STRING(sc->wmi_dev,
	    ACPI_ACER_AMW0_MGMT_GUID)) {
		device_printf(dev,
		    "WMI device does not provide the ACER management GUID\n");
		return (EINVAL);
	}

	/* Find proper and attach to notify GUID. */
	if (ACPI_WMI_PROVIDES_GUID_STRING(sc->wmi_dev,
	    ACPI_ACER_AMW0_EVENT_GUID)) {
		sc->notify_guid = ACPI_ACER_AMW0_EVENT_GUID;
		if (ACPI_WMI_INSTALL_EVENT_HANDLER(sc->wmi_dev,
		    sc->notify_guid, acpi_acer_amw0_notify, dev))
			sc->notify_guid = NULL;
	} else {
		sc->notify_guid = NULL;
	}

	if (sc->notify_guid == NULL)
		device_printf(dev, "Could not install event handler!\n");

	/* XXX: Only works with first EC */
	if ((sc->ec_dev = devclass_get_device(devclass_find("acpi_ec"), 0))
	    == NULL)
		device_printf(dev, "cannot find EC device\n");

	/* Initialize. */
	/* Probe devices */
	if (acpi_acer_amw0_probe_devices(sc)) {
		printf("Unable to detect available AMW0 devices\n");
		return (EINVAL);
	}

	if (bootverbose)
		device_printf(dev, "Capabilities bitmap: 0x%x\n", sc->devices);

	ACPI_SERIAL_BEGIN(acer_amw0);

	sc->sysctl_ctx = device_get_sysctl_ctx(dev);
	sc->sysctl_tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_INT(sc->sysctl_ctx,
	    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
	    "handle_keys", CTLFLAG_RW, &sc->handle_keys,
	    0, "Handle some hardware keys inside the driver");
	for (i = 0; acpi_acer_amw0_sysctls[i].name != NULL; ++i) {
		dev_id = acpi_acer_amw0_sysctls[i].dev_id;

		if (!(sc->devices & dev_id))
			continue;

		if (acpi_acer_amw0_get_devstate(sc, dev_id, &val))
			continue;

		switch (dev_id) {
		case ACER_AMW0_DEVID_BRIGHTNESS:
			if (val == 0)
				continue;
			break;
		}

		SYSCTL_ADD_PROC(sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    acpi_acer_amw0_sysctls[i].name, CTLTYPE_INT | CTLFLAG_RW,
		    sc, i, acpi_acer_amw0_sysctl, "I",
		    acpi_acer_amw0_sysctls[i].description);
	}
	ACPI_SERIAL_END(acer_amw0);

	return (0);
}

static int
acpi_acer_amw0_detach(device_t dev)
{
	struct acpi_acer_amw0_softc *sc = device_get_softc(dev);

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	if (sc->notify_guid)
		ACPI_WMI_REMOVE_EVENT_HANDLER(dev, sc->notify_guid);

	return (0);
}

static int
acpi_acer_amw0_suspend(device_t dev)
{
	struct acpi_acer_amw0_softc *sc = device_get_softc(dev);

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	if (sc->devices & ACER_AMW0_DEVID_MAILLED) {
		acpi_acer_amw0_get_devstate(sc, ACER_AMW0_DEVID_MAILLED,
		    &sc->mailled);
		acpi_acer_amw0_set_devstate(sc, ACER_AMW0_DEVID_MAILLED, 0);
	}

	if (sc->devices & ACER_AMW0_DEVID_BRIGHTNESS) {
		acpi_acer_amw0_get_devstate(sc, ACER_AMW0_DEVID_BRIGHTNESS,
		    &sc->brightness);
	}

	return (0);
}

static int
acpi_acer_amw0_resume(device_t dev)
{
	struct acpi_acer_amw0_softc *sc = device_get_softc(dev);

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	if (sc->devices & ACER_AMW0_DEVID_MAILLED)
		acpi_acer_amw0_set_devstate(sc, ACER_AMW0_DEVID_MAILLED,
		    sc->mailled);
	if (sc->devices & ACER_AMW0_DEVID_BRIGHTNESS)
		acpi_acer_amw0_set_devstate(sc, ACER_AMW0_DEVID_BRIGHTNESS,
		    sc->brightness);

	return (0);
}

static int
acpi_acer_amw0_probe_devices(struct acpi_acer_amw0_softc *sc)
{
	struct acpi_acer_amw0_params params;
	struct acpi_acer_amw0_result result;
	int status;

	params = (struct acpi_acer_amw0_params) {
		.eax = ACER_AMW0_WRITE_MAGIC,
		.ebx = 0xa200 | ACER_AMW0_LB_WLAN,
		.ecx = 0,
		.edx = 0
	};

	status = acpi_acer_amw0_evaluate(sc->wmi_dev, &params, &result);
	if (status)
		return (status);

	if (result.eax & 0x1)
		sc->devices |= ACER_AMW0_DEVID_WLAN;

	params.ebx = 0x0200 | ACER_AMW0_LB_BLUETOOTH;

	status = acpi_acer_amw0_evaluate(sc->wmi_dev, &params, &result);
	if (status)
		return (status);

	if (result.eax & 0x1)
		sc->devices |= ACER_AMW0_DEVID_BLUETOOTH;

	params = (struct acpi_acer_amw0_params) {
		.eax = 0x86,
		.ebx = 0,
		.ecx = 0,
		.edx = 0
	};

	status = acpi_acer_amw0_evaluate(sc->wmi_dev, &params, &result);
	if (status)
		return (status);

	if (result.flg & 0x1)
		sc->devices |= ACER_AMW0_DEVID_MAILLED;

	/*
	 * Accordind to comments in Linux acer_wmi.c enabling of brightness
	 * control is safe since all Wistron based laptops use the same
	 * EC register for brightness
	 */
	sc->devices |= ACER_AMW0_DEVID_BRIGHTNESS;

	return (0);
}

static void
acpi_acer_amw0_identify(driver_t *driver, device_t parent)
{

	/* Don't do anything if driver is disabled. */
	if (acpi_disabled("acer_amw0"))
		return;

	/* Add only a single device instance. */
	if (device_find_child(parent, "acpi_acer_amw0", -1) != NULL)
		return;

	/* Check management GUID to see whether system is compatible. */
	if (!ACPI_WMI_PROVIDES_GUID_STRING(parent, ACPI_ACER_AMW0_MGMT_GUID))
		return;

	if (BUS_ADD_CHILD(parent, 0, "acpi_acer_amw0", -1) == NULL)
		device_printf(parent, "add acpi_acer_amw0 child failed\n");
}

static int
acpi_acer_amw0_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct acpi_acer_amw0_softc	*sc;
	int			arg;
	int			error = 0;
	int			function;
	int			dev_id;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);

	sc = (struct acpi_acer_amw0_softc *)oidp->oid_arg1;
	function = oidp->oid_arg2;
	dev_id = acpi_acer_amw0_sysctls[function].dev_id;

	ACPI_SERIAL_BEGIN(acer_amw0);
	arg = acpi_acer_amw0_sysctl_get(sc, dev_id);
	error = sysctl_handle_int(oidp, &arg, 0, req);
	if (!error && req->newptr != NULL)
		error = acpi_acer_amw0_sysctl_set(sc, dev_id, arg);
	ACPI_SERIAL_END(acer_amw0);

	return (error);
}

static int
acpi_acer_amw0_sysctl_get(struct acpi_acer_amw0_softc *sc, int dev_id)
{
	int	val = 0;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	ACPI_SERIAL_ASSERT(acer_amw0);

	acpi_acer_amw0_get_devstate(sc, dev_id, &val);

	return (val);
}

static int
acpi_acer_amw0_sysctl_set(struct acpi_acer_amw0_softc *sc, int dev_id, int arg)
{

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	ACPI_SERIAL_ASSERT(acer_amw0);

	switch(dev_id) {
	case ACER_AMW0_DEVID_MAILLED:
	case ACER_AMW0_DEVID_WLAN:
	case ACER_AMW0_DEVID_BLUETOOTH:
		if (arg < 0 || arg > 1)
			return (EINVAL);
		break;
	case ACER_AMW0_DEVID_BRIGHTNESS:
		if (arg < 0 || arg > 0xF)
			return (EINVAL);
		break;
	}

	acpi_acer_amw0_set_devstate(sc, dev_id, arg);
	return (0);
}

static __inline void
acpi_acer_amw0_free_buffer(ACPI_BUFFER* buf) {
	if (buf && buf->Pointer) {
		AcpiOsFree(buf->Pointer);
	}
}

static void
acpi_acer_amw0_notify(ACPI_HANDLE h, UINT32 notify, void *context)
{
	device_t dev = context;
	ACPI_OBJECT *obj;
	UINT8 code = 0;

	ACPI_FUNCTION_TRACE_U32((char *)(uintptr_t)__func__, notify);

	struct acpi_acer_amw0_softc *sc = device_get_softc(dev);
	ACPI_BUFFER response = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_WMI_GET_EVENT_DATA(sc->wmi_dev, notify, &response);

	obj = (ACPI_OBJECT*) response.Pointer;
	if (obj && obj->Type == ACPI_TYPE_BUFFER) {
		code = *((UINT8 *)obj->Buffer.Pointer);
		acpi_UserNotify("ACER", ACPI_ROOT_OBJECT, code);
	}

	acpi_acer_amw0_free_buffer(&response);
}

static int
acpi_acer_amw0_evaluate(device_t wmi_dev,
    struct acpi_acer_amw0_params *params, struct acpi_acer_amw0_result *result)
{
	ACPI_BUFFER in = { sizeof(*params), params };
	ACPI_BUFFER out = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_OBJECT *obj;

	if (ACPI_FAILURE(ACPI_WMI_EVALUATE_CALL(wmi_dev,
	    ACPI_ACER_AMW0_MGMT_GUID, 1, 1, &in, &out))) {
		acpi_acer_amw0_free_buffer(&out);
		return (-EINVAL);
	}

	obj = out.Pointer;
	if (!obj || obj->Type != ACPI_TYPE_BUFFER) {
		acpi_acer_amw0_free_buffer(&out);
		return (-EINVAL);
	}

	if (result != NULL)
		*result =
		    *((struct acpi_acer_amw0_result *) obj->Buffer.Pointer);

	acpi_acer_amw0_free_buffer(&out);

	return (0);
}

static int
acpi_acer_amw0_get_devstate(struct acpi_acer_amw0_softc *sc,
    UINT16 dev_id, int *retval)
{
	UINT8 ec_addr;
	UINT64 ec_data;
	int shift;
	UINT16 mask;

	if (sc->ec_dev == NULL)
		return (-EINVAL);

	switch (dev_id) {
	case ACER_AMW0_DEVID_WLAN:
		ec_addr = 0x0A;
		shift = 2;
		mask = 0x01;
		break;

	case ACER_AMW0_DEVID_BLUETOOTH:
		ec_addr = 0x0A;
		shift = 4;
		mask = 0x01;
		break;

	case ACER_AMW0_DEVID_MAILLED:
		ec_addr = 0x0A;
		shift = 7;
		mask = 0x01;
		break;

	case ACER_AMW0_DEVID_BRIGHTNESS:
		ec_addr = 0x83;
		shift = 0;
		mask = 0xFFFF;
		break;

	default:
		return (-EINVAL);
	}

	if (ACPI_FAILURE(ACPI_EC_READ(sc->ec_dev, ec_addr, &ec_data, 1)))
		return (-EINVAL);
	*retval = (ec_data >> shift) & mask;
	return (0);
}

static int
acpi_acer_amw0_set_devstate(struct acpi_acer_amw0_softc *sc,
    UINT16 dev_id, int ctrl_param)
{
	struct acpi_acer_amw0_params params = {
		.eax = ACER_AMW0_WRITE_MAGIC,
		.ebx = ctrl_param ? (1<<8) : 0,
		.ecx = 0,
		.edx = 0
	};

	switch (dev_id) {
	case ACER_AMW0_DEVID_WLAN:
		params.ebx |= ACER_AMW0_LB_WLAN;
		break;

	case ACER_AMW0_DEVID_BLUETOOTH:
		params.ebx |= ACER_AMW0_LB_BLUETOOTH;
		break;

	case ACER_AMW0_DEVID_MAILLED:
		params.ebx |= ACER_AMW0_LB_MAILLED;
		break;

	case ACER_AMW0_DEVID_BRIGHTNESS:
		if (sc->ec_dev == NULL)
			return (-EINVAL);
		if (ACPI_FAILURE(ACPI_EC_WRITE(sc->ec_dev, 0x83, ctrl_param,
		    1)))
			return (-EINVAL);
		break;
	default:
		return (-EINVAL);
	}

	return (acpi_acer_amw0_evaluate(sc->wmi_dev, &params, NULL));
}
