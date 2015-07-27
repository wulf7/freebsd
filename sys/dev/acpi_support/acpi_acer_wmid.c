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

#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/pc/bios.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>
#include "acpi_wmi_if.h"

#define _COMPONENT	ACPI_OEM
ACPI_MODULE_NAME("ACER-WMI-WMID")

#define ACPI_ACER_WMID_MGMT_GUID	"61EF69EA-865C-4BC3-A502-A0DEBA0CB531"
#define ACPI_ACER_WMID_EVENT_GUID	"676AA15E-6A47-4D9F-A2CC-1E6D18D14026"

/* WMID Instances */
#define	ACER_WMID_INSTANCE_DEV		0x00
#define	ACER_WMID_INSTANCE_LM		0x01

/* WMID Methods */
#define ACER_WMID_METHOD_SET		0x00000001
#define ACER_WMID_METHOD_GET		0x00000002

/* WMID interface device ids */
#define ACER_WMID_DEVID_WLAN		0x0001
#define ACER_WMID_DEVID_TOUCHPAD	0x0002
#define ACER_WMID_DEVID_BACKLIGHT	0x0004
#define ACER_WMID_DEVID_WWAN3G		0x0040
#define ACER_WMID_DEVID_WIMAX		0x0080
#define ACER_WMID_DEVID_BLUETOOTH	0x0800

#define	ACER_WMID_EVENT_HOTKEY		0x01
#define ACER_WMID_EVENT_ACCEL		0x05

struct acer_smbios_entry {
	struct smbios_structure_header header;
	uint16_t	comm_dev_types;
	uint16_t	app_dev_types;
	uint16_t	media_dev_types;
	uint16_t	display_dev_types;
	uint16_t	other_dev_types;
	uint8_t		fn_key_code;
} __packed;

struct acpi_acer_wmid_softc {
	device_t	dev;
	device_t	wmi_dev;
	const char	*notify_guid;
	struct sysctl_ctx_list	*sysctl_ctx;
	struct sysctl_oid	*sysctl_tree;
	int		handle_keys;
	UINT16		devices;
	UINT8		fn_key;
};

static struct {
	char	*name;
	int	dev_id;
	char	*description;
} acpi_acer_wmid_sysctls[] = {
	{
		.name		= "wlan",
		.dev_id		= ACER_WMID_DEVID_WLAN,
		.description	= "WLAN power control",
	},
	{
		.name		= "bluetooth",
		.dev_id		= ACER_WMID_DEVID_BLUETOOTH,
		.description	= "Bluetooth power control",
	},
	{
		.name		= "wimax",
		.dev_id		= ACER_WMID_DEVID_WIMAX,
		.description	= "WiMAX power control",
	},
	{
		.name		= "wwan3g",
		.dev_id		= ACER_WMID_DEVID_WWAN3G,
		.description	= "WWAN-3G power control",
	},
	{ NULL, 0, NULL }
};

/* WMID interface */
struct acer_wmid_event {
	UINT8	event;
	UINT8	key_code;
	UINT16	dev_state;
	UINT32	unused;
} __packed;
/* WMID set device status parameters */
struct acpi_acer_wmid_devset_params {
	UINT8	function_id;
	UINT8	hotkey_code;
	UINT16	devices;
	UINT8	unused;
} __packed;
/* WMID get device status parameters */
struct acpi_acer_wmid_devget_params {
	UINT8	function_id;
	UINT8	hotkey_code;
	UINT16	devices;
} __packed;
/* WMID Evaluate Method return value*/
struct acpi_acer_wmid_result {
	UINT8	status;
	UINT8	ec_status;
	UINT16	retval;
	UINT32	unused;
} __packed;

ACPI_SERIAL_DECL(acer_wmid, "ACER WMI WMID device");

static void	acpi_acer_wmid_identify(driver_t *driver, device_t parent);
static int	acpi_acer_wmid_probe(device_t dev);
static int	acpi_acer_wmid_attach(device_t dev);
static int	acpi_acer_wmid_detach(device_t dev);

static int	acpi_acer_wmid_sysctl(SYSCTL_HANDLER_ARGS);
static int	acpi_acer_wmid_sysctl_set(struct acpi_acer_wmid_softc *sc,
		    int dev_id, int arg);
static int	acpi_acer_wmid_sysctl_get(struct acpi_acer_wmid_softc *sc,
		    int dev_id);
static int	acpi_acer_wmid_evaluate(device_t wmi_dev, UINT8 instance,
		    UINT32 method, ACPI_BUFFER *args, UINT16 *retval);
static int	acpi_acer_wmid_get_devstate(struct acpi_acer_wmid_softc *sc,
		    UINT16 dev_id, int *retval);
static int	acpi_acer_wmid_set_devstate(struct acpi_acer_wmid_softc *sc,
		    UINT16 dev_id, int ctrl_param);
static void	acpi_acer_wmid_notify(ACPI_HANDLE h, UINT32 notify,
		    void *context);
static void	acpi_acer_smbios_probe_cb(struct smbios_structure_header *h,
		    void *arg);

typedef void (*smbios_callback_t)(struct smbios_structure_header *, void *);

static void	smbios_probe(smbios_callback_t, void *);
static int	smbios_cksum(struct smbios_eps *);
static void	smbios_walk_table(uint8_t *, int, smbios_callback_t, void *);

static device_method_t acpi_acer_wmid_methods[] = {
	DEVMETHOD(device_identify, acpi_acer_wmid_identify),
	DEVMETHOD(device_probe, acpi_acer_wmid_probe),
	DEVMETHOD(device_attach, acpi_acer_wmid_attach),
	DEVMETHOD(device_detach, acpi_acer_wmid_detach),

	DEVMETHOD_END
};

static driver_t	acpi_acer_wmid_driver = {
	"acpi_acer_wmid",
	acpi_acer_wmid_methods,
	sizeof(struct acpi_acer_wmid_softc),
};

static devclass_t acpi_acer_wmid_devclass;

DRIVER_MODULE(acpi_acer_wmid, acpi_wmi, acpi_acer_wmid_driver,
    acpi_acer_wmid_devclass, 0, 0);
MODULE_DEPEND(acpi_acer_wmid, acpi_wmi, 1, 1, 1);
MODULE_DEPEND(acpi_acer_wmid, acpi, 1, 1, 1);

static int
acpi_acer_wmid_probe(device_t dev)
{

	if (ACPI_WMI_PROVIDES_GUID_STRING(device_get_parent(dev),
	    ACPI_ACER_WMID_MGMT_GUID))
		device_set_desc(dev, "ACER WMI WMID device");
	else
		return (EINVAL);

	return (0);
}

static int
acpi_acer_wmid_attach(device_t dev)
{
	struct acpi_acer_wmid_softc *sc;
	int dev_id, i, val;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->wmi_dev = device_get_parent(dev);
	sc->handle_keys = 1;

	/* Check management GUID. */
	if (!ACPI_WMI_PROVIDES_GUID_STRING(sc->wmi_dev,
	    ACPI_ACER_WMID_MGMT_GUID)){
		device_printf(dev,
		    "WMI device does not provide the ACER management GUID\n");
		return (EINVAL);
	}

	/* Find proper and attach to notify GUID. */
	if (ACPI_WMI_PROVIDES_GUID_STRING(sc->wmi_dev,
	    ACPI_ACER_WMID_EVENT_GUID)) {
		sc->notify_guid = ACPI_ACER_WMID_EVENT_GUID;
		if (ACPI_WMI_INSTALL_EVENT_HANDLER(sc->wmi_dev,
		    sc->notify_guid, acpi_acer_wmid_notify, dev))
			sc->notify_guid = NULL;
	} else {
		sc->notify_guid = NULL;
	}

	if (sc->notify_guid == NULL)
		device_printf(dev, "Could not install event handler!\n");

	/* Probe devices */
	smbios_probe(acpi_acer_smbios_probe_cb, sc);
	if (!sc->devices) {
		printf("Unable to detect available WMID devices\n");
		return (EINVAL);
	}
	if (bootverbose)
		device_printf(dev, "Capabilities bitmap: 0x%x\n", sc->devices);

	ACPI_SERIAL_BEGIN(acer_wmid);

	sc->sysctl_ctx = device_get_sysctl_ctx(dev);
	sc->sysctl_tree = device_get_sysctl_tree(dev);
	SYSCTL_ADD_INT(sc->sysctl_ctx,
	    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
	    "handle_keys", CTLFLAG_RW, &sc->handle_keys,
	    0, "Handle some hardware keys inside the driver");
	for (i = 0; acpi_acer_wmid_sysctls[i].name != NULL; ++i) {
		dev_id = acpi_acer_wmid_sysctls[i].dev_id;

		if (!(sc->devices & dev_id))
			continue;

		if (acpi_acer_wmid_get_devstate(sc, dev_id, &val))
			continue;

		SYSCTL_ADD_PROC(sc->sysctl_ctx,
		    SYSCTL_CHILDREN(sc->sysctl_tree), OID_AUTO,
		    acpi_acer_wmid_sysctls[i].name, CTLTYPE_INT | CTLFLAG_RW,
		    sc, i, acpi_acer_wmid_sysctl, "I",
		    acpi_acer_wmid_sysctls[i].description);
	}
	ACPI_SERIAL_END(acer_wmid);

	return (0);
}

static int
acpi_acer_wmid_detach(device_t dev)
{
	struct acpi_acer_wmid_softc *sc = device_get_softc(dev);

	ACPI_FUNCTION_TRACE((char *)(uintptr_t) __func__);

	if (sc->notify_guid)
		ACPI_WMI_REMOVE_EVENT_HANDLER(dev, sc->notify_guid);

	return (0);
}

static void
acpi_acer_wmid_identify(driver_t *driver, device_t parent)
{

	/* Don't do anything if driver is disabled. */
	if (acpi_disabled("acer_wmid"))
		return;

	/* Add only a single device instance. */
	if (device_find_child(parent, "acpi_acer_wmid", -1) != NULL)
		return;

	/* Check management GUID to see whether system is compatible. */
	if (!ACPI_WMI_PROVIDES_GUID_STRING(parent, ACPI_ACER_WMID_MGMT_GUID))
		return;

	if (BUS_ADD_CHILD(parent, 0, "acpi_acer_wmid", -1) == NULL)
		device_printf(parent, "add acpi_acer_wmid child failed\n");
}

static int
acpi_acer_wmid_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct acpi_acer_wmid_softc	*sc;
	int			arg;
	int			error = 0;
	int			function;
	int			dev_id;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);

	sc = (struct acpi_acer_wmid_softc *)oidp->oid_arg1;
	function = oidp->oid_arg2;
	dev_id = acpi_acer_wmid_sysctls[function].dev_id;

	ACPI_SERIAL_BEGIN(acer_wmid);
	arg = acpi_acer_wmid_sysctl_get(sc, dev_id);
	error = sysctl_handle_int(oidp, &arg, 0, req);
	if (!error && req->newptr != NULL)
		error = acpi_acer_wmid_sysctl_set(sc, dev_id, arg);
	ACPI_SERIAL_END(acer_wmid);

	return (error);
}

static int
acpi_acer_wmid_sysctl_get(struct acpi_acer_wmid_softc *sc, int dev_id)
{
	int	val = 0;

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	ACPI_SERIAL_ASSERT(acer_wmid);

	acpi_acer_wmid_get_devstate(sc, dev_id, &val);

	return (val);
}

static int
acpi_acer_wmid_sysctl_set(struct acpi_acer_wmid_softc *sc, int dev_id, int arg)
{

	ACPI_FUNCTION_TRACE((char *)(uintptr_t)__func__);
	ACPI_SERIAL_ASSERT(acer_wmid);

	switch(dev_id) {
	case ACER_WMID_DEVID_WLAN:
	case ACER_WMID_DEVID_WWAN3G:
	case ACER_WMID_DEVID_WIMAX:
	case ACER_WMID_DEVID_BLUETOOTH:
		if (arg < 0 || arg > 1)
			return (EINVAL);
		break;
	default:
		return (EINVAL);
	}

	acpi_acer_wmid_set_devstate(sc, dev_id, arg);
	return (0);
}

static __inline void
acpi_acer_wmid_free_buffer(ACPI_BUFFER* buf) {
	if (buf && buf->Pointer) {
		AcpiOsFree(buf->Pointer);
	}
}

static void
acpi_acer_wmid_notify(ACPI_HANDLE h, UINT32 notify, void *context)
{
	device_t dev = context;
	struct acer_wmid_event result;
	ACPI_OBJECT *obj;
	UINT8 code = 0;

	ACPI_FUNCTION_TRACE_U32((char *)(uintptr_t)__func__, notify);

	struct acpi_acer_wmid_softc *sc = device_get_softc(dev);
	ACPI_BUFFER response = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_WMI_GET_EVENT_DATA(sc->wmi_dev, notify, &response);

	obj = (ACPI_OBJECT*) response.Pointer;
	if (!obj || obj->Type != ACPI_TYPE_BUFFER) {
		acpi_acer_wmid_free_buffer(&response);
		return;
	}

	result = *((struct acer_wmid_event *)obj->Buffer.Pointer);
	acpi_acer_wmid_free_buffer(&response);

	switch (result.event) {
	case ACER_WMID_EVENT_HOTKEY:
		if (!sc->handle_keys)
			break;

		switch (result.key_code) {
		case 0x63:	/* Display backlight On/Off */
			code = (result.dev_state & ACER_WMID_DEVID_BACKLIGHT) ?
			    0x62 : 0x63;
			break;

		case 0x82:	/* Touchpad control. */
			code = (result.dev_state & ACER_WMID_DEVID_TOUCHPAD) ?
			    0x82 : 0x83;
			break;

		default:
			code = result.key_code;
			device_printf(dev, "Unknown key: 0x%02x\n", code);
		}

		/* Notify devd(8) */
		if (code)
			acpi_UserNotify("ACER", ACPI_ROOT_OBJECT, code);

		break;

	case ACER_WMID_EVENT_ACCEL:
		device_printf(dev, "Accelerometer events are not supported\n");
		break;

	default:
		device_printf(dev, "Unknown event: 0x%02x\n", result.event);
	}
}

static int
acpi_acer_wmid_evaluate(device_t wmi_dev, UINT8 instance, UINT32 method,
    ACPI_BUFFER *in, UINT16 *retval)
{
	struct acpi_acer_wmid_result result;
	ACPI_BUFFER out = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_OBJECT *obj;

	if (ACPI_FAILURE(ACPI_WMI_EVALUATE_CALL(wmi_dev,
	    ACPI_ACER_WMID_MGMT_GUID, instance, method, in, &out))) {
		acpi_acer_wmid_free_buffer(&out);
		return (-EINVAL);
	}

	obj = out.Pointer;
	if (!obj || obj->Type != ACPI_TYPE_BUFFER) {
		acpi_acer_wmid_free_buffer(&out);
		return (-EINVAL);
	}

	result = *((struct acpi_acer_wmid_result *)obj->Buffer.Pointer);
	acpi_acer_wmid_free_buffer(&out);

	if (result.status || result.ec_status) {
		device_printf(wmi_dev, "Evaluate call failed: calling "
		    "0x%02x:0x%08x returns 0x%02x:0x%02x\n", instance, method,
		    result.status, result.ec_status);
		return (-ENODEV);
	}

	if (retval != NULL)
		*retval = result.retval;

	return (0);
}

static int
acpi_acer_wmid_get_devstate(struct acpi_acer_wmid_softc *sc,
    UINT16 dev_id, int *retval)
{
	int status;
	UINT16 devices;
	struct acpi_acer_wmid_devget_params params = {
		.function_id = 0x1,
		.hotkey_code = sc->fn_key,
		.devices = dev_id,
	};
	ACPI_BUFFER in = { sizeof(params), &params };

	status = acpi_acer_wmid_evaluate(sc->wmi_dev,
	    ACER_WMID_INSTANCE_DEV, ACER_WMID_METHOD_GET, &in, &devices);

	if (status == 0)
		*retval = (devices & dev_id) != 0;

	return (status);
}

static int
acpi_acer_wmid_set_devstate(struct acpi_acer_wmid_softc *sc,
    UINT16 dev_id, int ctrl_param)
{
	struct acpi_acer_wmid_devget_params get_params;
	struct acpi_acer_wmid_devset_params set_params;
	int status;
	UINT16 devices;
	ACPI_BUFFER in;

	get_params = (struct acpi_acer_wmid_devget_params) {
		.function_id = 0x1,
		.hotkey_code = sc->fn_key,
		.devices = sc->devices,
	};
	in = (ACPI_BUFFER) { sizeof(get_params), &get_params };

	status = acpi_acer_wmid_evaluate(sc->wmi_dev,
	    ACER_WMID_INSTANCE_DEV, ACER_WMID_METHOD_GET, &in, &devices);

	if (status)
		return status;

	set_params = (struct acpi_acer_wmid_devset_params) {
		.function_id = 0x2,
		.hotkey_code = sc->fn_key,
		.devices = ctrl_param ?
		    (devices | dev_id) : (devices & ~dev_id)
	};
	in = (ACPI_BUFFER) { sizeof(set_params), &set_params };

	return (acpi_acer_wmid_evaluate(sc->wmi_dev,
	    ACER_WMID_INSTANCE_DEV, ACER_WMID_METHOD_SET, &in, NULL));
}

static void
acpi_acer_smbios_probe_cb(struct smbios_structure_header *h, void *arg)
{
	struct acer_smbios_entry *s;
	struct acpi_acer_wmid_softc *sc;

	if (h->type != 0xAA || h->length < sizeof(struct acer_smbios_entry))
		return;

	s = (struct acer_smbios_entry *)h;
	sc = arg;

	if (arg != NULL) {
		sc->devices = s->comm_dev_types &
		    (ACER_WMID_DEVID_WLAN | ACER_WMID_DEVID_WWAN3G |
		     ACER_WMID_DEVID_WIMAX | ACER_WMID_DEVID_BLUETOOTH);
		sc->fn_key = s->fn_key_code;
	}
}

static void
smbios_walk_table(uint8_t *p, int entries, smbios_callback_t cb, void *arg)
{
	struct smbios_structure_header *s;

	while (entries--) {
		s = (struct smbios_structure_header *)p;
		cb(s, arg);

		/*
		 * Look for a double-nul after the end of the
		 * formatted area of this structure.
		 */
		p += s->length;
		while (!(p[0] == 0 && p[1] == 0))
			p++;

		/*
		 * Skip over the double-nul to the start of the next
		 * structure.
		 */
		p += 2;
	}
}

/*
 * Walk the SMBIOS table looking for an type 170 entry.  If we find
 * one, return the parsed data in the passed in structure and
 * return true.  If we don't find one, return false.
 */
static void
smbios_probe(smbios_callback_t cb, void *arg)
{
	struct smbios_eps *header;
	void *table;
	u_int32_t addr;

	/* Find the SMBIOS table header. */
	addr = bios_sigsearch(SMBIOS_START, SMBIOS_SIG, SMBIOS_LEN,
			      SMBIOS_STEP, SMBIOS_OFF);
	if (addr == 0)
		return;

	/*
	 * Map the header.  We first map a fixed size to get the actual
	 * length and then map it a second time with the actual length so
	 * we can verify the checksum.
	 */
	header = pmap_mapbios(addr, sizeof(struct smbios_eps));
	table = pmap_mapbios(addr, header->length);
	pmap_unmapbios((vm_offset_t)header, sizeof(struct smbios_eps));
	header = table;
	if (smbios_cksum(header) != 0) {
		pmap_unmapbios((vm_offset_t)header, header->length);
		return;
	}

	/* Now map the actual table and walk it looking for an entry. */
	table = pmap_mapbios(header->structure_table_address,
	    header->structure_table_length);
	smbios_walk_table(table, header->number_structures, cb, arg);

	/* Unmap everything. */
	pmap_unmapbios((vm_offset_t)table, header->structure_table_length);
	pmap_unmapbios((vm_offset_t)header, header->length);
}

static int
smbios_cksum(struct smbios_eps *e)
{
	u_int8_t *ptr;
	u_int8_t cksum;
	int i;

	ptr = (u_int8_t *)e;
	cksum = 0;
	for (i = 0; i < e->length; i++) {
		cksum += ptr[i];
	}

	return (cksum);
}
