# $FreeBSD$

.PATH:	${.CURDIR}/../../../dev/acpi_support

KMOD=	acpi_acer_wmid
CFLAGS+=-I${.CURDIR}/../../../dev/acpi_support
SRCS=	acpi_acer_wmid.c opt_acpi.h acpi_if.h acpi_wmi_if.h device_if.h \
	bus_if.h

.include <bsd.kmod.mk>
