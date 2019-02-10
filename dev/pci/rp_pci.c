/*-
 * SPDX-License-Identifier: BSD-4-Clause
 *
 * Copyright (c) Comtrol Corporation <support@comtrol.com>
 * All rights reserved.
 *
 * PCI-specific part separated from:
 * sys/i386/isa/rp.c,v 1.33 1999/09/28 11:45:27 phk Exp
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted prodived that the follwoing conditions
 * are met.
 * 1. Redistributions of source code must retain the above copyright 
 *    notive, this list of conditions and the following disclainer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials prodided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *       This product includes software developed by Comtrol Corporation.
 * 4. The name of Comtrol Corporation may not be used to endorse or 
 *    promote products derived from this software without specific 
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY COMTROL CORPORATION ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL COMTROL CORPORATION BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, LIFE OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
//XXX: __FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/fcntl.h>
#include <sys/malloc.h>
#include <sys/tty.h>
#include <sys/conf.h>
#include <sys/kernel.h>
//XXX: #include <sys/module.h>
//XXX: #include <machine/resource.h>
#include <machine/bus.h>
//XXX: #include <sys/bus.h>
//XXX: #include <sys/rman.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

#define ROCKET_C
#include <dev/ic/rpreg.h>
#include <dev/ic/rpvar.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

int rp_pci_match(struct device *, void *, void *);
void rp_pci_attach(struct device *, struct device *, void *);

struct rp_pci_softc {
	struct rp_softc		sc_rp;		/* real softc */

	bus_space_tag_t		sc_iot;		/* PLX i/o tag */
	bus_space_handle_t	sc_ioh;		/* PLX i/o handle */
};

struct cfattach rp_pci_ca = {
	sizeof(struct rp_pci_softc), rp_pci_match, rp_pci_attach
};

const struct pci_matchid rp_pci_devices[] = {
	{ PCI_VENDOR_COMCORP, PCI_PRODUCT_COMCORP_ROCKETPORT_16 },
};

/* PCI IDs  */
#define RP_VENDOR_ID		0x11FE
#define RP_DEVICE_ID_32I	0x0001
#define RP_DEVICE_ID_8I		0x0002
#define RP_DEVICE_ID_16I	0x0003
#define RP_DEVICE_ID_4Q		0x0004
#define RP_DEVICE_ID_8O		0x0005
#define RP_DEVICE_ID_8J		0x0006
#define RP_DEVICE_ID_4J		0x0007
#define RP_DEVICE_ID_6M		0x000C
#define RP_DEVICE_ID_4M		0x000D
#define RP_DEVICE_ID_UPCI_32	0x0801
#define RP_DEVICE_ID_UPCI_16	0x0803
#define RP_DEVICE_ID_UPCI_8O	0x0805

/**************************************************************************
  MUDBAC remapped for PCI
**************************************************************************/

#define _CFG_INT_PCI	0x40
#define _PCI_INT_FUNC	0x3A

#define PCI_STROB	0x2000
#define INTR_EN_PCI	0x0010

/***************************************************************************
Function: sPCIControllerEOI
Purpose:  Strobe the MUDBAC's End Of Interrupt bit.
Call:	  sPCIControllerEOI(CtlP)
	  CONTROLLER_T *CtlP; Ptr to controller structure
*/
#define sPCIControllerEOI(CtlP) rp_writeio2(CtlP, 0, _PCI_INT_FUNC, PCI_STROB)

/***************************************************************************
Function: sPCIGetControllerIntStatus
Purpose:  Get the controller interrupt status
Call:	  sPCIGetControllerIntStatus(CtlP)
	  CONTROLLER_T *CtlP; Ptr to controller structure
Return:   Byte_t: The controller interrupt status in the lower 4
			 bits.	Bits 0 through 3 represent AIOP's 0
			 through 3 respectively.  If a bit is set that
			 AIOP is interrupting.	Bits 4 through 7 will
			 always be cleared.
*/
#define sPCIGetControllerIntStatus(CTLP) ((rp_readio2(CTLP, 0, _PCI_INT_FUNC) >> 8) & 0x1f)

//XXX: static devclass_t rp_devclass;

int rp_pci_match(struct device *, void *, void *);
void rp_pci_attach(struct device *, struct device *, void *);
#ifdef notdef
static int rp_pcidetach(struct device dev);
static int rp_pcishutdown(struct device dev);
#endif /* notdef */
//XXX: static void rp_pcireleaseresource(struct rp_softc *sc);
static int sPCIInitController( struct rp_softc *sc,
			       int AiopNum,
			       int IRQNum,
			       Byte_t Frequency,
			       int PeriodicOnly,
			       int VendorDevice);
//XXX: static rp_aiop2rid_t rp_pci_aiop2rid;
//XXX: static rp_aiop2off_t rp_pci_aiop2off;
//XXX: static rp_ctlmask_t rp_pci_ctlmask;

static int rp_pci_aiop2rid(int aiop, int offset);		/* XXX */
static int rp_pci_aiop2off(int aiop, int offset);		/* XXX */
static unsigned char rp_pci_ctlmask(struct rp_softc *sc);	/* XXX */

/*
 * The following functions are the pci-specific part
 * of rp driver.
 */

int
rp_pci_match(struct device *parent, void *match, void *aux)
{
	return (pci_matchbyid((struct pci_attach_args *)aux, rp_pci_devices,
	    nitems(rp_pci_devices)));
}

void
rp_pci_attach(struct device *parent, struct device *self, void *aux)
{
	int	num_ports, num_aiops;
	int	aiop;
	struct rp_softc *sc = (struct rp_softc *)self;
	int	retval;
	pcireg_t	maptype;
	struct pci_attach_args	*pa = aux;

//XXX:	sc = device_get_softc(dev);
//XXX:	bzero(sc, sizeof(*sc));
//XXX:	sc->dev = dev;
	sc->aiop2rid = rp_pci_aiop2rid;
	sc->aiop2off = rp_pci_aiop2off;
	sc->ctlmask = rp_pci_ctlmask;

#if 0
	/* The IO ports of AIOPs for a PCI controller are continuous. */
	sc->io_num = 1;
	sc->io_rid = malloc(sizeof(*(sc->io_rid)) * sc->io_num, M_DEVBUF, M_NOWAIT | M_ZERO);
	sc->io = malloc(sizeof(*(sc->io)) * sc->io_num, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (sc->io_rid == NULL || sc->io == NULL) {
		device_printf(dev, "rp_pciattach: Out of memory.\n");
		retval = ENOMEM;
		goto nogo;
	}
#endif

	sc->bus_ctlp = NULL;

//XXX:	switch (pci_get_device(dev)) {
	switch (PCI_PRODUCT(pa->pa_id)) {
	case RP_DEVICE_ID_UPCI_16:
	case RP_DEVICE_ID_UPCI_32:
	case RP_DEVICE_ID_UPCI_8O:
//XXX:		sc->io_rid[0] = PCIR_BAR(2);
		break;
	default:
//XXX:		sc->io_rid[0] = PCIR_BAR(0);
		break;
	}

#if 0
	sc->io[0] = bus_alloc_resource_any(dev, SYS_RES_IOPORT,
		&sc->io_rid[0], RF_ACTIVE);
	if(sc->io[0] == NULL) {
		device_printf(dev, "ioaddr mapping failed for RocketPort(PCI).\n");
		retval = ENXIO;
		goto nogo;
	}
#endif

printf("%s:%d\n", __func__, __LINE__);
	maptype = pci_mapreg_type(pa->pa_pc, pa->pa_tag, RP_PCI_BAR_1);
	if (pci_mapreg_map(pa, RP_PCI_BAR_1, maptype, 0, &sc->sc_iot,
	    &sc->sc_ioh, NULL, &sc->sc_ios, 0) != 0) {
		printf(" unable to map registers\n");
		return;
	}

printf("%s:%d\n", __func__, __LINE__);
	num_aiops = sPCIInitController(sc,
				       MAX_AIOPS_PER_BOARD, 0,
//XXX:				       FREQ_DIS, 0, pci_get_device(dev));
				       FREQ_DIS, 0, PCI_PRODUCT(pa->pa_id));

printf("%s:%d\n", __func__, __LINE__);
	num_ports = 0;
	for(aiop=0; aiop < num_aiops; aiop++) {
		sResetAiopByNum(sc, aiop);
		num_ports += sGetAiopNumChan(sc, aiop);
	}

printf("%s:%d\n", __func__, __LINE__);
	retval = rp_attachcommon(sc, num_aiops, num_ports);
	if (retval != 0)
		goto nogo;

	return;

nogo:
//XXX:	rp_pcireleaseresource(sc);

	return;
}
#if 0
static int
rp_pcidetach(struct device *dev)
{
	struct rp_softc *sc;

	sc = device_get_softc(dev);
	rp_pcireleaseresource(sc);

	return (0);
}

static int
rp_pcishutdown(struct device *dev)
{
	struct rp_softc *sc;

	sc = device_get_softc(dev);
	rp_pcireleaseresource(sc);

	return (0);
}

static void
rp_pcireleaseresource(struct rp_softc *sc)
{
	rp_releaseresource(sc);
	if (sc->io != NULL) {
		if (sc->io[0] != NULL)
			bus_release_resource(sc->dev, SYS_RES_IOPORT, sc->io_rid[0], sc->io[0]);
		free(sc->io, M_DEVBUF);
		sc->io = NULL;
	}
	if (sc->io_rid != NULL) {
		free(sc->io_rid, M_DEVBUF);
		sc->io = NULL;
	}
}
#endif
static int
sPCIInitController( struct rp_softc *CtlP,
		    int AiopNum,
		    int IRQNum,
		    Byte_t Frequency,
		    int PeriodicOnly,
		    int VendorDevice)
{
	int		i;

	CtlP->CtlID = CTLID_0001;	/* controller release 1 */

	sPCIControllerEOI(CtlP);

	/* Init AIOPs */
	CtlP->NumAiop = 0;
	for(i=0; i < AiopNum; i++)
	{
		/*device_printf(CtlP->dev, "aiop %d.\n", i);*/
		CtlP->AiopID[i] = sReadAiopID(CtlP, i);	/* read AIOP ID */
		/*device_printf(CtlP->dev, "ID = %d.\n", CtlP->AiopID[i]);*/
		if(CtlP->AiopID[i] == AIOPID_NULL)	/* if AIOP does not exist */
		{
			break;				/* done looking for AIOPs */
		}

		switch( VendorDevice ) {
		case RP_DEVICE_ID_4Q:
		case RP_DEVICE_ID_4J:
		case RP_DEVICE_ID_4M:
      			CtlP->AiopNumChan[i] = 4;
			break;
		case RP_DEVICE_ID_6M:
      			CtlP->AiopNumChan[i] = 6;
			break;
		case RP_DEVICE_ID_8O:
		case RP_DEVICE_ID_8J:
		case RP_DEVICE_ID_8I:
		case RP_DEVICE_ID_16I:
		case RP_DEVICE_ID_32I:
      			CtlP->AiopNumChan[i] = 8;
			break;
		default:
#ifdef notdef
      			CtlP->AiopNumChan[i] = 8;
#else
      			CtlP->AiopNumChan[i] = sReadAiopNumChan(CtlP, i);
#endif /* notdef */
			break;
		}
		/*device_printf(CtlP->dev, "%d channels.\n", CtlP->AiopNumChan[i]);*/
		rp_writeaiop2(CtlP, i, _INDX_ADDR,_CLK_PRE);	/* clock prescaler */
		/*device_printf(CtlP->dev, "configuring clock prescaler.\n");*/
		rp_writeaiop1(CtlP, i, _INDX_DATA,CLOCK_PRESC);
		/*device_printf(CtlP->dev, "configured clock prescaler.\n");*/
		CtlP->NumAiop++;				/* bump count of AIOPs */
	}

	if(CtlP->NumAiop == 0)
		return(-1);
	else
		return(CtlP->NumAiop);
}

/*
 * ARGSUSED
 * Maps (aiop, offset) to rid.
 */
static int
rp_pci_aiop2rid(int aiop, int offset)
{
	/* Always return zero for a PCI controller. */
	return 0;
}

/*
 * ARGSUSED
 * Maps (aiop, offset) to the offset of resource.
 */
static int
rp_pci_aiop2off(int aiop, int offset)
{
	/* Each AIOP reserves 0x40 bytes. */
	return aiop * 0x40 + offset;
}

/* Read the int status for a PCI controller. */
static unsigned char
rp_pci_ctlmask(struct rp_softc *sc)
{
	return sPCIGetControllerIntStatus(sc);
}
#if 0
static device_method_t rp_pcimethods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rp_pciprobe),
	DEVMETHOD(device_attach,	rp_pciattach),
	DEVMETHOD(device_detach,	rp_pcidetach),
	DEVMETHOD(device_shutdown,	rp_pcishutdown),

	{ 0, 0 }
};

static driver_t rp_pcidriver = {
	"rp",
	rp_pcimethods,
	sizeof(struct rp_softc),
};

/*
 * rp can be attached to a pci bus.
 */
DRIVER_MODULE(rp, pci, rp_pcidriver, rp_devclass, 0, 0);
#endif
