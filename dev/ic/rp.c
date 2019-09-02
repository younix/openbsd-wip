/* $OpenBSD$ */
/*-
 * SPDX-License-Identifier: BSD-4-Clause
 *
 * Copyright (c) Comtrol Corporation <support@comtrol.com>
 * All rights reserved.
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
 *
 */

#include <sys/cdefs.h>

/* 
 * rp.c - for RocketPort FreeBSD
 */

#include <sys/types.h>
#include <sys/atomic.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/fcntl.h>
#include <sys/malloc.h>
#include <sys/tty.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <machine/bus.h>

#define ROCKET_C
#include <dev/ic/rpreg.h>
#include <dev/ic/rpvar.h>

static const char RP_Version[] = "3.02";

static uint8_t RData[RDATASIZE] = {
	0x00, 0x09, 0xf6, 0x82,
	0x02, 0x09, 0x86, 0xfb,
	0x04, 0x09, 0x00, 0x0a,
	0x06, 0x09, 0x01, 0x0a,
	0x08, 0x09, 0x8a, 0x13,
	0x0a, 0x09, 0xc5, 0x11,
	0x0c, 0x09, 0x86, 0x85,
	0x0e, 0x09, 0x20, 0x0a,
	0x10, 0x09, 0x21, 0x0a,
	0x12, 0x09, 0x41, 0xff,
	0x14, 0x09, 0x82, 0x00,
	0x16, 0x09, 0x82, 0x7b,
	0x18, 0x09, 0x8a, 0x7d,
	0x1a, 0x09, 0x88, 0x81,
	0x1c, 0x09, 0x86, 0x7a,
	0x1e, 0x09, 0x84, 0x81,
	0x20, 0x09, 0x82, 0x7c,
	0x22, 0x09, 0x0a, 0x0a
};

static uint8_t RRegData[RREGDATASIZE]= {
	0x00, 0x09, 0xf6, 0x82,	/* 00: Stop Rx processor */
	0x08, 0x09, 0x8a, 0x13,	/* 04: Tx software flow control */
	0x0a, 0x09, 0xc5, 0x11,	/* 08: XON char */
	0x0c, 0x09, 0x86, 0x85,	/* 0c: XANY */
	0x12, 0x09, 0x41, 0xff,	/* 10: Rx mask char */
	0x14, 0x09, 0x82, 0x00,	/* 14: Compare/Ignore #0 */
	0x16, 0x09, 0x82, 0x7b,	/* 18: Compare #1 */
	0x18, 0x09, 0x8a, 0x7d,	/* 1c: Compare #2 */
	0x1a, 0x09, 0x88, 0x81,	/* 20: Interrupt #1 */
	0x1c, 0x09, 0x86, 0x7a,	/* 24: Ignore/Replace #1 */
	0x1e, 0x09, 0x84, 0x81,	/* 28: Interrupt #2 */
	0x20, 0x09, 0x82, 0x7c,	/* 2c: Ignore/Replace #2 */
	0x22, 0x09, 0x0a, 0x0a	/* 30: Rx FIFO Enable */
};

#if 0
/* IRQ number to MUDBAC register 2 mapping */
uint8_t sIRQMap[16] =
{
   0,0,0,0x10,0x20,0x30,0,0,0,0x40,0x50,0x60,0x70,0,0,0x80
};
#endif

uint8_t rp_sBitMapClrTbl[8] = {0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};
uint8_t rp_sBitMapSetTbl[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void rpfree(void *);

struct cfdriver rp_cd = {
	NULL, "rp", DV_TTY
};

/*
 * Purpose:  Read the AIOP idenfication number directly from an AIOP.
 *
 * Return:
 * Flag AIOPID_XXXX if a valid AIOP is found, where X is replace by an
 * identifying number.  Flag AIOPID_NULL if no valid AIOP is found.
 *
 * Warnings: No context switches are allowed while executing this function.
 */
int
rp_read_aiopid(struct rp_softc *sc, int aiop)
{
	uint8_t AiopID;		/* ID byte from AIOP */

	rp_writeaiop1(sc, aiop, _CMD_REG, RESET_ALL);	/* reset AIOP */
	rp_writeaiop1(sc, aiop, _CMD_REG, 0x0);
	AiopID = rp_readaiop1(sc, aiop, _CHN_STAT0) & 0x07;

	if (AiopID == 0x06)
		return (1);

	return (-1); /* AIOP does not exist */
}

/*
 * Purpose:
 * Read the number of channels available in an AIOP directly from an AIOP.
 *
 * Return: The number of channels available
 *
 * Comments:
 * The number of channels is determined by write/reads from identical offsets
 * within the SRAM address spaces for channels 0 and 4.  If the channel 4 space
 * is mirrored to channel 0 it is a 4 channel AIOP, otherwise it is an 8
 * channel.
 *
 * Warnings: No context switches are allowed while executing this function.
 */
int
rp_read_aiop_numchan(struct rp_softc *sc, int aiop)
{
	uint16_t x, y;

	/* write to chan 0 SRAM */
	rp_writeaiop4(sc, aiop, _INDX_ADDR, 0x12340000L);

	/* read from SRAM, chan 0 */
	rp_writeaiop2(sc, aiop, _INDX_ADDR, 0);
	x = rp_readaiop2(sc, aiop, _INDX_DATA);

	/* read from SRAM, chan 4 */
	rp_writeaiop2(sc, aiop, _INDX_ADDR, 0x4000);
	y = rp_readaiop2(sc, aiop, _INDX_DATA);

	if (x != y)	/* if different must be 8 chan */
		return (8);

	return (4);
}

/*
 * Purpose: Initialization of a channel and channel structure
 *
 * Return:
 * True if initialization succeeded, false if it fails because channel number
 * exceeds number of channels available in AIOP.
 *
 * Comments: This function must be called before a channel can be used.
 *
 * Warnings:
 * No range checking on any of the parameters is done.
 * No context switches are allowed while executing this function.
 */
int
sInitChan(struct rp_softc *sc, struct rp_chan *ch, int AiopNum, int ChanNum)
{
	int i, ChOff;
	uint8_t *ChR;
	static uint8_t R[4];

	if (ChanNum >= sc->AiopNumChan[AiopNum])
		return (false);	/* exceeds num chans in AIOP */

	/* Channel, AIOP, and controller identifiers */
	ch->CtlP = sc;
	ch->ChanID = sc->AiopID[AiopNum];
	ch->AiopNum = AiopNum;
	ch->ChanNum = ChanNum;

	/* Initialize the channel from the RData array */
	for (i=0; i < RDATASIZE; i+=4) {
		R[0] = RData[i];
		R[1] = RData[i+1] + 0x10 * ChanNum;
		R[2] = RData[i+2];
		R[3] = RData[i+3];
		rp_writech4(ch, _INDX_ADDR, lemtoh32(R));
	}

	ChR = ch->R;
	for (i = 0; i < RREGDATASIZE; i += 4) {
		ChR[i] = RRegData[i];
		ChR[i+1] = RRegData[i+1] + 0x10 * ChanNum;
		ChR[i+2] = RRegData[i+2];
		ChR[i+3] = RRegData[i+3];
	}

	/* Indexed registers */
	ChOff = (uint16_t)ChanNum * 0x1000;

	ch->BaudDiv[0] = (uint8_t)(ChOff + _BAUD);
	ch->BaudDiv[1] = (uint8_t)((ChOff + _BAUD) >> 8);
	ch->BaudDiv[2] = (uint8_t)RP_BRD9600;
	ch->BaudDiv[3] = (uint8_t)(RP_BRD9600 >> 8);
	rp_writech4(ch,_INDX_ADDR,lemtoh32(ch->BaudDiv));

	ch->TxControl[0] = (uint8_t)(ChOff + _TX_CTRL);
	ch->TxControl[1] = (uint8_t)((ChOff + _TX_CTRL) >> 8);
	ch->TxControl[2] = 0;
	ch->TxControl[3] = 0;
	rp_writech4(ch,_INDX_ADDR,lemtoh32(ch->TxControl));

	ch->RxControl[0] = (uint8_t)(ChOff + _RX_CTRL);
	ch->RxControl[1] = (uint8_t)((ChOff + _RX_CTRL) >> 8);
	ch->RxControl[2] = 0;
	ch->RxControl[3] = 0;
	rp_writech4(ch,_INDX_ADDR,lemtoh32(ch->RxControl));

	ch->TxEnables[0] = (uint8_t)(ChOff + _TX_ENBLS);
	ch->TxEnables[1] = (uint8_t)((ChOff + _TX_ENBLS) >> 8);
	ch->TxEnables[2] = 0;
	ch->TxEnables[3] = 0;
	rp_writech4(ch,_INDX_ADDR,lemtoh32(ch->TxEnables));

	ch->TxCompare[0] = (uint8_t)(ChOff + _TXCMP1);
	ch->TxCompare[1] = (uint8_t)((ChOff + _TXCMP1) >> 8);
	ch->TxCompare[2] = 0;
	ch->TxCompare[3] = 0;
	rp_writech4(ch,_INDX_ADDR,lemtoh32(ch->TxCompare));

	ch->TxReplace1[0] = (uint8_t)(ChOff + _TXREP1B1);
	ch->TxReplace1[1] = (uint8_t)((ChOff + _TXREP1B1) >> 8);
	ch->TxReplace1[2] = 0;
	ch->TxReplace1[3] = 0;
	rp_writech4(ch, _INDX_ADDR,lemtoh32(ch->TxReplace1));

	ch->TxReplace2[0] = (uint8_t)(ChOff + _TXREP2);
	ch->TxReplace2[1] = (uint8_t)((ChOff + _TXREP2) >> 8);
	ch->TxReplace2[2] = 0;
	ch->TxReplace2[3] = 0;
	rp_writech4(ch, _INDX_ADDR,lemtoh32(ch->TxReplace2));

	ch->TxFIFOPtrs = ChOff + _TXF_OUTP;
	ch->TxFIFO = ChOff + _TX_FIFO;

	/* apply reset Tx FIFO count */
	rp_writech1(ch, _CMD_REG, (uint8_t)ChanNum | RESTXFCNT);

	rp_writech1(ch, _CMD_REG, (uint8_t)ChanNum);	/* remove reset Tx FIFO count */
	rp_writech2(ch, _INDX_ADDR, ch->TxFIFOPtrs);	/* clear Tx in/out ptrs */
	rp_writech2(ch, _INDX_DATA, 0);
	ch->RxFIFOPtrs = ChOff + _RXF_OUTP;
	ch->RxFIFO = ChOff + _RX_FIFO;

	/* apply reset Rx FIFO count */
	rp_writech1(ch, _CMD_REG, (uint8_t)ChanNum | RESRXFCNT);

	rp_writech1(ch, _CMD_REG, (uint8_t)ChanNum);	/* remove reset Rx FIFO count */
	rp_writech2(ch, _INDX_ADDR, ch->RxFIFOPtrs);	/* clear Rx out ptr */
	rp_writech2(ch, _INDX_DATA, 0);
	rp_writech2(ch, _INDX_ADDR, ch->RxFIFOPtrs + 2);	/* clear Rx in ptr */
	rp_writech2(ch, _INDX_DATA, 0);
	ch->TxPrioCnt = ChOff + _TXP_CNT;
	rp_writech2(ch, _INDX_ADDR, ch->TxPrioCnt);
	rp_writech1(ch, _INDX_DATA, 0);
	ch->TxPrioPtr = ChOff + _TXP_PNTR;
	rp_writech2(ch, _INDX_ADDR, ch->TxPrioPtr);
	rp_writech1(ch, _INDX_DATA, 0);
	ch->TxPrioBuf = ChOff + _TXP_BUF;
	sEnRxProcessor(ch);	/* start the Rx processor */

	return (true);
}

/*
 * Purpose:  Stop the receive processor from processing a channel.
 *
 * Comments:
 * The receive processor can be started again with sStartRxProcessor().  This
 * function causes the receive processor to skip over the stopped channel.  It
 * does not stop it from processing other channels.
 *
 * Warnings:
 * No context switches are allowed while executing this function.
 * Do not leave the receive processor stopped for more than one character time.
 * After calling this function a delay of 4 uS is required to ensure that the
 * receive processor is no longer processing this channel.
 */
void
sStopRxProcessor(struct rp_chan *ch)
{
	uint8_t R[4];

	R[0] = ch->R[0];
	R[1] = ch->R[1];
	R[2] = 0x0a;
	R[3] = ch->R[3];

	rp_writech4(ch, _INDX_ADDR, lemtoh32(R));
}

/*
 * Purpose:  Flush the Rx FIFO
 *
 * Comments:
 * To prevent data from being enqueued or dequeued in the Tx FIFO while it is
 * being flushed the receive processor is stopped and the transmitter is
 * disabled.  After these operations a 4 uS delay is done before clearing the
 * pointers to allow the receive processor to stop.  These items are handled
 * inside this function.
 *
 * Warnings: No context switches are allowed while executing this function.
 */
void
sFlushRxFIFO(struct rp_chan *ch)
{
	int	RxFIFOEnabled = false;	/* true if Rx FIFO enabled */
	int	i;
	uint8_t	Ch;			/* channel number within AIOP */

	if (sGetRxCnt(ch) == 0)	/* Rx FIFO empty */
		return;			/* don't need to flush */

	if (ch->R[0x32] == 0x08) {	/* Rx FIFO is enabled */
		RxFIFOEnabled = true;
		sDisRxFIFO(ch);	/* disable it */

		/* delay 2 uS to allow proc to disable FIFO*/
		for (i = 0; i < 2000/200; i++)
			rp_readch1(ch,_INT_CHAN);	/* depends on bus i/o timing */
	}
	sGetChanStatus(ch);	/* clear any pending Rx errors in chan stat */
	Ch = (uint8_t)sGetChanNum(ch);
	rp_writech1(ch,_CMD_REG,Ch | RESRXFCNT);	/* apply reset Rx FIFO count */
	rp_writech1(ch,_CMD_REG,Ch);			/* remove reset Rx FIFO count */
	rp_writech2(ch,_INDX_ADDR,ch->RxFIFOPtrs);	/* clear Rx out ptr */
	rp_writech2(ch,_INDX_DATA,0);
	rp_writech2(ch,_INDX_ADDR,ch->RxFIFOPtrs + 2);/* clear Rx in ptr */
	rp_writech2(ch,_INDX_DATA,0);

	if (RxFIFOEnabled)
		sEnRxFIFO(ch);	/* enable Rx FIFO */
}

/*
 * Purpose: Flush the Tx FIFO
 *
 * Comments:
 * To prevent data from being enqueued or dequeued in the Tx FIFO while it is
 * being flushed the receive processor is stopped and the transmitter is
 * disabled.  After these operations a 4 uS delay is done before clearing the
 * pointers to allow the receive processor to stop.  These items are handled
 * inside this function.
 *
 * Warnings: No context switches are allowed while executing this function.
 */
void
sFlushTxFIFO(struct rp_chan *ch)
{
	int i;
	uint8_t Ch;		/* channel number within AIOP */
	int TxEnabled;		/* true if transmitter enabled */

	if (sGetTxCnt(ch) == 0)	/* Tx FIFO empty */
		return;		/* don't need to flush */

	TxEnabled = false;
	if (ch->TxControl[3] & TX_ENABLE) {
		TxEnabled = true;
		sDisTransmit(ch);	       /* disable transmitter */
	}

	sStopRxProcessor(ch);		/* stop Rx processor */
	for (i = 0; i < 4000/200; i++)	/* delay 4 uS to allow proc to stop */
		rp_readch1(ch,_INT_CHAN);		/* depends on bus i/o timing */
	Ch = (uint8_t)sGetChanNum(ch);
	rp_writech1(ch, _CMD_REG, Ch | RESTXFCNT);	/* apply reset Tx FIFO count */
	rp_writech1(ch, _CMD_REG, Ch);			/* remove reset Tx FIFO count */
	rp_writech2(ch, _INDX_ADDR, ch->TxFIFOPtrs);	/* clear Tx in/out ptrs */
	rp_writech2(ch, _INDX_DATA, 0);

	if (TxEnabled)
		sEnTransmit(ch);	/* enable transmitter */

	sStartRxProcessor(ch);		/* restart Rx processor */
}

/*
 * Purpose : Write a byte of priority transmit data to a channel.
 * Return  : 1 if the bytes is successfully written, otherwise 0.
 * Comments: The priority byte is transmitted before any data in the Tx FIFO.
 * Warnings: No context switches are allowed while executing this function.
 */
int
sWriteTxPrioByte(struct rp_chan *ch, uint8_t Data)
{
	uint8_t DWBuf[4];		/* buffer for double word writes */

	if (sGetTxCnt(ch) > 1) {	/* write it to Tx priority buffer */
		/* get priority buffer status */
		rp_writech2(ch, _INDX_ADDR, ch->TxPrioCnt);

		if (rp_readch1(ch, _INDX_DATA) & PRI_PEND)	/* priority buffer busy */
			return (0);		/* nothing sent */

		htolem16(DWBuf, ch->TxPrioBuf);/* data byte address */

		DWBuf[2] = Data;		/* data byte value */
		DWBuf[3] = 0;			/* priority buffer pointer */
		rp_writech4(ch,_INDX_ADDR,lemtoh32(DWBuf)); /* write it out */

		htolem16(DWBuf,ch->TxPrioCnt);	/* Tx priority count address */

		DWBuf[2] = PRI_PEND + 1;	/* indicate 1 byte pending */
		DWBuf[3] = 0;			/* priority buffer pointer */
		rp_writech4(ch, _INDX_ADDR, lemtoh32(DWBuf));	/* write it out */
	} else {
		/* write it to Tx FIFO */
		sWriteTxByte(ch, sGetTxRxDataIO(ch), Data);
	}

	return (1);	/* 1 byte sent */
}

/*
 * Purpose: Enable one or more interrupts for a channel
 *
 * Call:	  sEnInterrupts(ch,Flags)
 * 	  struct rp_chan *ch; Ptr to channel structure
 * 	  uint16_t Flags: Interrupt enable flags, can be any combination
 * 	     of the following flags:
 * 		TXINT_EN:   Interrupt on Tx FIFO empty
 * 		RXINT_EN:   Interrupt on Rx FIFO at trigger level (see
 * 			    sSetRxTrigger())
 * 		SRCINT_EN:  Interrupt on SRC (Special Rx Condition)
 * 		MCINT_EN:   Interrupt on modem input change
 * 		CHANINT_EN: Allow channel interrupt signal to the AIOP's
 * 			    Interrupt Channel Register.
 *
 * Comments:
 * If an interrupt enable flag is set in Flags, that interrupt will be enabled.
 * If an interrupt enable flag is not set in Flags, that interrupt will not be
 * changed.  Interrupts can be disabled with function rp_disable_interrupts().
 *
 * This function sets the appropriate bit for the channel in the AIOP's
 * Interrupt Mask Register if the CHANINT_EN flag is set.  This allows this
 * channel's bit to be set in the AIOP's Interrupt Channel Register.
 *
 * Interrupts must also be globally enabled before channel interrupts will be
 * passed on to the host.  This is done with function sEnGlobalInt().
 *
 * In some cases it may be desirable to disable interrupts globally but enable
 * channel interrupts.  This would allow the global interrupt status register
 * to be used to determine which AIOPs need service.
 */
void
sEnInterrupts(struct rp_chan *ch, uint16_t Flags)
{
	uint8_t Mask;	/* Interrupt Mask Register */

	ch->RxControl[2] |= ((uint8_t)Flags & (RXINT_EN | SRCINT_EN | MCINT_EN));
	rp_writech4(ch, _INDX_ADDR, lemtoh32(ch->RxControl));

	ch->TxControl[2] |= ((uint8_t)Flags & TXINT_EN);
	rp_writech4(ch, _INDX_ADDR, lemtoh32(ch->TxControl));

	if (Flags & CHANINT_EN) {
		Mask = rp_readch1(ch,_INT_MASK) | rp_sBitMapSetTbl[ch->ChanNum];
		rp_writech1(ch, _INT_MASK, Mask);
	}
}

/*
 * Purpose: Disable one or more interrupts for a channel
 *
 * Call:	rp_disable_interrupts(ch, Flags)
 * 	  struct rp_chan *ch; Ptr to channel structure
 * 	  uint16_t Flags: Interrupt flags, can be any combination
 * 	     of the following flags:
 * 		TXINT_EN:   Interrupt on Tx FIFO empty
 * 		RXINT_EN:   Interrupt on Rx FIFO at trigger level (see
 * 			    sSetRxTrigger())
 * 		SRCINT_EN:  Interrupt on SRC (Special Rx Condition)
 * 		MCINT_EN:   Interrupt on modem input change
 * 		CHANINT_EN: Disable channel interrupt signal to the
 * 			    AIOP's Interrupt Channel Register.
 *
 * Comments:
 * If an interrupt flag is set in Flags, that interrupt will be disabled.  If
 * an interrupt flag is not set in Flags, that interrupt will not be changed.
 * Interrupts can be enabled with function sEnInterrupts().
 *
 * This function clears the appropriate bit for the channel in the AIOP's
 * Interrupt Mask Register if the CHANINT_EN flag is set.  This blocks this
 * channel's bit from being set in the AIOP's Interrupt Channel Register.
 */
void
rp_disable_interrupts(struct rp_chan *ch, uint16_t Flags)
{
	uint8_t Mask;	/* Interrupt Mask Register */

	ch->RxControl[2] &=
	    ~((uint8_t)Flags & (RXINT_EN | SRCINT_EN | MCINT_EN));
	rp_writech4(ch, _INDX_ADDR, lemtoh32(ch->RxControl));

	ch->TxControl[2] &= ~((uint8_t)Flags & TXINT_EN);
	rp_writech4(ch, _INDX_ADDR, lemtoh32(ch->TxControl));

	if (Flags & CHANINT_EN) {
		Mask = rp_readch1(ch,_INT_MASK) & rp_sBitMapClrTbl[ch->ChanNum];
		rp_writech1(ch,_INT_MASK,Mask);
	}
}

/*
 * Begin FreeBsd-specific driver code
 */

#define RP_POLL_INTERVAL	(hz / 100)

#define RP_ISMULTIPORT(dev)	((dev)->id_flags & 0x1)
#define RP_MPMASTER(dev)	(((dev)->id_flags >> 8) & 0xff)
#define RP_NOTAST4(dev) 	((dev)->id_flags & 0x04)

/*
 * The top-level routines begin here
 */

int	rpclose(dev_t dev, int, int, struct proc *);
void	rphardclose(struct tty *, struct rp_port *);
//int	rpmodem(struct tty *, int, int);
int	rpparam(struct tty *, struct termios *);
void	rpstart(struct tty *);
struct tty *rptty(dev_t);
int	rpioctl(dev_t dev, u_long , caddr_t, int, struct proc *);
int	rpopen(dev_t dev, int, int, struct proc *);

static void
rp_do_receive(struct rp_port *rp, struct tty *tp, struct rp_chan *cp,
    unsigned int ChanStatus)
{
	unsigned int CharNStat;
	int ToRecv, ch, s;

	ToRecv = sGetRxCnt(cp);
	if (ToRecv == 0)
		return;

	/*
	 * If status indicates there are errored characters in the FIFO, then
	 * enter status mode (a word in FIFO holds characters and status)
	 */
	if (ChanStatus & (RXFOVERFL | RXBREAK | RXFRAME | RXPARITY)) {
		if (!(ChanStatus & STATMODE)) {
			ChanStatus |= STATMODE;
			sEnRxStatusMode(cp);
		}
	}

	/*
	 * if we previously entered status mode then read down the FIFO one
	 * word at a time, pulling apart the character and the status. Update
	 * error counters depending on status.
	 */
	s = spltty();
	if (ChanStatus & STATMODE) {
		while (ToRecv) {
			CharNStat = rp_readch2(cp, sGetTxRxDataIO(cp));
			ch = CharNStat & 0xff;

			if ((CharNStat & STMBREAK) || (CharNStat & STMFRAMEH))
				ch |= TTY_FE;
			else if (CharNStat & STMPARITYH)
				ch |= TTY_PE;
			else if (CharNStat & STMRCVROVRH) {
				rp->rp_overflows++;

				printf("%s port %d tty overrun\n",
				    rp->rp_ctlp->sc_dev.dv_xname,
				    RP_PORT(tp->t_dev));
			}

			//ttydisc_rint(tp, ch, err);
			(*linesw[tp->t_line].l_rint)(ch, tp);
			ToRecv--;
		}

		/* After emtying FIFO in status mode, turn off status mode */
		if (sGetRxCnt(cp) == 0)
			sDisRxStatusMode(cp);
	} else {
		ToRecv = sGetRxCnt(cp);
		while (ToRecv) {
			ch = rp_readch1(cp, sGetTxRxDataIO(cp));
			(*linesw[tp->t_line].l_rint)(ch & 0xff, tp);
			ToRecv--;
		}
	}
	//XXX: do we need this?! -> ttydisc_rint_done(tp);
	splx(s);
}

static void
rp_handle_port(struct rp_port *rp)
{
	struct rp_chan	*cp;
	struct tty	*tp;
	unsigned int	 IntMask;
	unsigned int	 ChanStatus;

	if (rp == NULL)
		return;

	cp = &rp->rp_channel;
	tp = rp->rp_tty;
	IntMask = sGetChanIntID(cp);
	IntMask = IntMask & rp->rp_intmask;
	ChanStatus = sGetChanStatus(cp);
	if (IntMask & RXF_TRIG)
		rp_do_receive(rp, tp, cp, ChanStatus);

	if (IntMask & DELTA_CD) {
		if (ChanStatus & CD_ACT)
			(void)(*linesw[tp->t_line].l_modem)(tp, 1);
		else
			(void)(*linesw[tp->t_line].l_modem)(tp, 0);
	}
/*	oldcts = rp->rp_cts;
	rp->rp_cts = ((ChanStatus & CTS_ACT) != 0);
	if (oldcts != rp->rp_cts) {
		printf("CTS change (now %s)... on port %d\n", rp->rp_cts ? "on" : "off", rp->rp_port);
	}
*/
}

void
rp_do_poll(void *arg)
{
	struct rp_port	*rp = arg;
	struct rp_softc	*sc = rp->rp_ctlp;
	struct tty	*tp = rp->rp_tty;
	int		 count;
	unsigned char	 CtlMask;
	unsigned char	 AiopMask;

//	tty_lock_assert(tp, MA_OWNED);	//XXX: do we need this?
	CtlMask = sc->ctlmask(sc);
	if (CtlMask & (1 << rp->rp_aiop)) {
		AiopMask = sGetAiopIntStatus(sc, rp->rp_aiop);
		if (AiopMask & (1 << rp->rp_chan))
			rp_handle_port(rp);
	}

	count = sGetTxCnt(&rp->rp_channel);
	if (count >= 0 && count <= rp->rp_restart)
		rpstart(tp);

	timeout_add(&rp->rp_timer, RP_POLL_INTERVAL);
}

void
rpfree(void *softc)
{
	struct rp_port *rp = softc;
	struct rp_softc *sc = rp->rp_ctlp;

	atomic_dec_int(&sc->free);
}

int
rp_attach(struct rp_softc *sc, int num_aiops, int num_ports)
{
	struct rp_port	*rp;
	struct tty	*tp;
	int		 unit;
	int		 num_chan;
	int		 aiop, chan, port;
	int		 ChanStatus;
	int		 retval;

	unit = sc->sc_dev.dv_unit;

	printf(" rp%d (Version %s) %d ports\n", unit, RP_Version, num_ports);

	sc->num_ports = num_ports;
	sc->rp = rp = mallocarray(num_ports, sizeof(*rp), M_DEVBUF, M_NOWAIT|M_ZERO);

	if (rp == NULL) {
		//XXX: improve message
		printf("rp_attach: Could not malloc rp_ports structures.\n");
		retval = ENOMEM;
		goto nogo;
	}

	port = 0;
	for (aiop=0; aiop < num_aiops; aiop++) {
		num_chan = sGetAiopNumChan(sc, aiop);
		for (chan = 0; chan < num_chan; chan++, port++, rp++) {
			rp->rp_tty = tp = ttymalloc(0);
			tp->t_oproc = rpstart;
			tp->t_param = rpparam;

//XXX: do we need this:	callout_init_mtx(&rp->rp_timer, tty_getlock(tp), 0);
			rp->rp_port = port;
			rp->rp_ctlp = sc;
			rp->rp_unit = unit;
			rp->rp_chan = chan;
			rp->rp_aiop = aiop;

			rp->rp_intmask = RXF_TRIG | TXFIFO_MT | SRC_INT |
				DELTA_CD | DELTA_CTS | DELTA_DSR;
#ifdef notdef
			ChanStatus = sGetChanStatus(&rp->rp_channel);
#endif /* notdef */
			if (sInitChan(sc, &rp->rp_channel, aiop, chan) == 0) {
				//XXX: fix this message
				printf(" init channel (%d, %d, %d) failed.\n",
				    unit, aiop, chan);
				retval = ENXIO;
				goto nogo;
			}
			ChanStatus = sGetChanStatus(&rp->rp_channel);
			rp->rp_cts = (ChanStatus & CTS_ACT) != 0;
//XXX: do we need this:	tty_makedev(tp, NULL, "R%r%r", unit, port);
		}
	}

	mtx_init(&sc->hwmtx, IPL_TTY);
	sc->hwmtx_init = 1;
	return (0);
 nogo:
	rp_releaseresource(sc);
	return (retval);
}

void
rp_releaseresource(struct rp_softc *sc)
{
	if (sc->rp != NULL) {
		int i;

		for (i = 0; i < sc->num_ports; i++) {
			struct rp_port *rp = sc->rp + i;

			atomic_inc_int(&sc->free);
//XXX: do we need this:	s = spltty();	//tty_lock(rp->rp_tty);
//			tty_rel_gone(rp->rp_tty);
			ttyfree(rp->rp_tty);
		}
printf("free(%p, %d, %zu * %d);\n",
    sc->rp, M_DEVBUF, sizeof(*(sc->rp)), sc->num_ports);
		free(sc->rp, M_DEVBUF, sizeof(*(sc->rp)) * sc->num_ports);
		sc->rp = NULL;
	}

//XXX: do we need this:
//	while (sc->free != 0) {
//		pause("rpwt", hz / 10);
//	}

//XXX: do we need this:
//	if (sc->hwmtx_init)
//		mtx_destroy(&sc->hwmtx);
}

int
rpopen(dev_t dev, int flag, int mode, struct proc *p)
{
	int  card = RP_CARD(dev);
	int  port = RP_PORT(dev);
	struct rp_softc	*sc;
	struct rp_port	*rp;
	struct tty	*tp;
	int		 s, flags = 0;
	unsigned int	 IntMask, ChanStatus;

	if (card >= rp_cd.cd_ndevs || (sc = rp_cd.cd_devs[card]) == NULL)
		return (ENXIO);

#ifdef RP_DEBUG
	printf("%s open port %d flag 0x%x mode 0x%x\n", sc->sc_dev.dv_xname,
	    port, flag, mode);
#endif

	rp = &sc->rp[port];

	s = spltty();
	if (rp->rp_tty == NULL) {
		rp->rp_tty = ttymalloc(0);
	}
	splx(s);

	tp = rp->rp_tty;
	tp->t_oproc = rpstart;
	tp->t_param = rpparam;
	tp->t_dev = dev;

	flags |= SET_RTS;
	flags |= SET_DTR;
	rp->rp_channel.TxControl[3] =
	    ((rp->rp_channel.TxControl[3] & ~(SET_RTS | SET_DTR)) | flags);
	rp_writech4(&rp->rp_channel,_INDX_ADDR, lemtoh32(rp->rp_channel.TxControl));
	sSetRxTrigger(&rp->rp_channel, TRIG_1);
	sDisRxStatusMode(&rp->rp_channel);
	sFlushRxFIFO(&rp->rp_channel);
	sFlushTxFIFO(&rp->rp_channel);

	sEnInterrupts(&rp->rp_channel,
	    (TXINT_EN|MCINT_EN|RXINT_EN|SRCINT_EN|CHANINT_EN));
	sSetRxTrigger(&rp->rp_channel, TRIG_1);

	sDisRxStatusMode(&rp->rp_channel);
	sClrTxXOFF(&rp->rp_channel);

//	sDisRTSFlowCtl(&rp->rp_channel);
//	sDisCTSFlowCtl(&rp->rp_channel);

	sDisTxSoftFlowCtl(&rp->rp_channel);
	sStartRxProcessor(&rp->rp_channel);

	sEnRxFIFO(&rp->rp_channel);
	sEnTransmit(&rp->rp_channel);

//	sSetDTR(&rp->rp_channel);
//	sSetRTS(&rp->rp_channel);

	IntMask = sGetChanIntID(&rp->rp_channel);
	IntMask = IntMask & rp->rp_intmask;
	ChanStatus = sGetChanStatus(&rp->rp_channel);

//XXX:	callout_reset(&rp->rp_timer, POLL_INTERVAL, rp_do_poll, rp);
	timeout_set(&rp->rp_timer, rp_do_poll, rp);
	timeout_add(&rp->rp_timer, RP_POLL_INTERVAL);

//XXX:	device_busy(rp->rp_ctlp->dev);
	return (*linesw[tp->t_line].l_open)(dev, tp, p);
}

int
rpclose(dev_t dev, int flag, int mode, struct proc *p)
{
	int		 card = RP_CARD(dev);
	int		 port = RP_PORT(dev);
	struct rp_softc	*sc = rp_cd.cd_devs[card];
	struct rp_port	*rp = &sc->rp[port];
	struct tty	*tp = rp->rp_tty;

	timeout_del(&rp->rp_timer);
	rphardclose(tp, rp);
//XXX:	device_unbusy(rp->rp_ctlp->dev);

	return (0);
}

void
rphardclose(struct tty *tp, struct rp_port *rp)
{
//XXX:	struct	rp_port	*rp;
	struct rp_chan	*cp;

//XXX:	rp = tty_softc(tp);
	cp = &rp->rp_channel;

	sFlushRxFIFO(cp);
	sFlushTxFIFO(cp);
	sDisTransmit(cp);
	rp_disable_interrupts(cp, TXINT_EN|MCINT_EN|RXINT_EN|SRCINT_EN|CHANINT_EN);
	sDisRTSFlowCtl(cp);
	sDisCTSFlowCtl(cp);
	sDisTxSoftFlowCtl(cp);
	sClrTxXOFF(cp);

#ifdef DJA
	if (tp->t_cflag&HUPCL || !(tp->t_state & TS_ISOPEN) || !tp->t_actout)
		sClrDTR(cp);

	if (ISCALLOUT(tp->t_dev))
		sClrDTR(cp);

	tp->t_actout = false;
	wakeup(&tp->t_actout);
	wakeup(TSA_CARR_ON(tp));
#endif /* DJA */
}

int
rpread(dev_t dev, struct uio *uio, int flag)
{
	int		 card = RP_CARD(dev);
	int		 port = RP_PORT(dev);
	struct rp_softc	*sc = rp_cd.cd_devs[card];
	struct rp_port	*rp = &sc->rp[port];
	struct tty	*tp = rp->rp_tty;

#ifdef RP_DEBUG
	printf("%s read port %d uio %p flag 0x%x\n", sc->sc_dev.dv_xname,
	    port, uio, flag);
#endif

	return ((*linesw[tp->t_line].l_read)(tp, uio, flag));
}

int
rpwrite(dev_t dev, struct uio *uio, int flag)
{
	int card = RP_CARD(dev);
	int port = RP_PORT(dev);
	struct rp_softc *sc = rp_cd.cd_devs[card];
	struct rp_port *rp = &sc->rp[port];
	struct tty *tp = rp->rp_tty;

#ifdef RP_DEBUG
	printf("%s write port %d uio %p flag 0x%x\n", sc->sc_dev.dv_xname,
	    port, uio, flag);
#endif

	return ((*linesw[tp->t_line].l_write)(tp, uio, flag));
}

struct tty *
rptty(dev_t dev)
{
	int card = RP_CARD(dev);
	int port = RP_PORT(dev);
	struct rp_softc *sc = rp_cd.cd_devs[card];
	struct rp_port *rp = &sc->rp[port];
	struct tty *tp = rp->rp_tty;

	return (tp);
}

int
rpioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	int card = RP_CARD(dev);
	int port = RP_PORT(dev);
	struct rp_softc *sc = rp_cd.cd_devs[card];
	struct rp_port *rp = &sc->rp[port];
	struct tty *tp = rp->rp_tty;
	int error;

#ifdef RP_DEBUG
	printf("%s port %d ioctl cmd 0x%lx data %p flag 0x%x\n",
	    sc->sc_dev.dv_xname, port, cmd, data, flag);
#endif

	error = (*linesw[tp->t_line].l_ioctl)(tp, cmd, data, flag, p);
	if (error >= 0)
		return (error);

	error = ttioctl(tp, cmd, data, flag, p);
	if (error >= 0)
		return (error);

	switch (cmd) {
	case TIOCSBRK:
		return (0);
	case TIOCCBRK:
		return (0);
	case TIOCSDTR:	/* DIR on */
// TIOCM_DTR, DMBIS
	case TIOCCDTR:	/* DIR off */
	case TIOCMSET:	/* set new modem control line values */
	case TIOCMBIS:	/* turn modem control bits on */
	case TIOCMBIC:	/* turn modem control bits off */
	case TIOCMGET:	/* get modem control/status line state */
	case TIOCGFLAGS:/* set flags */
	case TIOCSFLAGS:/* get flags */
	default:
		return ENOTTY;
	}
}
#if 0
int
rpmodem(struct tty *tp, int sigon, int sigoff)
{
	struct rp_port	*rp;
	int		 i, j, k;

	rp = tty_softc(tp);
	if (sigon != 0 || sigoff != 0) {
		i = j = 0;
		if (sigon & SER_DTR)
			i = SET_DTR;
		if (sigoff & SER_DTR)
			j = SET_DTR;
		if (sigon & SER_RTS)
			i = SET_RTS;
		if (sigoff & SER_RTS)
			j = SET_RTS;
		rp->rp_channel.TxControl[3] &= ~i;
		rp->rp_channel.TxControl[3] |= j;
		rp_writech4(&rp->rp_channel,_INDX_ADDR,
			lemtoh32(rp->rp_channel.TxControl));
	} else {
		i = sGetChanStatusLo(&rp->rp_channel);
		j = rp->rp_channel.TxControl[3];
		k = 0;
		if (j & SET_DTR)
			k |= SER_DTR;
		if (j & SET_RTS)
			k |= SER_RTS;
		if (i & CD_ACT)
			k |= SER_DCD;
		if (i & DSR_ACT)
			k |= SER_DSR;
		if (i & CTS_ACT)
			k |= SER_CTS;
		return(k);
	}
	return (0);
}
#endif

static struct {
	int baud;
	int conversion;
} baud_table[] = {
	{B0,      0},		{B50,    RP_BRD50},	{B75,     RP_BRD75},
	{B110,    RP_BRD110}, 	{B134,   RP_BRD134}, 	{B150,    RP_BRD150},
	{B200,	  RP_BRD200}, 	{B300,   RP_BRD300}, 	{B600,    RP_BRD600},
	{B1200,	  RP_BRD1200},	{B1800,  RP_BRD1800},	{B2400,   RP_BRD2400},
	{B4800,   RP_BRD4800},	{B9600,  RP_BRD9600},	{B19200,  RP_BRD19200},
	{B38400,  RP_BRD38400},	{B7200,  RP_BRD7200},	{B14400,  RP_BRD14400},
	{B57600,  RP_BRD57600},	{B76800, RP_BRD76800},	{B115200, RP_BRD115200},
	{B230400, RP_BRD230400}, {-1, -1}
};

static int
rp_convert_baud(int baud)
{
	int i;

	for (i = 0; baud_table[i].baud >= 0; i++)
		if (baud_table[i].baud == baud)
			break;

	return baud_table[i].conversion;
}

int
rpstop(struct tty *tp, int flag)
{
	int		 card = RP_CARD(tp->t_dev);
	int		 port = RP_PORT(tp->t_dev);
	struct rp_softc *sc = rp_cd.cd_devs[card];
	//struct rp_port *rp = &sc->rp[port];

	/* XXX: not implemented yet */
	printf("%s port %d stop tty %p flag 0x%x NOT IMPLEMENTED!!!\n",
	    sc->sc_dev.dv_xname, port, tp, flag);

	return (0);
}

int
rpparam(struct tty *tp, struct termios *t)
{
	int		 card = RP_CARD(tp->t_dev);
	int		 port = RP_PORT(tp->t_dev);
	struct rp_softc	*sc = rp_cd.cd_devs[card];
	struct rp_port	*rp = &sc->rp[port];
	struct rp_chan	*cp = &rp->rp_channel;
	int		 cflag = t->c_cflag;
	int		 iflag = t->c_iflag;
//	int		 oflag = t->c_oflag;
//	int		 lflag = t->c_lflag;
	int		 ospeed = rp_convert_baud(t->c_ispeed);

#ifdef RPCLOCAL
	int devshift;
	devshift = umynor / 32;
	devshift = 1 << devshift;
	if (devshift & RPCLOCAL)
		cflag |= CLOCAL;
#endif

	if (ospeed < 0 || t->c_ispeed != t->c_ospeed)
		return (EINVAL);

	if (t->c_ospeed == 0) {
		sClrDTR(cp);
		return (0);
	}
	rp->rp_fifo_lw = ((t->c_ospeed*2) / 1000) +1;

	/* Set baud rate ----- we only pay attention to ispeed */
	sSetDTR(cp);
	sSetRTS(cp);
	sSetBaud(cp, ospeed);

	if (cflag & CSTOPB)
		sSetStop2(cp);
	else
		sSetStop1(cp);

	if (cflag & PARENB) {
		sEnParity(cp);

		if (cflag & PARODD)
			sSetOddParity(cp);
		else
			sSetEvenParity(cp);
	} else {
		sDisParity(cp);
	}

	if ((cflag & CSIZE) == CS8) {
		sSetData8(cp);
		rp->rp_imask = 0xFF;
	} else {
		sSetData7(cp);
		rp->rp_imask = 0x7F;
	}

	if (iflag & ISTRIP)
		rp->rp_imask &= 0x7F;

	if (cflag & CLOCAL)
		rp->rp_intmask &= ~DELTA_CD;
	else
		rp->rp_intmask |= DELTA_CD;

	/* Put flow control stuff here */

	if (cflag & CCTS_OFLOW)
		sEnCTSFlowCtl(cp);
	else
		sDisCTSFlowCtl(cp);

	if (cflag & CRTS_IFLOW)
		rp->rp_rts_iflow = 1;
	else
		rp->rp_rts_iflow = 0;

	if (cflag & CRTS_IFLOW)
		sEnRTSFlowCtl(cp);
	else
		sDisRTSFlowCtl(cp);

	return (0);
}

void
rpstart(struct tty *tp)
{
	int		 card = RP_CARD(tp->t_dev);
	int		 port = RP_PORT(tp->t_dev);
	struct rp_softc	*sc = rp_cd.cd_devs[card];
	struct rp_port	*rp = &sc->rp[port];
	struct rp_chan	*cp = &rp->rp_channel;
//	char		 flags = rp->rp_channel.TxControl[3];
	int		 xmit_fifo_room;
	int		 i, count, wcount;

#ifdef RP_DEBUG
	printf("%s port %d start, tty %p\n", sc->sc_dev.dv_xname, port, tp);
#endif

	if (rp->rp_xmit_stopped) {
		sEnTransmit(cp);
		rp->rp_xmit_stopped = 0;
	}

	xmit_fifo_room = TXFIFO_SIZE - sGetTxCnt(cp);
	count = q_to_b(&tp->t_outq, rp->TxBuf, xmit_fifo_room);
	if (xmit_fifo_room > 0) {
		for (i = 0, wcount = count >> 1; wcount > 0; i += 2, wcount--)
			rp_writech2(cp, sGetTxRxDataIO(cp), lemtoh16(&rp->TxBuf[i]));

		if (count & 1)
			rp_writech1(cp, sGetTxRxDataIO(cp), rp->TxBuf[(count-1)]);
	}
}
