/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * phone-flash burn engine, mcu-independent core: a polled otg bulk endpoint, single-datagram ncm
 * framing, a tiny arp/ip/udp stack and the flash protocol state machine. included by each per-mcu
 * wrapper (phoneflash_burn_<mcu>.c) after it has defined the RF/RFI macros, the otg register accessors,
 * the burn buffers (rxNtb/txNtb/txFrame/replyBuf), rfKickWdg, the flash ops (rfFlashUnlock/rfFlashLock/
 * rfProgram/rfProgramBegin/rfProgramFinish) and the RF_HZ / RF_IDLE_TIMEOUT / PF_FLASH_SIZE / PF_CHIP_ID
 * and mac/ip/endpoint constants. shared state and byte helpers come from phoneflash_burn.h. not a
 * standalone translation unit.
 */

// ---- polled otg bulk endpoint ----

static RF void rfArmOut(void)
{
    OTG_OUTEP(PF_OUT_EPNUM)->DOEPTSIZ = (32U << 19) | (2048U & 0x7FFFFU);   // 32 packets of 64
    OTG_OUTEP(PF_OUT_EPNUM)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

// take the endpoints back from the hal: flush the fifos left mid-transfer and re-arm out
static RF void rfUsbTakeover(void)
{
    uint32_t start = DWT->CYCCNT;
    OTG->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) {
        rfKickWdg();
        if ((uint32_t)(DWT->CYCCNT - start) > RF_HZ) {
            rfReboot();                                                     // nothing erased yet, firmware intact
        }
    }
    start = DWT->CYCCNT;
    OTG->GRSTCTL = (0x10U << 6) | USB_OTG_GRSTCTL_TXFFLSH;                  // txfnum 0x10 = all fifos
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) {
        rfKickWdg();
        if ((uint32_t)(DWT->CYCCNT - start) > RF_HZ) {
            rfReboot();
        }
    }
    rfArmOut();
}

// pull one ncm block (ended by a short or zero-length packet) into rxNtb, or -1 on timeout
static RF int rfUsbPoll(uint32_t timeoutCycles)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t rxLen = 0;
    for (;;) {
        rfKickWdg();
        if (!(OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL)) {
            if ((uint32_t)(DWT->CYCCNT - start) > timeoutCycles) {
                return -1;
            }
            continue;
        }
        uint32_t sts = OTG->GRXSTSP;
        uint32_t ep = sts & USB_OTG_GRXSTSP_EPNUM;
        uint32_t bcnt = (sts & USB_OTG_GRXSTSP_BCNT) >> 4;
        uint32_t pktsts = (sts & USB_OTG_GRXSTSP_PKTSTS) >> 17;
        if (pktsts == 2 && ep == PF_OUT_EPNUM) {                           // out data packet
            uint32_t i = 0;
            for (; i + 4 <= bcnt; i += 4) {
                uint32_t w = OTG_FIFO(0);
                if (rxLen + 4 <= sizeof(rxNtb)) {
                    rxNtb[rxLen] = w; rxNtb[rxLen + 1] = w >> 8; rxNtb[rxLen + 2] = w >> 16; rxNtb[rxLen + 3] = w >> 24;
                    rxLen += 4;
                }
            }
            if (i < bcnt) {
                uint32_t w = OTG_FIFO(0);
                for (uint32_t k = 0; i + k < bcnt; k++) {
                    if (rxLen < sizeof(rxNtb)) {
                        rxNtb[rxLen++] = w >> (8 * k);
                    }
                }
            }
            if (bcnt < 64) {                                               // short packet ends the block
                rfArmOut();
                return (int)rxLen;
            }
        } else if (pktsts == 3 && ep == PF_OUT_EPNUM) {                    // out transfer complete
            rfArmOut();
        } else {
            for (uint32_t i = 0; i + 4 <= bcnt; i += 4) {
                (void)OTG_FIFO(0);
            }
            if (bcnt & 3) {
                (void)OTG_FIFO(0);
            }
        }
        if ((uint32_t)(DWT->CYCCNT - start) > timeoutCycles) {
            return -1;
        }
    }
}

static RF void rfUsbSend(const uint8_t *buf, uint32_t len)
{
    uint32_t pkts = (len + 63) / 64;
    if (pkts == 0) {
        pkts = 1;
    }
    OTG_INEP(PF_OUT_EPNUM)->DIEPTSIZ = (pkts << 19) | (len & 0x7FFFFU);
    OTG_INEP(PF_OUT_EPNUM)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    uint32_t off = 0;
    while (off < len) {
        uint32_t chunk = len - off;
        if (chunk > 64) {
            chunk = 64;
        }
        uint32_t words = (chunk + 3) / 4;
        uint32_t guard = 0;
        while ((OTG_INEP(PF_OUT_EPNUM)->DTXFSTS & 0xFFFF) < words) {
            rfKickWdg();
            if (++guard > 4000000) {
                return;                                                    // host stopped reading, give up
            }
        }
        for (uint32_t i = 0; i < chunk; i += 4) {
            uint32_t w = buf[off + i];
            if (i + 1 < chunk) w |= (uint32_t)buf[off + i + 1] << 8;
            if (i + 2 < chunk) w |= (uint32_t)buf[off + i + 2] << 16;
            if (i + 3 < chunk) w |= (uint32_t)buf[off + i + 3] << 24;
            OTG_FIFO(PF_OUT_EPNUM) = w;
        }
        off += chunk;
    }
}

// ---- ncm framing (single datagram per block) ----

static RF int rfNcmFrame(uint32_t ntbLen, uint8_t **frame, uint32_t *frameLen)
{
    if (ntbLen < 12 || rd32(rxNtb) != 0x484D434EU) {                       // "NCMH"
        return 1;
    }
    uint32_t ndpIndex = rd16(rxNtb + 10);
    if (ndpIndex + 12 > ntbLen) {
        return 1;
    }
    uint8_t *ndp = rxNtb + ndpIndex;
    uint32_t sig = rd32(ndp);
    if (sig != 0x304D434EU && sig != 0x314D434EU) {                        // "NCM0" / "NCM1"
        return 1;
    }
    uint32_t dgIndex = rd16(ndp + 8);
    uint32_t dgLen = rd16(ndp + 10);
    if (dgLen == 0 || dgIndex + dgLen > ntbLen) {
        return 1;
    }
    *frame = rxNtb + dgIndex;
    *frameLen = dgLen;
    return 0;
}

static RF uint32_t rfNcmWrap(const uint8_t *frame, uint32_t frameLen)
{
    wr32(txNtb + 0, 0x484D434EU);
    wr16(txNtb + 4, 12);
    wr16(txNtb + 6, txSeq++);
    uint32_t o = 12;
    rfMemcpy(txNtb + o, frame, frameLen);
    o += frameLen;
    while (o & 3) {
        txNtb[o++] = 0;
    }
    uint32_t ndp = o;
    wr32(txNtb + ndp + 0, 0x304D434EU);
    wr16(txNtb + ndp + 4, 16);
    wr16(txNtb + ndp + 6, 0);
    wr16(txNtb + ndp + 8, 12);
    wr16(txNtb + ndp + 10, frameLen);
    wr16(txNtb + ndp + 12, 0);
    wr16(txNtb + ndp + 14, 0);
    o = ndp + 16;
    wr16(txNtb + 8, o);
    wr16(txNtb + 10, ndp);
    return o;
}

// ---- eth / arp / ip / udp ----

static RF uint16_t rfIpCksum(const uint8_t *h, uint32_t n)
{
    uint32_t s = 0;
    for (uint32_t i = 0; i + 1 < n; i += 2) {
        s += (uint32_t)(h[i] << 8 | h[i + 1]);
    }
    if (n & 1) {
        s += (uint32_t)h[n - 1] << 8;
    }
    while (s >> 16) {
        s = (s & 0xFFFF) + (s >> 16);
    }
    return (uint16_t)~s;
}

static RF void rfPutOurMac(uint8_t *p)
{
    p[0] = 0x02; p[1] = 0x00; p[2] = 0x00; p[3] = 0x00; p[4] = 0x00; p[5] = OUR_MAC_5;
}

static RF void rfHandleArp(const uint8_t *fr, uint32_t frLen)
{
    if (frLen < 42) {
        return;
    }
    const uint8_t *a = fr + 14;
    uint16_t op = (uint16_t)(a[6] << 8 | a[7]);
    uint32_t tpa = rd32be(a + 24);
    if (op != 1 || tpa != OUR_IP) {
        return;
    }
    uint8_t *o = txFrame;
    for (int i = 0; i < 6; i++) o[i] = fr[6 + i];
    rfPutOurMac(o + 6);
    o[12] = 0x08; o[13] = 0x06;
    o[14] = 0x00; o[15] = 0x01; o[16] = 0x08; o[17] = 0x00; o[18] = 6; o[19] = 4; o[20] = 0x00; o[21] = 0x02;
    rfPutOurMac(o + 22);
    wr32be(o + 28, OUR_IP);
    for (int i = 0; i < 6; i++) o[32 + i] = a[8 + i];   // arp target hw/ip = the requester (sha at a+8, spa at a+14)
    for (int i = 0; i < 4; i++) o[38 + i] = a[14 + i];
    uint32_t ntbLen = rfNcmWrap(o, 42);
    rfUsbSend(txNtb, ntbLen);
}

static RF void rfSendUdp(const uint8_t *pay, uint32_t payLen)
{
    uint8_t *o = txFrame;
    for (int i = 0; i < 6; i++) o[i] = hostMac[i];
    rfPutOurMac(o + 6);
    o[12] = 0x08; o[13] = 0x00;
    uint8_t *ip = o + 14;
    uint32_t ipLen = 20 + 8 + payLen;
    ip[0] = 0x45; ip[1] = 0x00; wr16be(ip + 2, (uint16_t)ipLen); wr16be(ip + 4, 0);
    ip[6] = 0x40; ip[7] = 0x00; ip[8] = 64; ip[9] = 17; wr16be(ip + 10, 0);
    wr32be(ip + 12, OUR_IP); wr32be(ip + 16, hostIp);
    wr16be(ip + 10, rfIpCksum(ip, 20));
    uint8_t *udp = ip + 20;
    wr16be(udp + 0, PHONE_FLASH_UDP_PORT); wr16be(udp + 2, hostPort);
    wr16be(udp + 4, (uint16_t)(8 + payLen)); wr16be(udp + 6, 0);          // udp checksum optional in ipv4
    rfMemcpy(udp + 8, pay, payLen);
    uint32_t ntbLen = rfNcmWrap(o, 14 + ipLen);
    rfUsbSend(txNtb, ntbLen);
}

// ---- protocol ----

static RF uint32_t rfAck(uint8_t op)
{
    wr32(replyBuf, PHONE_FLASH_MAGIC);
    replyBuf[4] = op | PF_REPLY;
    replyBuf[5] = PF_ERR_NONE;
    return 6;
}

static RF uint32_t rfErr(uint8_t op, uint8_t code)
{
    wr32(replyBuf, PHONE_FLASH_MAGIC);
    replyBuf[4] = op | PF_REPLY;
    replyBuf[5] = code;
    return 6;
}

// if/else, not switch, to keep gcc from emitting a jump table in flash rodata
static RF uint32_t rfProto(const uint8_t *p, uint32_t n)
{
    if (n < 5 || rd32(p) != PHONE_FLASH_MAGIC) {
        return 0;
    }
    uint8_t op = p[4];

    if (op == PF_HELLO) {
        wr32(replyBuf, PHONE_FLASH_MAGIC);
        replyBuf[4] = PF_HELLO | PF_REPLY;
        replyBuf[5] = PF_ERR_NONE;
        wr32(replyBuf + 6, (uint32_t)(uintptr_t)&__config_start);
        wr32(replyBuf + 10, (uint32_t)(uintptr_t)&__config_end);
        wr32(replyBuf + 14, PF_FLASH_SIZE);
        wr16(replyBuf + 18, PHONE_FLASH_CHUNK);
        wr16(replyBuf + 20, PF_CHIP_ID);
        if (state == ST_WAIT_HELLO) {
            state = ST_READY;                          // a late duplicate must not regress an active session
        }
        return 22;
    }
    if (op == PF_BEGIN) {
        if (state == ST_WAIT_HELLO) {                  // a hello must establish the session first
            return rfErr(op, PF_ERR_STATE);
        }
        if (n < 13) return rfErr(op, PF_ERR_BADOP);
        totalSize = rd32(p + 5);
        imageCrc = rd32(p + 9);
        if (totalSize == 0 || totalSize > PF_FLASH_SIZE) return rfErr(op, PF_ERR_RANGE);
        // begin (re)starts the session from scratch, so a failed end (or any mid-flash fault) is simply
        // retried by sending begin again rather than being unrecoverable short of dfu
        writtenUpTo = 0;
        programmedBytes = 0;
        crcState = 0xFFFFFFFFU;
        erasedMask = 0;
        rfProgramBegin();
        rfFlashUnlock();
        state = ST_FLASHING;
        return rfAck(op);
    }
    if (op == PF_WRITE) {
        if (state != ST_FLASHING) return rfErr(op, PF_ERR_STATE);
        if (n < 11) return rfErr(op, PF_ERR_BADOP);
        uint32_t off = rd32(p + 5);
        uint32_t len = rd16(p + 9);
        if (n < 11 + len) return rfErr(op, PF_ERR_BADOP);
        if ((off & 3) || (len & 3)) return rfErr(op, PF_ERR_ALIGN);
        if (off + len > PF_FLASH_SIZE) return rfErr(op, PF_ERR_RANGE);
        if (off > writtenUpTo) {
            // the host streams strictly in order, so the only legitimate forward jump is over the
            // preserved config region. any other gap means a write was lost and programming past it
            // would silently leave that hole erased, surfacing only as a crc failure at the end.
            const uint32_t cfgLo = (uint32_t)(uintptr_t)&__config_start - PHONE_FLASH_FC_BASE;
            const uint32_t cfgHi = (uint32_t)(uintptr_t)&__config_end - PHONE_FLASH_FC_BASE;
            if (!(writtenUpTo < cfgHi && off > cfgLo)) {
                return rfErr(op, PF_ERR_SEQ);
            }
        }
        if (off >= writtenUpTo) {                       // new data, sequential or the config-region jump
            if (rfProgram(PHONE_FLASH_FC_BASE + off, p + 11, len)) return rfErr(op, PF_ERR_FLASH);
            writtenUpTo = off + len;
            programmedBytes += len;
        }
        // off < writtenUpTo is a lost-ack retransmit of already-programmed data, just re-ack
        wr32(replyBuf, PHONE_FLASH_MAGIC);
        replyBuf[4] = PF_WRITE | PF_REPLY;
        replyBuf[5] = PF_ERR_NONE;
        wr32(replyBuf + 6, writtenUpTo);
        return 10;
    }
    if (op == PF_END) {
        if (state != ST_FLASHING) return rfErr(op, PF_ERR_STATE);
        if (rfProgramFinish()) {                        // flush any staged partial row (h7)
            return rfErr(op, PF_ERR_FLASH);
        }
        uint32_t crc = crcState ^ 0xFFFFFFFFU;
        const uint32_t ok = (crc == imageCrc) && (programmedBytes == totalSize);
        wr32(replyBuf, PHONE_FLASH_MAGIC);
        replyBuf[4] = PF_END | PF_REPLY;
        replyBuf[5] = ok ? PF_ERR_NONE : PF_ERR_CRC;
        wr32(replyBuf + 6, crc);
        if (ok) {
            rfFlashLock();
            state = ST_DONE;
        }
        return 10;
    }
    if (op == PF_REBOOT) {
        if (state == ST_DONE) {
            rebootReq = 1;
            return rfAck(op);
        }
        return rfErr(op, PF_ERR_STATE);
    }
    if (op == PF_ABORT) {
        if (state < ST_FLASHING) {
            abortReq = 1;
            return rfAck(op);
        }
        return rfErr(op, PF_ERR_STATE);
    }
    return rfErr(op, PF_ERR_BADOP);
}

static RF void rfHandleFrame(uint8_t *fr, uint32_t frLen)
{
    if (frLen < 14) {
        return;
    }
    uint16_t ethType = (uint16_t)(fr[12] << 8 | fr[13]);
    if (ethType == 0x0806) {
        rfHandleArp(fr, frLen);
        return;
    }
    if (ethType != 0x0800 || frLen < 34) {
        return;
    }
    uint8_t *ip = fr + 14;
    if ((ip[0] >> 4) != 4) {
        return;
    }
    uint32_t ihl = (uint32_t)(ip[0] & 0x0F) * 4;
    if (ihl < 20 || 14 + ihl + 8 > frLen) {
        return;
    }
    if (ip[9] != 17 || rd32be(ip + 16) != OUR_IP) {
        return;
    }
    uint8_t *udp = ip + ihl;
    if (((udp[2] << 8) | udp[3]) != PHONE_FLASH_UDP_PORT) {
        return;
    }
    uint32_t udpLen = (uint32_t)(udp[4] << 8 | udp[5]);
    if (udpLen < 8 || (uint32_t)(udp - fr) + udpLen > frLen) {
        return;
    }
    for (int i = 0; i < 6; i++) {
        hostMac[i] = fr[6 + i];
    }
    hostIp = rd32be(ip + 12);
    hostPort = (uint16_t)(udp[0] << 8 | udp[1]);
    uint32_t rl = rfProto(udp + 8, udpLen - 8);
    if (rl) {
        rfSendUdp(replyBuf, rl);
    }
}

// ram entry, interrupts masked. runs until the burn reboots, or returns if the host never engages.
static RF void rfMain(void)
{
    state = ST_WAIT_HELLO;
    txSeq = 0;
    rebootReq = 0;
    abortReq = 0;
    writtenUpTo = 0;
    programmedBytes = 0;
    erasedMask = 0;
    rfProgramBegin();

    rfUsbTakeover();

    uint32_t lastRx = DWT->CYCCNT;
    for (;;) {
        int len = rfUsbPoll(RF_HZ / 10);
        if (len > 0) {
            lastRx = DWT->CYCCNT;
            uint8_t *frame;
            uint32_t frameLen;
            if (rfNcmFrame((uint32_t)len, &frame, &frameLen) == 0) {
                rfHandleFrame(frame, frameLen);
            }
            if (rebootReq) {
                rfReboot();
            }
            if (abortReq) {
                return;
            }
        }
        if ((uint32_t)(DWT->CYCCNT - lastRx) > RF_IDLE_TIMEOUT) {
            if (state < ST_FLASHING) {
                return;                                // nothing erased yet, fall back to the intact firmware
            }
            rfReboot();                                // stalled mid-flash, reboot (dfu recovers a bad image)
        }
    }
}

// ---- flash-resident glue (runs before the takeover, while flash is still live) ----

static volatile bool flashPending;

void phoneFlashNoteHello(const uint8_t *payload, uint16_t len)
{
    if (len >= 5 && rd32(payload) == PHONE_FLASH_MAGIC && payload[4] == PF_HELLO) {
        flashPending = true;
    }
}

bool phoneFlashPending(void)
{
    return flashPending;
}

// does not return on success (the engine reboots into the new firmware, or back to normal)
void phoneFlashRun(void)
{
    // must be set here: after the takeover the flash-resident persistent code is gone. the rtc flag
    // survives the reflash and brings the fc back up in phone-config so the host reconnects.
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_PHONECONFIG_REQUEST);

    __disable_irq();

    // verify reads must hit flash, not stale cache lines over reprogrammed sectors
    SCB_DisableDCache();
    SCB_DisableICache();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    for (uint32_t *s = _siram_flash, *d = _sram_flash; d < _eram_flash; ) {
        *d++ = *s++;
    }
    __DSB();
    __ISB();

    rfMain();

    // host never committed; reboot into the untouched firmware
    __DSB();
    SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    for (;;) { }
}
