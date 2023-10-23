"""
############################################################################
#
# Copyright (c) Waybyte Solutions
#
# MT6261/MT2503 Flasher in Python
#
# Based on MT6261 Flash Utility By Georgi Angelov
#
############################################################################
#
# Copyright (C) 2019 Georgi Angelov. All rights reserved.
# Author: Georgi Angelov <the.wizarda@gmail.com> WizIO
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name WizIO nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
# Dependency:
#      https://github.com/pyserial/pyserial/tree/master/serial
############################################################################

"""

import argparse
import logging
import serial.serialutil as serialutil
import struct
import sys
import time
from binascii import hexlify
from collections import namedtuple
from pathlib import Path
from serial import Serial

APP_VER = "0.2.0"

NONE = ""
CONF = b"\x69"
STOP = b"\x96"
ACK = b"\x5A"
NACK = b"\xA5"

CMD_READ_16 = b"\xA2"
CMD_READ16 = b"\xD0"
CMD_READ32 = b"\xD1"
CMD_WRITE16 = b"\xD2"
CMD_WRITE32 = b"\xD4"
CMD_JUMP_DA = b"\xD5"
CMD_SEND_DA = b"\xD7"
CMD_SEND_EPP = b"\xD9"

DA_SYNC = b"\xC0"
DA_FORMAT_FAT = b"\xB8"
DA_CONFIG_EMI = b"\xD0"
DA_POST_PROCESS = b"\xD1"
DA_SPEED = b"\xD2"
DA_MEM = b"\xD3"
DA_FORMAT = b"\xD4"
DA_WRITE = b"\xD5"
DA_READ = b"\xD6"
DA_WRITE_REG16 = b"\xD7"
DA_READ_REG16 = b"\xD8"
DA_FINISH = b"\xD9"
DA_GET_DSP_VER = b"\xDA"
DA_ENABLE_WATCHDOG = b"\xDB"
DA_NFB_WRITE_BLOADER = b"\xDC"
DA_NAND_IMAGE_LIST = b"\xDD"
DA_NFB_WRITE_IMAGE = b"\xDE"
DA_NAND_READPAGE = b"\xDF"

DA_CLEAR_POWERKEY_IN_META_MODE_CMD = b"\xB9"
DA_ENABLE_WATCHDOG_CMD = b"\xDB"
DA_GET_PROJECT_ID_CMD = b"\xEF"


Firmware = namedtuple("Firmware", "addr,size,type,data")


def ASSERT(f, s):
    if not f:
        logger.error(s)
        exit(2)


def hexs(s):
    return hexlify(s).decode("ascii").upper()


HEXS01 = hexs(b"\0\1")


class CustomFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    green = "\x1b[32;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format_d = "%(asctime)s [%(name)s]: %(levelname)s: %(message)s"
    format = "%(asctime)s [%(name)s]: %(message)s"

    FORMATS = {
        logging.DEBUG: f"{grey}{format_d}{reset}",
        logging.INFO: f"{green}{format}{reset}",
        logging.WARNING: f"{yellow}{format_d}{reset}",
        logging.ERROR: f"\n{red}{format_d}{reset}\n",
        logging.CRITICAL: f"{bold_red}{format_d}{reset}",
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt="%H:%M:%S")
        return formatter.format(record)


class progressbar:
    def __init__(self, prefix="", total=100, size=32, f=sys.stdout):
        self.prefix = prefix
        self.total = total
        self.count = 0
        self.size = size
        self.file = f
        self.isatty = f.isatty()
        self.hash = -1

    def __enter__(self):
        self.update(0)
        return self

    def __exit__(self, exc, exc_value, exc_tb):
        wr = "| 100%\n" if not self.isatty else "\n"
        self.file.write(wr)
        self.file.flush()

    def update(self, inc: int):
        self.count += inc
        x = int(self.size * self.count / self.total)
        per = int(self.count * 100 / self.total)
        if self.isatty:
            self.file.write(
                f"{self.prefix:<20s} [{'#' * x}{'.' * (self.size - x)}] {per}%\r"
            )

        elif self.hash != x:
            if self.hash == -1:
                self.file.write(f"{self.prefix}\n|{'#' * x}")
            else:
                self.file.write("#" * (x - self.hash))
            self.hash = x
        self.file.flush()


class MT6261:
    def open(self):
        timeout = 10
        port_opened = False
        while not port_opened:
            try:
                self.s = Serial(self.port, 115200)
                port_opened = self.s.is_open
            except serialutil.SerialException as ex:
                timeout -= 0.5
                ASSERT(timeout > 0, ex)
                time.sleep(0.5)

    def crc_word(self, data: bytes, chs: int = 0) -> int:
        chs += sum((d & 0xFF for d in data))
        return chs & 0xFFFF

    def send(self, data: bytes, sz: int = 0) -> bytes:
        r = b""
        if len(data):
            logger.debug(f"--> {hexs(data)}")
            self.s.write(data)
        if sz > 0:
            r = self.s.read(sz)
            logger.debug(f"<-- {hexs(r)}")
        return r

    def cmd(self, cmd: bytes, sz: int = 0) -> bytes:
        r = b""
        size = len(cmd)
        if size > 0:
            r = self.send(cmd, size)
            ASSERT(r == cmd, f"Command response fail: {hexs(cmd)} != {hexs(r)}")
        if sz > 0:
            r = self.s.read(sz)
            logger.debug(f"<-- {hexs(r)}")
        return r

    def da_read16(self, addr: int, sz: int = 1) -> tuple:
        r = self.cmd(CMD_READ_16 + struct.pack(">II", addr, sz), sz * 2)
        return struct.unpack(">" + sz * "H", r)

    def da_write16(self, addr: int, val: int):
        r = self.cmd(CMD_WRITE16 + struct.pack(">II", addr, 1), 2)
        ASSERT(r == b"\0\1", f"WR16 CMD {hexs(r)} != {HEXS01}")
        r = self.cmd(struct.pack(">H", val), 2)
        ASSERT(r == b"\0\1", f"WR16 VAL {hexs(r)} != {HEXS01}")

    def da_write32(self, addr: int, val: int):
        r = self.cmd(CMD_WRITE32 + struct.pack(">II", addr, 1), 2)
        ASSERT(r == b"\0\1", f"WR32 CMD {hexs(r)} != {HEXS01}")
        r = self.cmd(struct.pack(">I", val), 2)
        ASSERT(r == b"\0\1", f"WR32 VAL {hexs(r)} != {HEXS01}")

    def da_send_da(
        self,
        address: int,
        size: int,
        data: bytes,
        block: int = 4096,
        update_cb: callable = None,
    ):
        r = self.cmd(CMD_SEND_DA + struct.pack(">III", address, size, block), 2)
        ASSERT(r == b"\0\0", f"SEND DA CMD {hexs(r)} != 0x0000")
        while data:
            self.s.write(data[:block])
            data = data[block:]
            if update_cb:
                update_cb(block)
        self.cmd(b"", 4)  # checksum

    def sendFlashInfo(self, offset: int):
        for _ in range(512):
            data = self.bl[offset : offset + 36]
            ASSERT(data[:4] != b"\xFF\xFF\0\0", f"Invalid flash info: {hexs(data[:4])}")
            offset += 36
            r = self.send(data, 1)
            if r == ACK:
                r = self.cmd(b"", 2)
                ASSERT(r == b"\xA5\x69", f"Flashinfo END: {hexs(r)}")
                break
            ASSERT(r == CONF, f"Flashinfo ACK Fail: {hexs(r)}")

    def loadBootLoader(self, fname: str = ""):
        fname = Path(__file__).resolve().parent / fname
        ASSERT(fname.is_file(), f"Missing download agent: {fname}")
        self.bl = Path(fname).read_bytes()

    def connect(self, timeout: int = 30):
        self.s.timeout = 0.02
        logger.info("Please reset the device.\nWaiting......")
        sys.stdout.flush()
        start = time.time()
        mtk_found = False
        while not mtk_found:
            self.s.write(b"\xA0")
            if self.s.read(1) == b"\x5F":
                self.s.write(b"\x0A\x50\x05")
                ASSERT(self.s.read(3) == b"\xF5\xAF\xFA", "BOOT")
                mtk_found = True

            ASSERT((time.time() - start) < timeout, "Timeout")

        logger.info("Connected")
        self.s.timeout = 1.0
        BB_CPU_HW = self.da_read16(0x80000000)[0]  # BB_CPU_HW = CB01
        BB_CPU_SW = self.da_read16(0x80000004)[0]  # BB_CPU_SW = 0001
        BB_CPU_ID = self.da_read16(0x80000008)[0]  # BB_CPU_ID = 6261
        BB_CPU_SB = self.da_read16(0x8000000C)[0]  # BB_CPU_SB = 8000
        logger.debug(f"{BB_CPU_HW} {BB_CPU_SW} {BB_CPU_ID} {BB_CPU_SB}")
        self.da_write16(0xA0700A28, 0x4010)  # 01
        self.da_write16(0xA0700A00, 0xF210)  # 02
        self.da_write16(0xA0030000, 0x2200)  # 03
        self.da_write16(0xA071004C, 0x1A57)  # 04
        self.da_write16(0xA071004C, 0x2B68)  # 05
        self.da_write16(0xA071004C, 0x042E)  # 06
        self.da_write16(0xA0710068, 0x586A)  # 07
        self.da_write16(0xA0710074, 0x0001)  # 08
        self.da_write16(0xA0710068, 0x9136)  # 09
        self.da_write16(0xA0710074, 0x0001)  # 10
        self.da_write16(0xA0710000, 0x430E)  # 11
        self.da_write16(0xA0710074, 0x0001)  # 12
        self.da_write32(0xA0510000, 0x00000002)  # ???
        ASSERT(
            BB_CPU_ID == 0x6261, f"Flasher does not support this SoC: {BB_CPU_ID:04x}"
        )
        logger.info("MT6261 found")
        self.loadBootLoader("mt6261_da.bin")

    def da_start(self):
        DA = (
            # offset  size     address     block
            (0x00000, 0x00718, 0x70007000, 0x400),
            (0x00718, 0x1E5C8, 0x10020000, 0x800),
        )
        with progressbar("Download DA", DA[0][1] + DA[1][1]) as pb:
            # SEND_DA_1 and SEND_DA_2
            for ofs, size, addr, block in DA:
                data = self.bl[ofs : ofs + size]
                self.da_send_da(addr, size, data, block, update_cb=pb.update)

            # CMD_JUMP_DA
            r = self.cmd(CMD_JUMP_DA + struct.pack(">I", DA[0][2]), 2)  # D5-
            ASSERT(r == b"\0\0", f"DA JUMP Fail: {hexs(r)}")

            # <-- C003028E DA_INFO: 0xC0 , Ver : 3.2 , BBID : 0x8E
            r = self.cmd(b"", 4)
            self.send(
                b"\xa5\x05\xfe\x00\x08\x00\x70\x07\xff\xff\x02\x00\x00\x01\x08", 1
            )

            # FLASH ID INFOS
            self.sendFlashInfo(DA[1][0] + DA[1][1])
            self.send(b"\0\0\0\0", 256)  # EMI_SETTINGS ??

    def da_changebaud(self, baud: int = 460800):
        if baud == 115200:
            return

        speed_table = {
            921600: b"\x01",
            460800: b"\x02",
            230400: b"\x03",
            115200: b"\x04",
        }
        DEFAULT_BAUDRATE = speed_table[460800]
        r = self.send(DA_SPEED + speed_table.get(baud, DEFAULT_BAUDRATE) + b"\x01", 1)
        ASSERT(r == ACK, f"DA Change Baud CMD ACK Fail: {hexs(r)}")
        self.send(ACK)
        self.s.baudrate = baud
        time.sleep(0.2)
        i = 0
        while self.send(DA_SYNC, 1) != DA_SYNC and i < 10:
            time.sleep(0.02)
            i += 1

        ASSERT(i < 10, "DA SPEED sync fail")
        ASSERT(self.send(ACK, 1) == ACK, "DA SPEED ACK fail")
        for i in range(256):
            loop_val = struct.pack(">B", i)
            ASSERT(self.send(loop_val, 1) == loop_val, "DA SPEED Loop fail")

    # NACK: disable FOTA feature
    def da_mem(self, fws: list[Firmware], fota: bytes = NACK):
        self.send(DA_MEM + fota + struct.pack(">B", len(fws)))

        for addr, size, ftype, _ in fws:
            sa = addr & 0x07FFFFFF
            ea = sa + size - 1
            r = self.send(struct.pack(">III", sa, ea, ftype), 1)
            ASSERT(
                r == ACK,
                f"DA_MEM ACK: send {sa}:{ea}:{ftype}; read {hexs(r)}, but expected {hexs(ACK)}",
            )

        r = struct.unpack(">BB", self.send(NONE, 2))  # filecount + ACK
        # ASSERT(r[0] == file_count, "File count does not match")

        for i in range(len(fws)):
            # Format Ack Count for each file
            format_acks = struct.unpack(">I", self.send(NONE, 4))[0]
            with progressbar(f"Pre-Format {i}", format_acks + 1) as pb:
                for i in range(format_acks):
                    ASSERT(self.send(NONE, 1) == ACK, "Firmware memory format failed")
                    pb.update(1)
                pb.update(1)

        ASSERT(self.send(NONE, 1) == ACK, "Firmware memory format failed 2")

    def da_write(self, block: int = 4096):
        ASSERT(self.send(DA_WRITE, 1) == ACK, "DA_WRITE ACK")
        # Sequential Erase (0x1). (0x0) for Best-Effort Erase, packet_length
        r = self.send(struct.pack(">BI", 0, block), 2)
        ASSERT(r == ACK + ACK, "DA_WRITE OK")

    def da_write_data(self, firmwares, block=4096, update_cb: callable = None):
        crc = []
        for _, _, _, data in firmwares:
            w = 0
            crc.append(0)
            while data:
                self.s.write(ACK)
                self.s.write(data[:block])
                w = self.crc_word(data[:block])
                r = self.send(struct.pack(">H", w), 1)
                if r == CONF:
                    if update_cb:
                        update_cb(len(data[:block]))
                    crc.append(crc.pop() + w)
                    data = data[block:]
                elif r == NACK:
                    # need to wait for ack before sending next packet
                    start_time = time.time()
                    while self.send(NONE, 1) != ACK:
                        ASSERT(
                            (time.time() - start_time) < 60,
                            "Firmware Data write timeout",
                        )
                    self.s.write(CONF)
                else:
                    ASSERT(False, "Firmware fail")

        ack_count = 0
        start_time = time.time()
        while ack_count != 3:
            if self.send(NONE, 1) == ACK:
                ack_count += 1
            ASSERT((time.time() - start_time) < 10, "Firmware Write Error")

        for d in crc:  # range(count):
            r = self.send(struct.pack(">H", d & 0xFFFF), 1)
            ASSERT(r == ACK, "Firmware write ack failed")
        # <-- 14175A  is error

    def printVersion(self):
        self.send(DA_GET_PROJECT_ID_CMD, 1)
        r = self.send(DA_GET_PROJECT_ID_CMD, 256)
        r = r[:24].rstrip(b"\0").lstrip(b"\0")
        logger.info(f"Version {hexs(r)}")

    def da_reset(self):
        self.send(DA_CLEAR_POWERKEY_IN_META_MODE_CMD, 1)  # <-- 5A
        self.send(b"\xC9\x00", 1)  # ???<-- 5A
        self.send(
            DA_ENABLE_WATCHDOG_CMD + b"\x01\x40\x00\x00\x00\x00", 1
        )  # <-- 5A, RESET

    def openApplication(self, check: bool = True):
        fw_list = []
        for fw in self.firmware:
            _data = fw.read()
            _type = struct.unpack("<H", _data[0x18:0x1A])[0]
            _addr, _size = struct.unpack("<II", _data[0x1C:0x24])
            app_size = len(_data)
            ASSERT(_size == app_size, "APP: Size mismatch")
            ASSERT(app_size > 0x40, "APP: Invalid size.")
            if check:
                ASSERT(
                    _data[:3].decode() == "MMM", "APP: Invalid header 'MMM' expected."
                )
                ASSERT(
                    _data[8:17].decode() == "FILE_INFO",
                    "APP: Invalid header 'FILE_INFO' expected.",
                )

            fw_list.append(Firmware(_addr, _size, _type, _data))

        # Sort by address
        fw_list.sort(key=lambda x: x.addr)
        return fw_list

    def uploadApplication(self):
        self.da_mem(self.fw_list)

        with progressbar(
            "Download Firmware", sum([fw.size for fw in self.fw_list])
        ) as pb:
            self.da_write()
            self.da_write_data(self.fw_list, update_cb=pb.update)

        logger.info("Finished")

    def formatFAT(self):
        self.send(DA_FORMAT_FAT + b"\x00\x01")
        self.send(NONE, 4)  # 00000000
        addr = struct.unpack(">I", self.send(NONE, 4))[0]
        len = struct.unpack(">I", self.send(NONE, 4))[0]
        self.send(NONE, 16)
        ASSERT(self.send(NONE, 2) == ACK + ACK, "Format FAT ack failed")

        with progressbar(f"Format Fat [0x{addr:08x} : 0x{(addr + len - 1):08x}]") as pb:
            start_time = time.time()
            pre = 0
            while (time.time() - start_time) < 20:
                self.send(NONE, 4)
                curr = self.send(NONE, 1)[0]
                pb.update(curr - pre)
                pre = curr
                self.send(ACK)
                if curr == 100:
                    break

    def da_finish(self):
        self.send(DA_FINISH, 1)  # ACK
        self.send(b"\x00\x00\x00\x00", 1)  # NAK


######################################################################


def upload_app(flasher: MT6261):
    flasher.fw_list = flasher.openApplication(True)
    flasher.open()
    flasher.connect()
    flasher.da_start()
    flasher.da_changebaud(flasher.baud)
    flasher.uploadApplication()
    if flasher.opt == 0:
        flasher.formatFAT()
    if flasher.no_reset:
        flasher.da_finish()
    else:
        flasher.da_reset()


if __name__ == "__main__":
    flasher = MT6261()
    parser = argparse.ArgumentParser(description="MT6261/MT2503 Flash Tool")
    parser.add_argument("-p", "--port", required=True, help="Serial port for flashing.")
    parser.add_argument(
        "-b", "--baud", type=int, default=460800, help="Serial port baudrate."
    )
    parser.add_argument(
        "-o",
        "--opt",
        type=int,
        default=1,
        help="""Flash Options:\n
    0: Download Firmware and Format\n
    1: Download Firmware only""",
    )
    parser.add_argument(
        "-n", "--no-reset", help="Do not reset after flashing", action="store_true"
    )
    parser.add_argument(
        "firmware",
        nargs="+",
        type=argparse.FileType("rb"),
        help="Firmware binary file.",
    )
    parser.add_argument(
        "-v",
        "--version",
        action="version",
        version="MT6261/MT2503 Flash Tool v" + APP_VER,
    )
    parser.add_argument("-d", "--debug", action="store_true")
    parser.parse_args(namespace=flasher)

    logger = logging.getLogger("MTKFlasher")
    logger.setLevel(logging.INFO)
    if flasher.debug:
        logger.setLevel(logging.DEBUG)
    console = logging.StreamHandler()
    console.setFormatter(CustomFormatter())
    logger.addHandler(console)

    upload_app(flasher)
