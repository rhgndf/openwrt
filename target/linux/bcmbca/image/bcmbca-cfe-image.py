#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Build NAND images for Broadcom BCA boards booted by CFE from pureUBI slots.

The CFE ROM and CFE RAM (cferam.000) find everything they need in UBI
volumes of the firmware slot. Subcommands:

  bootfs   build the boot-file volume content ("filestruct_full.bin")
  ubi      build the UBI image of a firmware slot (with ubinize)

Record list (METADATA volumes and the boot-file volume)
-------------------------------------------------------
A volume holds a list of records, all integers big endian, with no padding
or alignment anywhere:

  +0    be32  next      bytes from +4 up to and including data_crc,
                        i.e. 12 + len(name) + 1 + len(data) + 4
  +4    be32  data_ofs  offset of data counted from +8: 8 + len(name) + 1
  +8    be32  data_len  len(data)
  +12   be32  hdr_crc   crc(bytes +0 .. +11)
  +16         name      ASCII, then one NUL byte
              data      data_len bytes
              data_crc  be32 crc(name + NUL + data)

The next record starts at +4 + next. The list ends with a be32 0 (next == 0).
crc() is the CRC-32 of zlib/Ethernet (reflected, poly 0xedb88320) with init
0xffffffff and *no* final inversion, i.e. zlib.crc32(x) ^ 0xffffffff, the
same function UBI uses (ubicrc32).

METADATA (vol 1) and METADATACOPY (vol 2) hold the same three records:
committed = "0" or "1" (1 byte), cferam.000 = 3 ASCII digits (the slot
sequence number, 000..999) and squash = "1" (rootfs is squashfs on ubiblock).
The images built here carry the safe uncommitted default (committed "0",
cferam.000 "998"), which never wins over a committed slot. To commit a slot,
rewrite both volumes with cferam.000 = "%03d" % ((seq of the running slot
+ 1) % 1000) and committed = "1".

The boot-file volume (vol 10) holds the records cferam.000 (vendor CFE RAM),
vmlinux.lz and the DTB (e.g. 947622.dtb), sorted by name. vmlinux.lz is a
20 byte header of little endian u32 {load, entry, len(lzma), "BRCM", 0}
followed by an LZMA-alone stream; load is p_vaddr of the first PT_LOAD
segment and entry is e_entry of the kernel ELF. CFE decompresses it to
physical load & 0xffffff (below 0xf00000) and copies the DTB to 0x8000.

UBI volumes, in this order:

  id  name                 type     size
  10  filestruct_full.bin  static   boot-file records
   1  METADATA             dynamic  1 LEB
   2  METADATACOPY         dynamic  1 LEB
   0  rootfs               dynamic  squashfs, used through ubiblock
   3  rootfs_data          dynamic  1 MiB, autoresize
"""

import argparse
import os
import struct
import subprocess
import sys
import tempfile
import zlib


def crc32(data):
    """CRC-32 with init 0xffffffff and no final xor (Broadcom and UBI)."""
    return zlib.crc32(data) ^ 0xffffffff


def fail(msg):
    sys.exit(f"{os.path.basename(sys.argv[0])}: error: {msg}")


def read(path):
    with open(path, "rb") as f:
        return f.read()


def write(path, data):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def record(name, data):
    raw_name = name.encode("ascii") + b"\0"
    head = struct.pack(">III", 12 + len(raw_name) + len(data) + 4,
                       8 + len(raw_name), len(data))
    head += struct.pack(">I", crc32(head))
    return head + raw_name + data + struct.pack(">I", crc32(raw_name + data))


def record_list(items):
    return b"".join(record(n, d) for n, d in items) + struct.pack(">I", 0)


# The state of a freshly written, not yet committed slot (as in stock images)
DEFAULT_METADATA = record_list([("committed", b"0"), ("cferam.000", b"998"),
                                ("squash", b"1")])

# Memory map of the BCM47622 CFE, check it before adding other SoCs
CFE_RAM_BASE = 0xf00000     # CFE RAM, the kernel must end below it
DTB_ADDR = 0x8000
# CFE refuses DTBs above 0x7c00 bytes, but then opens them with 0x800 bytes
# of headroom, which only stays inside its 0x8000 buffer up to 0x7800 bytes.
DTB_MAX = 0x7800


def elf_load_entry(elf):
    """Return (p_vaddr of the first PT_LOAD, e_entry) of a 32 bit LE ELF."""
    if elf[:4] != b"\x7fELF" or elf[4:6] != b"\x01\x01":
        fail("kernel is not a 32 bit little endian ELF file")
    entry, phoff = struct.unpack_from("<II", elf, 24)
    phentsize, phnum = struct.unpack_from("<HH", elf, 42)
    for i in range(phnum):
        p_type, _, p_vaddr = struct.unpack_from("<III", elf,
                                                phoff + i * phentsize)
        if p_type == 1:                                 # PT_LOAD
            return p_vaddr, entry
    fail("kernel ELF has no PT_LOAD segment")


def vmlinux_lz(lzma_data, load, entry):
    props, _, size = struct.unpack_from("<BIQ", lzma_data)
    if props >= 9 * 5 * 5 or size >> 32:
        fail("kernel must be an LZMA-alone stream with the size in its header")
    phys = load & 0xffffff
    if phys < 2 * DTB_ADDR:
        fail(f"kernel load address {load:#x} overlaps the DTB at "
             f"{DTB_ADDR:#x}")
    if phys + size > CFE_RAM_BASE:
        fail(f"kernel ({size} bytes at {phys:#x}) overlaps CFE RAM at "
             f"{CFE_RAM_BASE:#x}")
    return struct.pack("<III4sI", load, entry, len(lzma_data), b"BRCM", 0) \
        + lzma_data


def cmd_bootfs(args):
    dtb = read(args.dtb)
    if len(dtb) > DTB_MAX:
        fail(f"DTB is {len(dtb)} bytes, CFE takes at most {DTB_MAX}")
    cferam = read(args.cferam)
    # Rescue lives in cferam, a broken one in the newest slot can't be
    # recovered. The first word of its 12 byte header is the load address.
    if len(cferam) <= 12 or struct.unpack_from("<I", cferam)[0] == 0:
        fail("cferam does not look like a CFE RAM image")
    load, entry = elf_load_entry(read(args.elf))
    files = {"cferam.000": cferam,
             "vmlinux.lz": vmlinux_lz(read(args.kernel), load, entry),
             args.dtb_name: dtb}
    write(args.output, record_list(sorted(files.items())))


def cmd_ubi(args):
    with tempfile.TemporaryDirectory(dir=os.path.dirname(args.output)) as tmp:
        metadata = os.path.join(tmp, "metadata")
        write(metadata, DEFAULT_METADATA)
        volumes = [
            (10, "filestruct_full.bin", "static", f"image={args.bootfs}"),
            (1, "METADATA", "dynamic", f"image={metadata}"),
            (2, "METADATACOPY", "dynamic", f"image={metadata}"),
            (0, "rootfs", "dynamic", f"image={args.rootfs}"),
            (3, "rootfs_data", "dynamic",
             "vol_size=1MiB\nvol_flags=autoresize"),
        ]
        ini = os.path.join(tmp, "ubinize.ini")
        with open(ini, "w") as f:
            for vol_id, name, vol_type, extra in volumes:
                f.write(f"[{name}]\nmode=ubi\nvol_id={vol_id}\n"
                        f"vol_type={vol_type}\nvol_name={name}\n{extra}\n")
        cmd = ["ubinize", "-o", args.output, "-p", args.peb_size,
               "-m", args.page_size]
        if os.environ.get("SOURCE_DATE_EPOCH"):
            cmd += ["-Q", os.environ["SOURCE_DATE_EPOCH"]]
        subprocess.run(cmd + [ini], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("bootfs", help="build the boot-file volume content")
    p.add_argument("--kernel", required=True, help="LZMA-alone kernel image")
    p.add_argument("--elf", required=True, help="kernel ELF (load/entry)")
    p.add_argument("--dtb", required=True, help="device tree blob")
    p.add_argument("--dtb-name", required=True,
                   help="file name CFE looks up, e.g. 947622.dtb")
    p.add_argument("--cferam", required=True, help="vendor cferam.000")
    p.add_argument("-o", "--output", required=True)
    p.set_defaults(func=cmd_bootfs)

    p = sub.add_parser("ubi", help="build the UBI image of a firmware slot")
    p.add_argument("--bootfs", required=True, help="output of 'bootfs'")
    p.add_argument("--rootfs", required=True, help="squashfs image")
    p.add_argument("--peb-size", required=True, help="ubinize -p")
    p.add_argument("--page-size", required=True, help="ubinize -m")
    p.add_argument("-o", "--output", required=True)
    p.set_defaults(func=cmd_ubi)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
