#!/usr/bin/env python3
"""Pin the camera DTB in extlinux.conf, keeping the current config as a fallback.

Bypasses Arducam's broken jetson-io overlay installer ("Multiple DTBs found")
by naming the device tree explicitly instead of letting UEFI pick a fallback.

Produces two boot entries:
  camera   (DEFAULT) -- same kernel, with FDT pointing at the merged camera+SPI DTB
  primary            -- byte-identical to what boots today, as a rescue option

With TIMEOUT 30 and a monitor attached you can select "primary" if the camera
DTB fails to boot, so this is recoverable without reflashing.

Run with sudo. Idempotent: re-running replaces a previous camera entry.
"""
import os
import re
import shutil
import sys
import time

CONF = "/boot/extlinux/extlinux.conf"
DTB = "/boot/arducam/dts/dtb/tegra234-p3768-merged-camera-spi.dtb"

if not os.path.exists(DTB):
    sys.exit(f"ABORT: camera DTB missing: {DTB}")
if os.path.getsize(DTB) < 100_000:
    sys.exit(f"ABORT: camera DTB implausibly small: {os.path.getsize(DTB)} bytes")

src = open(CONF).read()

# Drop any camera entry from a previous run so this stays idempotent.
src = re.sub(r"^LABEL camera\b.*?(?=^LABEL |\Z)", "", src, flags=re.M | re.S)

m = re.search(r"^LABEL primary\b.*?(?=^LABEL |\Z)", src, flags=re.M | re.S)
if not m:
    sys.exit("ABORT: no 'LABEL primary' block found -- refusing to guess")

block = m.group(0)


def line_starting(kw):
    hits = [l for l in block.splitlines() if l.strip().startswith(kw)]
    if not hits:
        sys.exit(f"ABORT: primary block has no {kw} line")
    return hits[0]


linux = line_starting("LINUX")
initrd = line_starting("INITRD")
append = line_starting("APPEND")

if "FDT" in block:
    print("note: primary block already had an FDT line; the fallback keeps it")

backup = f"{CONF}.bak-claude-{time.strftime('%Y%m%d-%H%M%S')}"
shutil.copy2(CONF, backup)
print(f"backed up -> {backup}")

camera_entry = "\n".join([
    "LABEL camera",
    "      MENU LABEL Arducam camera DTB (default)",
    linux,
    f"      FDT {DTB}",
    initrd,
    append,
]) + "\n"

fallback_entry = "\n".join([
    "LABEL primary",
    "      MENU LABEL primary kernel - NO camera DTB (rescue)",
    linux,
    initrd,
    append,
]) + "\n"

head = src[:m.start()]
head = re.sub(r"^DEFAULT\s+\S+", "DEFAULT camera", head, flags=re.M)
if "DEFAULT camera" not in head:
    sys.exit("ABORT: could not set DEFAULT -- no DEFAULT line found")

new = head.rstrip("\n") + "\n\n" + camera_entry + "\n" + fallback_entry

open(CONF, "w").write(new)
print(f"wrote {CONF}\n")
print("=" * 60)
print(new)
