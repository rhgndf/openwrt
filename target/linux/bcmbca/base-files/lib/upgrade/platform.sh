# SPDX-License-Identifier: GPL-2.0-only
#
# Sysupgrade for CFE booted devices with two pureUBI firmware slots.
#
# The partition parser names the slot CFE booted from "firmware" and the
# other one "backup". The UBI image is written to "backup" and then made the
# slot CFE boots by rewriting its METADATA and METADATACOPY volumes: CFE
# prefers the slot with the higher "cferam.000" sequence number as long as
# that slot is committed. Until then the running slot stays preferred, so an
# interrupted upgrade boots the old firmware again.
#
# The metadata record format is described in the target's
# image/bcmbca-cfe-image.py.

REQUIRE_IMAGE_METADATA=1
RAMFS_COPY_BIN='ubicrc32'
CI_UBIPART=backup

BCMBCA_BOOTFS=filestruct_full.bin

bcmbca_hex() {
	hexdump -v -e '1/1 "%02x"'
}

bcmbca_unhex() {
	printf "$(echo "$1" | sed 's/../\\x&/g')"
}

# crc of the bytes in hex string $1
bcmbca_crc() {
	local crc="$(bcmbca_unhex "$1" | ubicrc32)"

	echo "${crc#0x}"
}

# List the records of metadata volume $1 as "<name> <data>" lines, both in
# hex, the name including its NUL.
bcmbca_meta_list() {
	local hex="$(dd if="$1" bs=2048 count=1 2>/dev/null | bcmbca_hex)"
	local next nlen

	while [ ${#hex} -ge 32 ]; do
		next=$((0x${hex:0:8}))
		[ $next -gt 0 ] && [ $((8 + next * 2)) -le ${#hex} ] || return 0
		[ "$(bcmbca_crc ${hex:0:24})" = "${hex:24:8}" ] || return 1
		nlen=$((0x${hex:8:8} - 8))
		echo "${hex:32:nlen*2} ${hex:32+nlen*2:0x${hex:16:8}*2}"
		hex="${hex:8+next*2}"
	done
}

# Encode a record with name $1 and data $2, as hex
bcmbca_meta_record() {
	local name="$(printf '%s' "$1" | bcmbca_hex)00"
	local data="$(printf '%s' "$2" | bcmbca_hex)"
	local hdr="$(printf '%08x%08x%08x' $((16 + (${#name} + ${#data}) / 2)) \
		$((8 + ${#name} / 2)) $((${#data} / 2)))"

	echo "$hdr$(bcmbca_crc $hdr)$name$data$(bcmbca_crc $name$data)"
}

# Print the cferam.000 sequence number of the slot attached as UBI device $1.
bcmbca_slot_seq() {
	local key="$(printf 'cferam.000' | bcmbca_hex)00"
	local vol data

	for vol in METADATA METADATACOPY; do
		vol="$(nand_find_volume $1 $vol)"
		[ -n "$vol" ] || continue
		data="$(bcmbca_meta_list /dev/$vol | sed -n "s/^$key //p")"
		case "$data" in
		3[0-9]3[0-9]3[0-9])
			# the leading 1 keeps ash from reading the digits as octal
			echo $((1${data:1:1}${data:3:1}${data:5:1} - 1000))
			return 0
			;;
		esac
	done
	return 1
}

# Commit the slot attached as UBI device $1 with sequence number $2, in both
# metadata volumes. The records are those of the images we build.
bcmbca_slot_commit() {
	local meta="$(nand_find_volume $1 METADATA)"
	local copy="$(nand_find_volume $1 METADATACOPY)"
	local out=/tmp/bcmbca-metadata.bin

	[ -n "$meta" ] && [ -n "$copy" ] || return 1

	bcmbca_unhex "$(bcmbca_meta_record committed 1)$(bcmbca_meta_record \
		cferam.000 $(printf '%03d' $2))$(bcmbca_meta_record squash 1)00000000" \
		> $out
	ubiupdatevol /dev/$meta $out && ubiupdatevol /dev/$copy $out
}

# Check that all volumes CFE and OpenWrt need are there. The first open of
# the static boot file volume makes UBI check its data CRC.
bcmbca_slot_check() {
	local name vol

	for name in METADATA METADATACOPY rootfs rootfs_data; do
		[ -n "$(nand_find_volume $1 $name)" ] || {
			v "volume $name is missing"
			return 1
		}
	done
	vol="$(nand_find_volume $1 $BCMBCA_BOOTFS)"
	[ -n "$vol" ] && : < /dev/$vol &&
		[ "$(cat /sys/class/ubi/$vol/corrupted)" = 0 ] || {
		v "volume $BCMBCA_BOOTFS is missing or corrupted"
		return 1
	}
}

platform_check_image() {
	[ "$#" -gt 1 ] && return 1

	[ "$(get_magic_long "$1")" = 55424923 ] || {
		v "Invalid image, a UBI image is required"
		return 1
	}
	[ -n "$(find_mtd_index backup)" ] || {
		v "Cannot find the inactive firmware slot"
		return 1
	}
}

platform_do_upgrade() {
	local seq ubidev

	seq="$(bcmbca_slot_seq "$(nand_find_ubi firmware)")" || {
		v "Cannot read the sequence number of the running slot"
		return 1
	}
	seq=$(((seq + 1) % 1000))

	# ubiformat needs a whole number of erase blocks, drop the fwtool trailer
	fwtool -q -t -s /dev/null "$1"
	fwtool -q -t -i /dev/null "$1"

	sync
	nand_upgrade_ubinized "$1" cat || return 1
	ubidev="$(nand_find_ubi backup)"
	bcmbca_slot_check $ubidev || return 1
	nand_do_restore_config || return 1
	bcmbca_slot_commit $ubidev $seq || {
		v "Cannot update the slot metadata"
		return 1
	}
	v "The new firmware will boot from slot sequence $seq"
}
