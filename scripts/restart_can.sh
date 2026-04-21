#!/bin/bash
# Bring up a single Piper follower CAN bus as interface "left_piper" at 1 Mbit/s.
# Works with:
#   - Native SocketCAN USB (e.g. gs_usb): kernel already exposes can0 / can1
#   - CANable2 in serial/SLCAN mode (16d0:117e → /dev/ttyACM*): uses slcand
#
# Optional env:
#   BITRATE   default 1000000
#   CAN_NAME  default left_piper

BITRATE="${BITRATE:-1000000}"
CAN_NAME="${CAN_NAME:-left_piper}"

# slcan -s# for 1M=8, 500k=6, 250k=5 (see slcand(8))
slcan_speed_for_bitrate() {
	case "$BITRATE" in
	1000000) echo 8 ;;
	800000) echo 7 ;;
	500000) echo 6 ;;
	250000) echo 5 ;;
	125000) echo 4 ;;
	100000) echo 3 ;;
	50000) echo 2 ;;
	20000) echo 1 ;;
	10000) echo 0 ;;
	*)
		echo "Unsupported BITRATE=$BITRATE (set one of 1000000,800000,500000,...)" >&2
		return 1
		;;
	esac
}

find_canable_tty() {
	local tty
	for tty in /dev/ttyACM[0-9]*; do
		[ -e "$tty" ] || continue
		if udevadm info -q property -n "$tty" 2>/dev/null | grep -qi '^ID_VENDOR_ID=16d0'; then
			if udevadm info -q property -n "$tty" 2>/dev/null | grep -qi '^ID_MODEL_ID=117e'; then
				echo "$tty"
				return 0
			fi
		fi
	done
	return 1
}

first_can_iface() {
	ip -br link show type can 2>/dev/null | awk '{print $1}' | grep -vx "$CAN_NAME" | head -1
}

refresh_iface() {
	local iface=$1
	sudo ip link set "$iface" down
	sudo ip link set "$iface" type can bitrate "$BITRATE"
	sudo ip link set "$iface" up
}

# Already configured: just reset bitrate and up
if ip link show "$CAN_NAME" &>/dev/null; then
	refresh_iface "$CAN_NAME"
	exit 0
fi

# Stale slcand from a previous CANable session (ignore errors)
sudo killall slcand 2>/dev/null || true
sleep 0.3

# Native CAN adapter already visible
first=$(first_can_iface)
if [ -n "$first" ]; then
	sudo ip link set "$first" down
	sudo ip link set "$first" type can bitrate "$BITRATE"
	sudo ip link set "$first" name "$CAN_NAME"
	sudo ip link set "$CAN_NAME" up
	exit 0
fi

# No SocketCAN yet: try CANable2 over SLCAN
canable_tty=$(find_canable_tty) || true
if [ -z "${canable_tty:-}" ]; then
	echo "No CAN interface found and no CANable2 (USB 16d0:117e) tty under /dev/ttyACM*." >&2
	echo "Plug your adapter, or install/use firmware that exposes SocketCAN (e.g. gs_usb)." >&2
	exit 1
fi

speed=$(slcan_speed_for_bitrate) || exit 1

sudo modprobe slcan
sudo slcand -o -c -s"$speed" "$canable_tty" can0
sleep 1

if ! ip link show can0 &>/dev/null; then
	echo "slcand did not create can0. Is CANable firmware in SLCAN mode?" >&2
	exit 1
fi

sudo ip link set can0 down
sudo ip link set can0 type can bitrate "$BITRATE"
sudo ip link set can0 name "$CAN_NAME"
sudo ip link set "$CAN_NAME" up
