#!/bin/sh
set -eu

IFACE="${IFACE:-end0}"
ADDR="${ADDR:-10.10.10.2/24}"

ip link set "$IFACE" up
if ! ip addr show dev "$IFACE" | grep -q " ${ADDR%/*}/"; then
  ip addr add "$ADDR" dev "$IFACE"
fi

printf 'Configured %s with %s without default gateway.\n' "$IFACE" "$ADDR"
ip -br addr show dev "$IFACE"
