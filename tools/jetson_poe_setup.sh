#!/bin/bash
# jetson_poe_setup.sh: make the Jetson settings that the PoE cameras need survive a reboot.
#
#   sudo bash tools/jetson_poe_setup.sh            # apply (idempotent; safe to run again)
#   bash tools/jetson_poe_setup.sh --check         # only report the current state
#
# What it sets, and why (see README, "Jetson Orin Nano over PoE"):
#   1. UDP socket buffers (/etc/sysctl.d/60-dds.conf). With the default 212 kB, five
#      640x360 streams overflowed the receive buffers and whole cameras delivered no frames.
#   2. jetson_clocks at every boot (systemd unit). Without it the CPU drops to 730 MHz;
#      it is also the configuration the E6 latency was measured with.
#   3. A NetworkManager profile "poe-camaras" on the camera interface: 192.168.11.1/24,
#      MTU 9000 (the D555 needs jumbo frames), connected automatically at boot.
#      For internet on the same port, bring the DHCP profile up by hand:
#      sudo nmcli con up <dhcp-profile>, and back with sudo nmcli con up poe-camaras.
#   4. ROS 2 environment for the user who ran sudo (~/.bashrc): Humble, CycloneDDS
#      (librealsense's own Fast-DDS conflicts with rmw_fastrtps), and the camera workspace.
#
# Environment overrides: IFACE (default: the wired interface carrying 192.168.11.x, else the
# first en* interface), CAM_NET (192.168.11.1/24), WS (~/magdalene_ws).
set -uo pipefail

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1
TARGET_USER=${SUDO_USER:-$(id -un)}
TARGET_HOME=$(getent passwd "$TARGET_USER" | cut -d: -f6)
CAM_NET=${CAM_NET:-192.168.11.1/24}
WS=${WS:-$TARGET_HOME/magdalene_ws}
IFACE=${IFACE:-$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.11\./ {print $2; exit}')}
if [ -z "${IFACE:-}" ]; then
    for d in /sys/class/net/en*; do [ -e "$d" ] && IFACE=${d##*/} && break; done
fi
SYSCTL_FILE=/etc/sysctl.d/60-dds.conf
UNIT=/etc/systemd/system/jetson-clocks.service
MARK="# >>> magdalena PoE cameras >>>"

ok()   { printf '  [ok]   %s\n' "$*"; }
bad()  { printf '  [--]   %s\n' "$*"; }
step() { printf '\n%s\n' "$*"; }

check() {
    step "State on $(hostname) (user $TARGET_USER, interface ${IFACE:-?})"
    local rmem; rmem=$(sysctl -n net.core.rmem_max)
    [ "$rmem" -ge 134217728 ] && ok "net.core.rmem_max = $rmem" || bad "net.core.rmem_max = $rmem (needs >= 134217728)"
    systemctl is-enabled jetson-clocks.service >/dev/null 2>&1 && ok "jetson-clocks.service enabled" \
        || bad "jetson-clocks.service not enabled"
    if [ -n "${IFACE:-}" ]; then
        local mtu; mtu=$(cat /sys/class/net/"$IFACE"/mtu)
        [ "$mtu" -ge 9000 ] && ok "$IFACE MTU $mtu" || bad "$IFACE MTU $mtu (needs 9000)"
        ip -o -4 addr show dev "$IFACE" | grep -q " ${CAM_NET%/*}/" && ok "$IFACE has ${CAM_NET}" \
            || bad "$IFACE has no ${CAM_NET}"
    fi
    nmcli -t -f NAME con show 2>/dev/null | grep -qx poe-camaras && ok "NetworkManager profile poe-camaras exists" \
        || bad "no NetworkManager profile poe-camaras"
    grep -qF "$MARK" "$TARGET_HOME/.bashrc" 2>/dev/null && ok "$TARGET_HOME/.bashrc has the ROS 2 / CycloneDDS block" \
        || bad "$TARGET_HOME/.bashrc has no ROS 2 / CycloneDDS block"
    dpkg -s ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1 && ok "ros-humble-rmw-cyclonedds-cpp installed" \
        || bad "ros-humble-rmw-cyclonedds-cpp missing (sudo apt install ros-humble-rmw-cyclonedds-cpp)"
    local lrs; lrs=$(readlink -f /usr/local/lib/librealsense2.so 2>/dev/null)
    case "$lrs" in *2.58*|*2.59*|*2.6*) ok "librealsense $lrs";; *) bad "librealsense '${lrs:-none}' (needs >= 2.58.2 with DDS)";; esac
    local reach=0
    for i in 55 56 57 58 59; do ping -c1 -W1 192.168.11.$i >/dev/null 2>&1 && reach=$((reach + 1)); done
    [ "$reach" -eq 5 ] && ok "5/5 cameras answer ping" || bad "$reach/5 cameras answer ping"
}

if [ "$CHECK" -eq 1 ]; then check; exit 0; fi
[ "$(id -u)" -eq 0 ] || { echo "run with sudo (or use --check)"; exit 1; }
[ -n "${IFACE:-}" ] || { echo "no wired interface found; set IFACE=<name>"; exit 1; }

step "1. UDP buffers -> $SYSCTL_FILE"
cat > "$SYSCTL_FILE" <<'EOF'
# Buffers for DDS over UDP (RealSense D555 over PoE). One 640x360 frame is ~0.5-0.7 MB.
net.core.rmem_max=2147483647
net.core.rmem_default=134217728
net.core.wmem_max=2147483647
net.core.wmem_default=134217728
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sysctl -q --system && ok "applied"

step "2. jetson_clocks at boot -> $UNIT"
cat > "$UNIT" <<'EOF'
[Unit]
Description=Lock Jetson clocks (jetson_clocks) after the power mode is set
After=nvpmodel.service multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload && systemctl enable --now jetson-clocks.service >/dev/null 2>&1 && ok "enabled and started"

step "3. NetworkManager profile poe-camaras on $IFACE"
if nmcli -t -f NAME con show | grep -qx poe-camaras; then
    nmcli con mod poe-camaras connection.interface-name "$IFACE" ipv4.method manual ipv4.addresses "$CAM_NET" \
        ipv6.method disabled 802-3-ethernet.mtu 9000 connection.autoconnect yes connection.autoconnect-priority 10
    ok "updated"
else
    nmcli con add type ethernet ifname "$IFACE" con-name poe-camaras ipv4.method manual ipv4.addresses "$CAM_NET" \
        ipv6.method disabled 802-3-ethernet.mtu 9000 connection.autoconnect yes connection.autoconnect-priority 10 >/dev/null
    ok "created"
fi
nmcli con up poe-camaras >/dev/null && ok "active (other profiles on $IFACE are now down; see the header for internet)"

step "4. ROS 2 environment in $TARGET_HOME/.bashrc"
if grep -qF "$MARK" "$TARGET_HOME/.bashrc" 2>/dev/null; then
    ok "already present"
else
    cat >> "$TARGET_HOME/.bashrc" <<EOF

$MARK
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
# <<< magdalena PoE cameras <<<
EOF
    chown "$TARGET_USER": "$TARGET_HOME/.bashrc"
    ok "added (open a new terminal)"
fi

check
