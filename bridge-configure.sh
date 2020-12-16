#!/usr/bin/env bash

set -e

[ $EUID -ne 0 ] && echo "run as root" >&2 && exit 1

####################################
# Update these variables as needed #
####################################

ssid="LOPES"
key_mgmt=NONE
country="US"

##########################################################
# You should not need to update anything below this line #
##########################################################

# parprouted  - Proxy ARP IP bridging daemon
# dhcp-helper - DHCP/BOOTP relay agent

apt update && apt install -y parprouted dhcp-helper

systemctl stop dhcp-helper
systemctl enable dhcp-helper

# Prevent the networking and dhcpcd service from running. networking.service
# is a debian-specific package and not the same as systemd-network. Disable the
# dhcpcd service as well.
systemctl mask networking.service dhcpcd.service

# This tells resolvconf to ignore whenever some daemon tries to modify the
# resolv.conf file - https://wiki.debian.org/resolv.conf
sed -i '1i resolvconf=NO' /etc/resolvconf.conf

systemctl enable systemd-networkd.service systemd-resolved.service

# Use systemd-resolved to handle the /etc/resolf.conf config
ln -sf /run/systemd/resolve/resolv.conf /etc/resolv.conf

# Create the WiFi config to connect to the upstream network like normal
cat > /etc/wpa_supplicant/wpa_supplicant-wlan0.conf <<EOF
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=${country}

network={
    ssid="${ssid}"
    key_mgmt="${key_mgmt}"
}
EOF

# Service configuration for standard WiFi connection
chmod 600 /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
systemctl disable wpa_supplicant.service
systemctl enable wpa_supplicant@wlan0.service

cat > /etc/systemd/network/08-wlan0.network <<EOF
[Match]
Name=wlan0
[Network]
DHCP=yes
IPForward=yes
EOF

cat > /etc/default/dhcp-helper <<EOF
DHCPHELPER_OPTS="-b wlan0"
EOF

cat <<'EOF' >/etc/avahi/avahi-daemon.conf
[server]
use-ipv4=yes
use-ipv6=yes
ratelimit-interval-usec=1000000
ratelimit-burst=1000

[wide-area]
enable-wide-area=yes

[publish]
publish-hinfo=no
publish-workstation=no

[reflector]
enable-reflector=yes

[rlimits]
EOF

# Create a helper script to get an adapter's IP address
cat <<'EOF' >/usr/bin/get-adapter-ip
#!/usr/bin/env bash

/sbin/ip -4 -br addr show ${1} | /bin/grep -Po "\\d+\\.\\d+\\.\\d+\\.\\d+"
EOF
chmod +x /usr/bin/get-adapter-ip

# I have to admit, I do not understand ARP and IP forwarding enough to explain
# exactly what is happening here. I am building off the work of others. In short
# this is a service to forward traffic from WiFi to Ethernet
cat <<'EOF' >/etc/systemd/system/parprouted.service
[Unit]
Description=proxy arp routing service
Documentation=https://raspberrypi.stackexchange.com/q/88954/79866

[Service]
Type=forking
# Restart until wlan0 gained carrier
Restart=on-failure
RestartSec=5
TimeoutStartSec=30
ExecStartPre=/lib/systemd/systemd-networkd-wait-online --interface=wlan0 --timeout=6 --quiet
ExecStartPre=/bin/echo 'systemd-networkd-wait-online: wlan0 is online'
# clone the dhcp-allocated IP to eth0 so dhcp-helper will relay for the correct subnet
ExecStartPre=/bin/bash -c '/sbin/ip addr add $(/usr/bin/get-adapter-ip wlan0)/32 dev eth0'
ExecStartPre=/sbin/ip link set dev eth0 up
ExecStartPre=/sbin/ip link set wlan0 promisc on
ExecStart=-/usr/sbin/parprouted eth0 wlan0
ExecStopPost=/sbin/ip link set wlan0 promisc off
ExecStopPost=/sbin/ip link set dev eth0 down
ExecStopPost=/bin/bash -c '/sbin/ip addr del $(/usr/bin/get-adapter-ip eth0)/32 dev eth0'

[Install]
WantedBy=wpa_supplicant@wlan0.service
EOF

systemctl daemon-reload
systemctl enable parprouted.service

systemctl start wpa_supplicant@wlan0 dhcp-helper systemd-networkd systemd-resolved
