#!/bin/sh
set -eu

SERVER="${SERVER:-192.168.1.11:8765}"
PKG="jetson_vision_export_20260612_0215.tar.gz"
SHA="jetson_vision_export_20260612_0215.tar.gz.sha256"
WORK="/tmp/jetson_vision_import_$$"
TS="$(date +%Y%m%d_%H%M%S)"

if [ "$(id -u)" != "0" ]; then
  echo "Run with sudo: sudo sh $0" >&2
  exit 1
fi

mkdir -p "$WORK"
cd "$WORK"

echo "=== download package ==="
if command -v wget >/dev/null 2>&1; then
  wget -O "$PKG" "http://$SERVER/$PKG"
  wget -O "$SHA" "http://$SERVER/$SHA" || true
elif command -v curl >/dev/null 2>&1; then
  curl -fL -o "$PKG" "http://$SERVER/$PKG"
  curl -fL -o "$SHA" "http://$SERVER/$SHA" || true
else
  echo "missing wget/curl" >&2
  exit 1
fi

if [ -s "$SHA" ] && command -v sha256sum >/dev/null 2>&1; then
  sha256sum -c "$SHA"
fi

echo "=== extract ==="
tar -xzf "$PKG"
SRC="$WORK/jetson_vision_export_20260612_0215"

if [ ! -d "$SRC/home/nvidia/vision_starter" ] || [ ! -d "$SRC/home/nvidia/orbbec_sdk" ]; then
  echo "package missing expected project directories" >&2
  exit 1
fi

echo "=== backup existing target dirs ==="
mkdir -p /home/nvidia
for d in /home/nvidia/vision_starter /home/nvidia/orbbec_sdk; do
  if [ -e "$d" ]; then
    mv "$d" "$d.backup_$TS"
    echo "backup: $d.backup_$TS"
  fi
done

echo "=== install project files ==="
cp -a "$SRC/home/nvidia/vision_starter" /home/nvidia/
cp -a "$SRC/home/nvidia/orbbec_sdk" /home/nvidia/
chown -R nvidia:nvidia /home/nvidia/vision_starter /home/nvidia/orbbec_sdk 2>/dev/null || true
chmod +x /home/nvidia/vision_starter/*.sh /home/nvidia/vision_starter/scripts/*.sh /home/nvidia/orbbec_sdk/*.sh 2>/dev/null || true
chmod +x /home/nvidia/orbbec_sdk/depth_center_json /home/nvidia/orbbec_sdk/depth_grid_daemon 2>/dev/null || true

echo "=== install services and udev rules ==="
cp -a "$SRC/etc/systemd/system/orbbec-depth-grid.service" /etc/systemd/system/
cp -a "$SRC/etc/systemd/system/jetson-vision.service" /etc/systemd/system/
[ -f "$SRC/etc/systemd/system/vision-coco-depth.service" ] && cp -a "$SRC/etc/systemd/system/vision-coco-depth.service" /etc/systemd/system/ || true

if [ -f "$SRC/etc/udev/rules.d/99-obsensor-libusb.rules" ]; then
  cp -a "$SRC/etc/udev/rules.d/99-obsensor-libusb.rules" /etc/udev/rules.d/
elif [ -f /home/nvidia/orbbec_sdk/v1/OrbbecSDK_v1.10.35/Script/99-obsensor-libusb.rules ]; then
  cp -a /home/nvidia/orbbec_sdk/v1/OrbbecSDK_v1.10.35/Script/99-obsensor-libusb.rules /etc/udev/rules.d/
fi
udevadm control --reload-rules 2>/dev/null || true
udevadm trigger 2>/dev/null || true

echo "=== systemd enable ==="
systemctl daemon-reload
systemctl disable --now vision-coco-depth.service 2>/dev/null || true
systemctl enable orbbec-depth-grid.service jetson-vision.service
systemctl restart orbbec-depth-grid.service || true
systemctl restart jetson-vision.service || true

echo "=== environment check ==="
uname -r || true
cat /etc/nv_tegra_release 2>/dev/null || true
python3 --version || true
python3 - <<'PY' || true
mods = ['cv2','numpy','tensorrt']
for m in mods:
    try:
        mod = __import__(m)
        print(m, getattr(mod, '__version__', 'ok'))
    except Exception as e:
        print(m, 'MISSING', e)
PY
[ -x /usr/src/tensorrt/bin/trtexec ] && /usr/src/tensorrt/bin/trtexec --version 2>/dev/null | head -5 || true
ls /dev/video* 2>/dev/null || true
lsusb 2>/dev/null | grep -Ei '2bc5|orbbec|camera' || true

echo "=== service status ==="
systemctl --no-pager --plain status orbbec-depth-grid.service jetson-vision.service 2>/dev/null | sed -n '1,160p' || true

echo "=== latest endpoint smoke test ==="
sleep 2
if command -v curl >/dev/null 2>&1; then
  curl -m 5 http://127.0.0.1:8090/latest.json || true
elif command -v wget >/dev/null 2>&1; then
  wget -T 5 -qO- http://127.0.0.1:8090/latest.json || true
fi

echo
echo "DONE. Installed /home/nvidia/vision_starter and /home/nvidia/orbbec_sdk"
echo "Backups, if any, have suffix .backup_$TS"
echo "Preview URL: http://<target-ip>:8090/"
echo "JSON URL:    http://<target-ip>:8090/latest.json"
