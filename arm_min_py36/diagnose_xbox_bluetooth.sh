#!/bin/sh
set -u

LOG=${LOG:-xbox_bluetooth_diagnostic.log}
SCAN_SECONDS=${SCAN_SECONDS:-12}
CMD_TIMEOUT=${CMD_TIMEOUT:-15}
APPLY_XBOX_FIXES=0
RESET_MAC=""
SCAN=0
MONITOR_SECONDS=0
CAPTURE_SECONDS=0
CAPTURE_DIR=""
PIDS_FILE=""

usage() {
    cat <<'EOF'
Usage:
  ./diagnose_xbox_bluetooth.sh
  ./diagnose_xbox_bluetooth.sh --scan
  ./diagnose_xbox_bluetooth.sh --monitor 30
  sudo ./diagnose_xbox_bluetooth.sh --capture 60
  ./diagnose_xbox_bluetooth.sh --reset-device XX:XX:XX:XX:XX:XX
  ./diagnose_xbox_bluetooth.sh --apply-xbox-fixes

Modes:
  default       One-shot Bluetooth diagnosis.
  --scan        Scan for controllers, then run one-shot diagnosis.
  --monitor N   Poll rfkill and bluetoothctl for N seconds in the summary log.
  --capture N   Create a log directory and record rfkill, BlueZ, journalctl,
                dmesg, udev, btmon, dbus, and polled adapter state for N seconds.

What it checks:
  - bluetooth service and rfkill state
  - systemd-rfkill saved state and units
  - adapter state reported by bluetoothctl and hciconfig
  - kernel hci/btusb/bluetooth/rfkill logs
  - udev rules and runtime udev events
  - optional HCI traffic through btmon when available
  - Xbox ERTM setting
  - joystick devices under /dev/input/js*

The default mode only reads state. --apply-xbox-fixes uses sudo to disable ERTM
now, persist it in /etc/modprobe.d/xbox-bluetooth.conf, unblock Bluetooth, and
restart bluetooth.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --apply-xbox-fixes)
            APPLY_XBOX_FIXES=1
            ;;
        --reset-device)
            shift
            RESET_MAC=${1:-}
            if [ -z "$RESET_MAC" ]; then
                echo "ERROR: --reset-device requires a MAC address."
                exit 2
            fi
            ;;
        --scan)
            SCAN=1
            ;;
        --monitor)
            shift
            MONITOR_SECONDS=${1:-30}
            case "$MONITOR_SECONDS" in
                ''|*[!0-9]*)
                    echo "ERROR: --monitor requires seconds, for example: --monitor 30"
                    exit 2
                    ;;
            esac
            ;;
        --capture)
            shift
            CAPTURE_SECONDS=${1:-60}
            case "$CAPTURE_SECONDS" in
                ''|*[!0-9]*)
                    echo "ERROR: --capture requires seconds, for example: --capture 60"
                    exit 2
                    ;;
            esac
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1"
            usage
            exit 2
            ;;
    esac
    shift
done

if [ "$CAPTURE_SECONDS" -gt 0 ]; then
    STAMP=$(date +%Y%m%d_%H%M%S 2>/dev/null || date +%s)
    CAPTURE_DIR=${CAPTURE_DIR:-xbox_bt_capture_$STAMP}
    mkdir -p "$CAPTURE_DIR"
    LOG="$CAPTURE_DIR/summary.log"
    PIDS_FILE="$CAPTURE_DIR/monitor_pids.txt"
    : > "$PIDS_FILE"
fi
: > "$LOG"

note() {
    printf '\n### %s\n' "$*" | tee -a "$LOG"
}

run() {
    note "$*"
    "$@" 2>&1 | tee -a "$LOG"
    status=$?
    echo "[exit=$status]" | tee -a "$LOG"
    return 0
}

run_sh() {
    note "$*"
    sh -c "$*" 2>&1 | tee -a "$LOG"
    status=$?
    echo "[exit=$status]" | tee -a "$LOG"
    return 0
}

have() {
    command -v "$1" >/dev/null 2>&1
}

limited_cmd() {
    if have timeout; then
        timeout "$CMD_TIMEOUT" "$@"
    else
        "$@"
    fi
}

limited_sh() {
    cmd="$1"
    if have timeout; then
        timeout "$CMD_TIMEOUT" sh -c "$cmd"
    else
        sh -c "$cmd"
    fi
}

save_cmd() {
    out_file="$1"
    shift
    {
        echo "### $*"
        echo "time: $(date -Iseconds 2>/dev/null || date)"
        limited_cmd "$@"
        status=$?
        echo "[exit=$status]"
    } > "$out_file" 2>&1
    return 0
}

save_sh() {
    out_file="$1"
    shift
    {
        echo "### $*"
        echo "time: $(date -Iseconds 2>/dev/null || date)"
        limited_sh "$*"
        status=$?
        echo "[exit=$status]"
    } > "$out_file" 2>&1
    return 0
}

start_bg_sh() {
    out_file="$1"
    shift
    cmd="$*"
    note "starting background monitor: $cmd -> $out_file"
    (
        echo "### started: $cmd"
        echo "time: $(date -Iseconds 2>/dev/null || date)"
        if have timeout; then
            timeout $((CAPTURE_SECONDS + 10)) sh -c "$cmd"
        else
            sh -c "$cmd"
        fi
        status=$?
        echo "### exited status=$status time=$(date -Iseconds 2>/dev/null || date)"
    ) >> "$CAPTURE_DIR/$out_file" 2>&1 &
    echo "$!" >> "$PIDS_FILE"
}

stop_bg() {
    if [ -n "$PIDS_FILE" ] && [ -f "$PIDS_FILE" ]; then
        while IFS= read -r pid; do
            [ -n "$pid" ] || continue
            kill "$pid" >/dev/null 2>&1 || true
        done < "$PIDS_FILE"
        sleep 1
        while IFS= read -r pid; do
            [ -n "$pid" ] || continue
            kill -9 "$pid" >/dev/null 2>&1 || true
        done < "$PIDS_FILE"
    fi
}

trap 'stop_bg' INT TERM EXIT

write_sysfs_state() {
    echo "time: $(date -Iseconds 2>/dev/null || date)"
    echo ""
    echo "[/sys/class/rfkill]"
    for path in /sys/class/rfkill/rfkill*; do
        [ -e "$path" ] || continue
        echo "--- $path"
        for item in name type state soft hard persistent uevent; do
            if [ -r "$path/$item" ]; then
                printf '%s=' "$item"
                cat "$path/$item" 2>/dev/null
            fi
        done
    done
    echo ""
    echo "[/sys/class/bluetooth]"
    for path in /sys/class/bluetooth/hci*; do
        [ -e "$path" ] || continue
        echo "--- $path"
        for item in address name type uevent power/control power/runtime_status power/runtime_suspended_time power/runtime_active_time; do
            if [ -r "$path/$item" ]; then
                printf '%s=' "$item"
                cat "$path/$item" 2>/dev/null
            fi
        done
    done
    echo ""
    echo "[bluetooth module parameters]"
    if [ -r /sys/module/bluetooth/parameters/disable_ertm ]; then
        printf 'disable_ertm='
        cat /sys/module/bluetooth/parameters/disable_ertm 2>/dev/null
    else
        echo "disable_ertm=not-readable"
    fi
}

write_env_basics() {
    {
        echo "time: $(date -Iseconds 2>/dev/null || date)"
        echo "user: $(id 2>/dev/null || true)"
        echo "pwd: $(pwd)"
        echo "kernel: $(uname -a 2>/dev/null || true)"
        echo "cmdline:"
        cat /proc/cmdline 2>/dev/null || true
        if [ -r /etc/os-release ]; then
            echo "os-release:"
            sed 's/^/  /' /etc/os-release
        fi
    }
}

write_static_snapshot() {
    prefix="$1"
    [ -n "$CAPTURE_DIR" ] || return 0

    write_env_basics > "$CAPTURE_DIR/${prefix}_env.txt" 2>&1
    write_sysfs_state > "$CAPTURE_DIR/${prefix}_sysfs.txt" 2>&1

    if have rfkill; then
        save_cmd "$CAPTURE_DIR/${prefix}_rfkill.txt" rfkill list
        if rfkill --help 2>&1 | grep -q event; then
            echo "rfkill event command is available." >> "$CAPTURE_DIR/${prefix}_rfkill.txt"
        fi
    fi

    if have bluetoothctl; then
        save_cmd "$CAPTURE_DIR/${prefix}_bluetoothctl_version.txt" bluetoothctl --version
        save_cmd "$CAPTURE_DIR/${prefix}_bluetoothctl_list.txt" bluetoothctl list
        save_cmd "$CAPTURE_DIR/${prefix}_bluetoothctl_show.txt" bluetoothctl show
        save_cmd "$CAPTURE_DIR/${prefix}_bluetoothctl_devices.txt" bluetoothctl devices
        save_cmd "$CAPTURE_DIR/${prefix}_bluetoothctl_paired.txt" bluetoothctl paired-devices
        save_sh "$CAPTURE_DIR/${prefix}_bluetoothctl_info.txt" "bluetoothctl devices | awk '{print \$2}' | while read mac; do [ -n \"\$mac\" ] || continue; echo \"--- device \$mac\"; bluetoothctl info \"\$mac\"; done"
    fi

    if have systemctl; then
        save_sh "$CAPTURE_DIR/${prefix}_systemd_units.txt" "systemctl list-units --all '*bluetooth*' '*rfkill*' '*NetworkManager*' --no-pager"
        save_sh "$CAPTURE_DIR/${prefix}_systemd_unit_files.txt" "systemctl list-unit-files '*bluetooth*' '*rfkill*' '*NetworkManager*' --no-pager"
        save_cmd "$CAPTURE_DIR/${prefix}_bluetooth_status.txt" systemctl status bluetooth --no-pager
        save_cmd "$CAPTURE_DIR/${prefix}_systemd_rfkill_service_status.txt" systemctl status systemd-rfkill.service --no-pager
        save_cmd "$CAPTURE_DIR/${prefix}_systemd_rfkill_socket_status.txt" systemctl status systemd-rfkill.socket --no-pager
        save_sh "$CAPTURE_DIR/${prefix}_systemd_cat.txt" "systemctl cat bluetooth systemd-rfkill.service systemd-rfkill.socket NetworkManager 2>&1"
        save_sh "$CAPTURE_DIR/${prefix}_systemd_show.txt" "systemctl show bluetooth systemd-rfkill.service systemd-rfkill.socket -p Id -p Names -p ActiveState -p SubState -p FragmentPath -p DropInPaths -p ExecStart -p WantedBy -p Before -p After -p Requires -p Wants 2>&1"
    fi

    if have journalctl; then
        save_sh "$CAPTURE_DIR/${prefix}_journal_recent.txt" "journalctl -b --no-pager -u bluetooth -u systemd-rfkill.service -u systemd-rfkill.socket -u NetworkManager | tail -300"
    fi

    if have dmesg; then
        save_sh "$CAPTURE_DIR/${prefix}_dmesg_recent.txt" "dmesg -T 2>/dev/null | grep -iE 'bluetooth|btusb|hci|xpad|xbox|firmware|rfkill|usb' | tail -300"
    fi

    if have hciconfig; then
        save_cmd "$CAPTURE_DIR/${prefix}_hciconfig.txt" hciconfig -a
    fi
    if have lsusb; then
        save_cmd "$CAPTURE_DIR/${prefix}_lsusb.txt" lsusb
    fi
    if have lsmod; then
        save_sh "$CAPTURE_DIR/${prefix}_lsmod_bt.txt" "lsmod | grep -iE 'bluetooth|btusb|btrtl|btintel|btbcm|xpad|hid|uhid|joydev' || true"
    fi
    if have modinfo; then
        save_sh "$CAPTURE_DIR/${prefix}_modinfo.txt" "modinfo bluetooth 2>&1; modinfo btusb 2>&1; modinfo xpad 2>&1"
    fi

    save_sh "$CAPTURE_DIR/${prefix}_input_devices.txt" "ls -l /dev/input/js* /dev/input/event* 2>&1; cat /proc/bus/input/devices 2>/dev/null"
    save_sh "$CAPTURE_DIR/${prefix}_processes.txt" "ps -eo pid,ppid,user,stat,comm,args | grep -iE 'bluetooth|bluez|rfkill|blueman|gnome|network|dbus|udev|xpad|hid' | grep -v grep || true"

    save_sh "$CAPTURE_DIR/${prefix}_systemd_rfkill_saved_state.txt" "ls -l /var/lib/systemd/rfkill 2>&1; for f in /var/lib/systemd/rfkill/*; do [ -e \"\$f\" ] || continue; echo \"--- \$f\"; cat \"\$f\" 2>&1; done"
    save_sh "$CAPTURE_DIR/${prefix}_bluetooth_storage_listing.txt" "find /var/lib/bluetooth -maxdepth 3 -printf '%M %u %g %p\n' 2>&1"
    save_sh "$CAPTURE_DIR/${prefix}_bluetooth_config.txt" "echo '--- /etc/bluetooth/main.conf'; sed -n '1,240p' /etc/bluetooth/main.conf 2>&1; echo '--- /etc/modprobe.d bluetooth files'; ls -l /etc/modprobe.d 2>&1; grep -RInE 'bluetooth|btusb|disable_ertm|xpad|xbox' /etc/modprobe.d /lib/modprobe.d 2>&1 || true"
    save_sh "$CAPTURE_DIR/${prefix}_udev_rules_matches.txt" "grep -RInE 'bluetooth|rfkill|hci|btusb|xpad|xbox|joydev|hid' /etc/udev/rules.d /lib/udev/rules.d 2>&1 || true"
}

poll_state() {
    seconds="$1"
    out_file="$CAPTURE_DIR/state_poll.log"
    count=0
    while [ "$count" -lt "$seconds" ]; do
        {
            echo ""
            echo "--- poll=$count time=$(date -Iseconds 2>/dev/null || date)"
            echo "[rfkill list bluetooth]"
            if have rfkill; then
                rfkill list bluetooth 2>&1
            else
                echo "rfkill missing"
            fi
            echo "[sysfs]"
            write_sysfs_state
            echo "[bluetoothctl show]"
            if have bluetoothctl; then
                limited_cmd bluetoothctl show 2>&1
                echo "[bluetoothctl devices]"
                limited_cmd bluetoothctl devices 2>&1
                echo "[bluetoothctl paired-devices]"
                limited_cmd bluetoothctl paired-devices 2>&1
            else
                echo "bluetoothctl missing"
            fi
            echo "[joystick devices]"
            ls -l /dev/input/js* 2>&1 || true
            echo "[hciconfig]"
            if have hciconfig; then
                limited_cmd hciconfig -a 2>&1
            else
                echo "hciconfig missing"
            fi
        } >> "$out_file" 2>&1
        sleep 1
        count=$((count + 1))
    done
}

run_capture() {
    note "capture started for $CAPTURE_SECONDS seconds"
    note "while this is running, reproduce the problem: toggle Bluetooth once, put the Xbox controller in pairing mode, and try to connect"
    if [ "$(id -u 2>/dev/null || echo 1)" != "0" ]; then
        note "not running as root"
        echo "Some logs such as btmon, dmesg, and /var/lib/bluetooth may be incomplete. Prefer: sudo ./diagnose_xbox_bluetooth.sh --capture $CAPTURE_SECONDS" | tee -a "$LOG"
    fi

    write_static_snapshot "before"

    if have journalctl; then
        start_bg_sh "journal_follow.log" "journalctl -f -b --no-pager -u bluetooth -u systemd-rfkill.service -u systemd-rfkill.socket -u NetworkManager"
    fi
    if have dmesg; then
        start_bg_sh "dmesg_follow.log" "dmesg -wT"
    fi
    if have udevadm; then
        start_bg_sh "udev_monitor.log" "udevadm monitor --kernel --udev --property --subsystem-match=bluetooth --subsystem-match=rfkill --subsystem-match=usb --subsystem-match=input"
    fi
    if have rfkill && rfkill --help 2>&1 | grep -q event; then
        start_bg_sh "rfkill_events.log" "rfkill event"
    fi
    if have btmon; then
        start_bg_sh "btmon.log" "btmon -t"
    fi
    if have dbus-monitor; then
        start_bg_sh "dbus_bluez.log" "dbus-monitor --system \"type='signal',sender='org.bluez'\" \"type='method_call',destination='org.bluez'\""
    fi

    poll_state "$CAPTURE_SECONDS"
    stop_bg
    trap - INT TERM EXIT

    write_static_snapshot "after"

    note "capture finished"
    if have tar; then
        archive="$CAPTURE_DIR.tar.gz"
        tar -czf "$archive" "$CAPTURE_DIR" 2>&1 | tee -a "$LOG"
        echo "Archive: $archive" | tee -a "$LOG"
    else
        echo "tar not found. Log directory: $CAPTURE_DIR" | tee -a "$LOG"
    fi
}

note "diagnostic started"
write_env_basics | tee -a "$LOG"

if [ "$APPLY_XBOX_FIXES" = "1" ]; then
    note "applying Xbox Bluetooth fixes"
    if have sudo; then
        run_sh "echo Y | sudo tee /sys/module/bluetooth/parameters/disable_ertm"
        run_sh "printf '%s\n' 'options bluetooth disable_ertm=1' | sudo tee /etc/modprobe.d/xbox-bluetooth.conf"
        run_sh "printf '%s\n' 'bluetooth' 'uhid' 'joydev' | sudo tee /etc/modules-load.d/xbox-bluetooth.conf"
        run sudo modprobe bluetooth
        run sudo modprobe uhid
        run sudo modprobe joydev
        run_sh "printf '%s\n' '[Unit]' 'Description=Disable Bluetooth ERTM and load HID modules for Xbox controllers' 'DefaultDependencies=no' 'After=systemd-modules-load.service' 'Before=bluetooth.service' '' '[Service]' 'Type=oneshot' \"ExecStart=/bin/sh -c 'modprobe bluetooth 2>/dev/null || true; modprobe uhid 2>/dev/null || true; modprobe joydev 2>/dev/null || true; i=0; while [ \$i -lt 20 ]; do if [ -w /sys/module/bluetooth/parameters/disable_ertm ]; then echo Y > /sys/module/bluetooth/parameters/disable_ertm; cat /sys/module/bluetooth/parameters/disable_ertm; exit 0; fi; i=\$((i + 1)); sleep 0.25; done; echo disable_ertm_not_found; exit 0'\" 'RemainAfterExit=yes' '' '[Install]' 'WantedBy=multi-user.target' 'WantedBy=bluetooth.service' | sudo tee /etc/systemd/system/xbox-bluetooth-ertm.service"
        if have rfkill; then
            run sudo rfkill unblock bluetooth
        fi
        if have systemctl; then
            run sudo systemctl daemon-reload
            run sudo systemctl enable xbox-bluetooth-ertm.service
            run sudo systemctl enable bluetooth.service
            run sudo systemctl start xbox-bluetooth-ertm.service
            run sudo systemctl restart bluetooth
        fi
        run_sh "cat /sys/module/bluetooth/parameters/disable_ertm 2>/dev/null || true"
        run_sh "ls -l /dev/uhid 2>&1 || true"
    else
        echo "ERROR: sudo is required for --apply-xbox-fixes." | tee -a "$LOG"
    fi
fi

if [ "$CAPTURE_SECONDS" -gt 0 ]; then
    run_capture
    exit 0
fi

if have rfkill; then
    run rfkill list
else
    note "rfkill missing"
    echo "rfkill command not found." | tee -a "$LOG"
fi

if have systemctl; then
    run systemctl is-enabled bluetooth
    run systemctl is-active bluetooth
    run systemctl status bluetooth --no-pager
    run systemctl status systemd-rfkill.service --no-pager
    run systemctl status systemd-rfkill.socket --no-pager
    run_sh "systemctl list-units --all '*bluetooth*' '*rfkill*' '*NetworkManager*' --no-pager"
else
    note "systemctl missing"
    echo "systemctl command not found." | tee -a "$LOG"
fi

note "systemd rfkill saved state"
if [ -d /var/lib/systemd/rfkill ]; then
    ls -l /var/lib/systemd/rfkill 2>&1 | tee -a "$LOG"
    for state_file in /var/lib/systemd/rfkill/*; do
        [ -e "$state_file" ] || continue
        echo "--- $state_file" | tee -a "$LOG"
        cat "$state_file" 2>&1 | tee -a "$LOG"
    done
else
    echo "/var/lib/systemd/rfkill not found." | tee -a "$LOG"
fi

if have bluetoothctl; then
    run bluetoothctl --version
    run bluetoothctl list
    run bluetoothctl show
    run bluetoothctl devices
    run bluetoothctl paired-devices
else
    note "bluetoothctl missing"
    echo "Install BlueZ tools first. On Ubuntu/Debian: sudo apt-get install bluez" | tee -a "$LOG"
fi

if [ -n "$RESET_MAC" ] && have bluetoothctl; then
    note "resetting remembered Bluetooth device $RESET_MAC"
    bluetoothctl disconnect "$RESET_MAC" 2>&1 | tee -a "$LOG" || true
    bluetoothctl remove "$RESET_MAC" 2>&1 | tee -a "$LOG" || true
fi

if [ "$SCAN" = "1" ] && have bluetoothctl; then
    note "scanning for $SCAN_SECONDS seconds"
    bluetoothctl power on 2>&1 | tee -a "$LOG" || true
    bluetoothctl agent on 2>&1 | tee -a "$LOG" || true
    bluetoothctl default-agent 2>&1 | tee -a "$LOG" || true
    bluetoothctl scan on >/dev/null 2>&1 || true
    sleep "$SCAN_SECONDS"
    bluetoothctl scan off >/dev/null 2>&1 || true
    run bluetoothctl devices
fi

if [ "$MONITOR_SECONDS" -gt 0 ]; then
    note "monitoring rfkill and adapter state for $MONITOR_SECONDS seconds"
    count=0
    while [ "$count" -lt "$MONITOR_SECONDS" ]; do
        echo "--- monitor second $count time $(date -Iseconds 2>/dev/null || date)" | tee -a "$LOG"
        if have rfkill; then
            rfkill list bluetooth 2>&1 | tee -a "$LOG"
        fi
        if have bluetoothctl; then
            bluetoothctl show 2>&1 | grep -E 'Controller|Name:|Alias:|Powered:|Discoverable:|Pairable:|Blocked:|Discovering:' | tee -a "$LOG"
        fi
        sleep 1
        count=$((count + 1))
    done
fi

note "Xbox ERTM setting"
if [ -r /sys/module/bluetooth/parameters/disable_ertm ]; then
    cat /sys/module/bluetooth/parameters/disable_ertm 2>&1 | tee -a "$LOG"
else
    echo "/sys/module/bluetooth/parameters/disable_ertm is not readable or not present." | tee -a "$LOG"
fi

note "Bluetooth adapters and USB devices"
if have hciconfig; then
    run hciconfig -a
else
    echo "hciconfig not found." | tee -a "$LOG"
fi
if have lsusb; then
    run lsusb
else
    echo "lsusb not found." | tee -a "$LOG"
fi

note "sysfs state"
write_sysfs_state | tee -a "$LOG"

note "joystick devices"
if ls -l /dev/input/js* >/tmp/arm_min_js_devices.$$ 2>&1; then
    cat /tmp/arm_min_js_devices.$$ | tee -a "$LOG"
else
    cat /tmp/arm_min_js_devices.$$ | tee -a "$LOG"
    echo "No /dev/input/js* devices found." | tee -a "$LOG"
fi
rm -f /tmp/arm_min_js_devices.$$

note "recent Bluetooth kernel logs"
if have dmesg; then
    dmesg -T 2>/dev/null | grep -iE 'bluetooth|btusb|hci|xpad|xbox|firmware|rfkill|usb' | tail -160 | tee -a "$LOG"
else
    echo "dmesg not found." | tee -a "$LOG"
fi

note "recent bluetooth service logs"
if have journalctl; then
    journalctl -u bluetooth -u systemd-rfkill.service -u systemd-rfkill.socket -b --no-pager 2>/dev/null | tail -180 | tee -a "$LOG"
else
    echo "journalctl not found." | tee -a "$LOG"
fi

note "summary hints"
{
    echo "If rfkill shows blocked: run sudo rfkill unblock bluetooth."
    echo "If disable_ertm is N/0/false: run ./diagnose_xbox_bluetooth.sh --apply-xbox-fixes, then reboot if needed."
    echo "If bluetooth service is inactive: run sudo systemctl enable --now bluetooth."
    echo "If hci logs show firmware or command timeout errors: try a powered USB hub or another Bluetooth adapter."
    echo "If pairing exists but connect fails: run this script with --reset-device MAC, then pair again."
    echo "For unstable toggling, run: sudo ./diagnose_xbox_bluetooth.sh --capture 60"
    echo "Diagnostic log saved to: $LOG"
} | tee -a "$LOG"
