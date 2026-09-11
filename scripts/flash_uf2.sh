#!/usr/bin/env bash
# Copy a .uf2 onto a mounted RP2040 BOOTSEL mass-storage drive.
#
# Run automatically as a POST_BUILD step on the firmware target so that VS
# Code's CMake Tools "Build" button both builds and flashes. If no bootloader
# drive is mounted (board not in BOOTSEL mode) this is a no-op and never fails
# the build.
set -u

uf2=${1:-}
if [ -z "$uf2" ] || [ ! -f "$uf2" ]; then
    echo "flash: no .uf2 to copy ('${uf2:-<none>}')" >&2
    exit 0
fi

user=${USER:-$(id -un 2>/dev/null || echo "")}

find_mount() {
    local dir
    # Preferred: GNOME/KDE auto-mount locations.
    for dir in \
        "/media/${user}/RPI-RP2" \
        "/run/media/${user}/RPI-RP2" \
        "/mnt/RPI-RP2"; do
        [ -n "$user" ] && [ -d "$dir" ] && { printf '%s\n' "$dir"; return 0; }
    done

    # Fallback: ask the kernel for a volume labelled RPI-RP2.
    if command -v lsblk >/dev/null 2>&1; then
        lsblk -rno MOUNTPOINT,LABEL 2>/dev/null \
            | awk '$2 == "RPI-RP2" && $1 != "" { print $1; exit }'
    fi
}

dest=${FLASH_MOUNT:-$(find_mount)}
if [ -z "${dest:-}" ]; then
    echo "flash: no RPI-RP2 drive mounted; skipping (put the board in BOOTSEL mode)"
    exit 0
fi

if cp -f "$uf2" "$dest/"; then
    sync || true
    echo "flash: $(basename "$uf2") -> $dest"
else
    echo "flash: copy to $dest failed (ignored)" >&2
fi
exit 0
