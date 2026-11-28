#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 || ! -x "$1/bin/px4" || ! -f "$1/etc/init.d-posix/rcS" ]]; then
    echo "Usage: bash $0 /path/to/PX4-Autopilot/build/px4_sitl_default" >&2
    exit 2
fi

build="$(cd "$1" && pwd)"
firmwares="$(cd "$(dirname "$0")" && pwd)"
session="$(mktemp -d -t pterosim-h480.XXXXXXXX)"
trap 'rm -rf -- "$session/romfs"; echo "PX4 logs and parameters of this run: $session/run" >&2' EXIT
mkdir "$session/romfs" "$session/run"
cp -aL "$build/etc" "$session/romfs/"

airframes="$session/romfs/etc/init.d-posix/airframes"
autostart=22100
while compgen -G "$airframes/${autostart}_*" >/dev/null; do
    ((autostart += 1))
done
cp "$firmwares/px4_typhoon_h480" "$airframes/${autostart}_none_typhoon_h480"

export PX4_SYS_AUTOSTART="$autostart"
"$build/bin/px4" -w "$session/run" "$session/romfs/etc"
