#!/usr/bin/env bash
# Official ROS apt packages, extracted only inside this workspace; no sudo.
set -euo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$workspace/.vendor/downloads" "$workspace/.vendor/middleware"
cd "$workspace/.vendor/downloads"
apt download ros-lyrical-rmw-cyclonedds-cpp ros-lyrical-cyclonedds \
  ros-lyrical-iceoryx-binding-c ros-lyrical-iceoryx-hoofs ros-lyrical-iceoryx-posh
for archive in ./*.deb; do
  dpkg-deb -x "$archive" "$workspace/.vendor/middleware"
done
sha256sum ./*.deb > "$workspace/.vendor/middleware-packages.sha256"
echo 'Local middleware ready. Reload env.sh in each project terminal.'
