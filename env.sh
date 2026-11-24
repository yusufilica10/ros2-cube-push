# Load with: source ./env.sh  (Bash)
_CUBE_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
source "${_CUBE_WORKSPACE}/install/setup.bash"
unset _CUBE_WORKSPACE
export ROS_DOMAIN_ID=47
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Keep sensor publication from blocking navigation callbacks.
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
