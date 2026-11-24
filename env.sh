# Load with: source ./env.sh  (Bash)
_CUBE_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
source "${_CUBE_WORKSPACE}/install/setup.bash"
_CUBE_MIDDLEWARE_PREFIX="${_CUBE_WORKSPACE}/.vendor/middleware/opt/ros/lyrical"
if [[ -f "${_CUBE_MIDDLEWARE_PREFIX}/lib/librmw_cyclonedds_cpp.so" ]]; then
  export AMENT_PREFIX_PATH="${_CUBE_MIDDLEWARE_PREFIX}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  _CUBE_MIDDLEWARE_TRIPLET="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
  export LD_LIBRARY_PATH="${_CUBE_MIDDLEWARE_PREFIX}/lib:${_CUBE_MIDDLEWARE_PREFIX}/lib/${_CUBE_MIDDLEWARE_TRIPLET}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
export ROS_DOMAIN_ID=47
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_CUBE_WORKSPACE}/src/cube_push_sim/config/cyclonedds.xml"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export LP_NUM_THREADS=1
unset _CUBE_MIDDLEWARE_PREFIX _CUBE_MIDDLEWARE_TRIPLET _CUBE_WORKSPACE
