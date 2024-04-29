set -e

# TARGET_SOC="rk3588"
GCC_COMPILER=aarch64-linux-gnu

export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

ROOT_PWD=$( cd "$( dirname $0 )" && cd -P "$( dirname "$SOURCE" )" && pwd )

# build
BUILD_DIR=${ROOT_PWD}/build/build_linux_aarch64

if [ ! -d "${BUILD_DIR}" ]; then
  mkdir -p ${BUILD_DIR}
fi

cd ${BUILD_DIR}
cmake ../.. -DCMAKE_SYSTEM_NAME=Linux
make -j8
make install
cd -


#cd install && ./object_detection /home/rpdzkj/code2/object_detection_project/install/model/RK3588/yolox_RK3588_i8.rknn /home/rpdzkj/code/opencv_test/02_1080.mp4

# ./install/object_detection /home/rpdzkj/code/object_detection_project/install/model/RK3588/yolox_RK3588_i8.rknn /home/rpdzkj/code/opencv_test/02_1080.mp4