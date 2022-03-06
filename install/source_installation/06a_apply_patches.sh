BASE_DIR=$(realpath "$(dirname $0)")

DEPENDENCIES_WS=$HOME/packages_ros_ws
DEPENDENCIES_WS_SRC=${DEPENDENCIES_WS}/src

find ${DEPENDENCIES_WS_SRC} -type f -name CMakeLists.txt -exec sed -i'' -e 's/Boost REQUIRED python37/Boost REQUIRED python3/g' {} +

# find ${DEPENDENCIES_WS_SRC} -type f -name CMakeLists.txt -exec sed -i'' -e 's/find_package(realsense2 2.45.0)/find_package(realsense2 2.41.0)/g' {} +

sed -i -e 's/${G2O_INCREMENTAL_LIB}/#${G2O_INCREMENTAL_LIB}/g'  ${DEPENDENCIES_WS_SRC}/teb_local_planner/cmake_modules/FindG2O.cmake

# create a patch file: https://stackoverflow.com/questions/6658313/how-can-i-generate-a-git-patch-for-a-specific-commit
# helpful forum post: https://stackoverflow.com/questions/4770177/git-patch-does-not-apply
cd ${DEPENDENCIES_WS_SRC}/image_pipeline/
git apply ${BASE_DIR}/fix-image-pipeline.patch --reject --whitespace=fix 

cd ${DEPENDENCIES_WS_SRC}/rtabmap_ros/
git apply ${BASE_DIR}/skip-rtabmaprviz.patch --reject --whitespace=fix 
