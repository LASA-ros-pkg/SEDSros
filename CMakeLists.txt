cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(example examples/example.cpp)

rosbuild_add_library(fgmm src/fgmm/em.cpp src/fgmm/gaussian.cpp src/fgmm/gmm.cpp src/fgmm/gmmregression.cpp src/fgmm/smat.cpp src/fgmm/update.cpp)

rosbuild_add_library(mathlib 
src/MathLib/Differentiator.cpp	 src/MathLib/MathLibCommon.cpp	src/MathLib/ReferenceFrame.cpp	src/MathLib/SpatialMatrix.cpp	 src/MathLib/Vector3.cpp
src/MathLib/GradientDescent.cpp  src/MathLib/MathLib.cpp	src/MathLib/Regression.cpp	src/MathLib/SpatialVector.cpp	 src/MathLib/Vector.cpp
src/MathLib/IKGroupSolver.cpp	 src/MathLib/Matrix3.cpp	src/MathLib/SpatialForce.cpp	src/MathLib/SpatialVelocity.cpp
src/MathLib/IKSubSolver.cpp	 src/MathLib/Matrix4.cpp	src/MathLib/SpatialFrame.cpp	src/MathLib/TMatrix.cpp
src/MathLib/Macros.cpp		 src/MathLib/Matrix.cpp		src/MathLib/SpatialInertia.cpp	src/MathLib/TVector.cpp)

# I don't really understand how to manage builds with cmake ... don't judge me.
rosbuild_add_executable(teleop_pr2_arms_keyboard src/teleop_pr2_arms_keyboard.cpp)

rosbuild_add_executable(ds_node src/ds_node.cpp src/SEDS.cpp)
target_link_libraries(ds_node fgmm)
target_link_libraries(ds_node mathlib)

rosbuild_add_executable(seds_node src/seds_node.cpp src/seds_wrapper.cpp src/SEDS.cpp)
target_link_libraries(seds_node fgmm)
target_link_libraries(seds_node mathlib)

