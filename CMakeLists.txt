cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(example examples/example.cpp)

# I don't really understand how to manage builds with cmake ... don't judge me.
rosbuild_add_executable(seds_node src/seds_node.cpp 

src/mldemos/datasetManager.cpp src/mldemos/SEDS.cpp src/mldemos/dynamicalSEDS.cpp src/mldemos/mymaths.cpp 

src/fgmm/em.cpp src/fgmm/gaussian.cpp src/fgmm/gmm.cpp src/fgmm/gmmregression.cpp src/fgmm/smat.cpp src/fgmm/update.cpp 

src/MathLib/Differentiator.cpp	 src/MathLib/MathLibCommon.cpp	src/MathLib/ReferenceFrame.cpp	src/MathLib/SpatialMatrix.cpp	 src/MathLib/Vector3.cpp
src/MathLib/GradientDescent.cpp  src/MathLib/MathLib.cpp	src/MathLib/Regression.cpp	src/MathLib/SpatialVector.cpp	 src/MathLib/Vector.cpp
src/MathLib/IKGroupSolver.cpp	 src/MathLib/Matrix3.cpp	src/MathLib/SpatialForce.cpp	src/MathLib/SpatialVelocity.cpp
src/MathLib/IKSubSolver.cpp	 src/MathLib/Matrix4.cpp	src/MathLib/SpatialFrame.cpp	src/MathLib/TMatrix.cpp
src/MathLib/Macros.cpp		 src/MathLib/Matrix.cpp		src/MathLib/SpatialInertia.cpp	src/MathLib/TVector.cpp)
