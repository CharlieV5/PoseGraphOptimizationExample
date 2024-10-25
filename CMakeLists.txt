cmake_minimum_required( VERSION 3.5 )
project( pose_graph )
set(CMAKE_CXX_STANDARD 17)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# g2o 
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

# gtsam 
find_package( GTSAMCMakeTools )
find_package( GTSAM REQUIRED )
include_directories( ${GTSAM_INCLUDE_DIR} )


#ceres
find_package(Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

add_executable( pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp )
target_link_libraries( pose_graph_g2o_SE3
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
)


add_executable( pose_graph_g2o_lie pose_graph_g2o_lie_algebra.cpp )
target_link_libraries( pose_graph_g2o_lie
    g2o_core g2o_stuff 
    ${CHOLMOD_LIBRARIES}
    ${Sophus_LIBRARIES}
)

add_executable( pose_graph_gtsam pose_graph_gtsam.cpp )
target_link_libraries( pose_graph_gtsam
    ${CHOLMOD_LIBRARIES} gtsam
)

add_executable( pose_graph_gtsam_isam pose_graph_gtsam_isam.cpp )
target_link_libraries( pose_graph_gtsam_isam
    ${CHOLMOD_LIBRARIES} gtsam
)

add_executable( pose_graph_ceres ceres_SE3.cpp )
target_link_libraries( pose_graph_ceres
    ${CHOLMOD_LIBRARIES} ${CERES_LIBRARIES} ${Sophus_LIBRARIES}
)

add_executable( pose_graph_ceres_new ceres_SE3_new.cpp )
target_link_libraries( pose_graph_ceres_new
    ${CHOLMOD_LIBRARIES} ${CERES_LIBRARIES} ${Sophus_LIBRARIES}
)
