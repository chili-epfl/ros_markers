cmake_minimum_required(VERSION 2.8.3)
project(chilitags_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
ExternalProject_Add(chilitags_src
  GIT_REPOSITORY https://github.com/chili-epfl/chilitags.git
  CONFIGURE_COMMAND cmake -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE=Release ../chilitags_src 
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND make install
)

cs_add_library(${PROJECT_NAME} src/lib_wrap.cc)
add_dependencies(${PROJECT_NAME} chilitags_src)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libchilitags${CMAKE_SHARED_LIBRARY_SUFFIX}
)

cs_install()
cs_export(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS chilitags-extras.cmake
)