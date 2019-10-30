cmake_minimum_required(VERSION 3.1)
project(graph_ads)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

# File Gathering
file(GLOB_RECURSE HDRS
	${SRC_DIR}/*.h
	${SRC_DIR}/*.hpp
	${VENDOR_DIR}/*.h
	${VENDOR_DIR}/*.hpp
)
file(GLOB_RECURSE SRCS
	${SRC_DIR}/*.cpp
	${SRC_DIR}/*.c
	${VENDOR_DIR}/*.cpp
	${VENDOR_DIR}/*.c
)

add_executable(${PROJECT_NAME} ${HDRS} ${SRCS})

# Use Modern OpenGL 3.0
# set(OpenGL_GL_PREFERENCE "GLVND")
# include(FindPkgConfig)


# set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
# set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
# set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# add_subdirectory(${LIB_DIR}/glfw)


# find_package(OpenGL REQUIRED)
# if(NOT OPENGL_FOUND)
# 	message(FATAL_ERROR "OpenGL not found!")
# endif(NOT OPENGL_FOUND)
# 
# find_package(GLEW REQUIRED)
# if(NOT GLEW_FOUND)
# 	message(FATAL_ERROR "GLEW not found!")
# endif(NOT GLEW_FOUND)


# Define the include DIRs
include_directories(
	${graph_ads_SOURCE_DIR}/src
	# ${OPENGL_INCLUDE_DIRS}
	# ${GLEW_INCLUDE_DIRS}
	# ${GLFW_INCLUDE_DIRS}
)

target_include_directories(graph_ads PRIVATE ${VENDOR_DIR})

# Define the link libraries
# target_link_libraries(graph_ads
	# glfw
	# ${OPENGL_LIBRARIES}
	# ${GLEW_LIBRARIES}
# )

# So we can use data/ folder from build dir.
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${graph_ads_SOURCE_DIR}/data ${graph_ads_BINARY_DIR}/data RESULT_VARIABLE exitcode)
