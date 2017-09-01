project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	include_directories(/usr/local/opt/openssl/include)
	link_directories(/usr/local/opt/openssl/lib)
	link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (WIN32)
    set(VCPKG_INSTALL ${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET})
    message("Using vcpkg_install ${VCPKG_INSTALL}")
    include_directories(${VCPKG_INSTALL}/include)
    link_directories(${VCPKG_INSTALL}/lib)
else()
	include_directories(/usr/local/include)
	link_directories(/usr/local/lib)
endif()

add_executable(mpc ${sources})

if (NOT WIN32)
    target_link_libraries(mpc ipopt z ssl uv uWS)
else()
    target_link_libraries(mpc ipopt-vc10 ssleay32 libuv uWS ws2_32 zlib)
    #target_link_libraries(mpc optimized zlib)
	#target_link_libraries(mpc debug zlibd)
endif()
