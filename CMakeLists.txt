cmake_minimum_required (VERSION 2.8)
project (VescUartControl)

enable_language(C)
enable_language(CXX)

add_subdirectory(libserial)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/libserial
  )

link_directories (
  ${CMAKE_BINARY_DIR}/libserial
  )

set(VescUartSample_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/buffer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/VescUart.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/crc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/VescUartSample.cpp
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_executable(VescUartSample ${VescUartSample_SRCS})

set(VescUartSample_LINK_LIBS libserial_static pthread rt)
target_link_libraries(VescUartSample ${VescUartSample_LINK_LIBS})

INSTALL(TARGETS VescUartSample
  RUNTIME DESTINATION bin
  )
