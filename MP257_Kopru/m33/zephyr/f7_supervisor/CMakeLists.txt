# F7 gözetim uygulaması (Zephyr, STM32MP257F-DK Cortex-M33).
# Temel: Zephyr samples/subsys/ipc/openamp_rsc_table; src/main_remote.c o örnekten kopyalanır (README adım Z4).
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(f7_supervisor)

set(COMMON ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/main_remote.c)
  message(FATAL_ERROR "src/main_remote.c yok: README adim Z4'e bak")
endif()

target_include_directories(app PRIVATE ${LIBMETAL_INCLUDE_DIR} ${OPENAMP_INCLUDE_DIR} ${PLATFORM_DIR} ${COMMON} src)

target_sources(app PRIVATE
  src/main_remote.c
  src/app.c
  ${COMMON}/link_proto.c
  ${COMMON}/supervisor.c
)
