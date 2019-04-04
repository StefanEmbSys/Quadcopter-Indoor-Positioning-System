cmake_minimum_required(VERSION 2.6)

set(ETHERNET_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(ETHERNET_INCLUDE_DIR ${SERIAL_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET ethernet)
	message("ethernet is already defined")
else()
	include_directories(${ETHERNET_INCLUDE_DIR})
	set(LIB_SRCS ethernet.cpp ethernet.h)
	add_library(ethernet STATIC ${LIB_SRCS})
endif()