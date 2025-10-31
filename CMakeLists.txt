## Copyright 2025
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(agx2usd LANGUAGES CXX)

## Dependencies ##

# USD dependencies
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS python QUIET)
find_package(TBB QUIET)

# ANARI (for AGX reading)
find_package(anari REQUIRED)

# USD (for writing USD files)
# Try to find USD/pxr package
find_package(pxr QUIET)

if(NOT pxr_FOUND)
  # If not found, try to locate it manually via USD_ROOT or PXR_ROOT
  if(DEFINED ENV{USD_ROOT})
    set(pxr_DIR "$ENV{USD_ROOT}/cmake")
    find_package(pxr REQUIRED)
  elseif(DEFINED ENV{PXR_ROOT})
    set(pxr_DIR "$ENV{PXR_ROOT}/cmake")
    find_package(pxr REQUIRED)
  else()
    message(FATAL_ERROR 
      "Could not find OpenUSD. Please install it or set USD_ROOT environment variable.\n"
      "See README.md for installation instructions.")
  endif()
endif()

## AGX library ##
add_subdirectory(agx)

## Main converter executable ##

add_executable(agx2usd main.cpp)
target_link_libraries(agx2usd PRIVATE 
    agx
    ${PXR_LIBRARIES}
)

target_include_directories(agx2usd PRIVATE
    ${PXR_INCLUDE_DIRS}
)

# USD requires these compile definitions
target_compile_definitions(agx2usd PRIVATE
    ${PXR_DEFINITIONS}
)

