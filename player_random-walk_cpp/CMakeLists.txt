cmake_minimum_required(VERSION 3.1)
project(player_random-walk)

find_package(Threads REQUIRED)

set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS random regex system thread) # VC require regex, don't know why

add_executable(player_random-walk player_random-walk.cpp ../common/ai_base.cpp)
include_directories(
  ../common
  ../extlibs/autobahn-cpp
  ../extlibs/msgpack-c/include
  ${Boost_INCLUDE_DIRS}
  )
target_link_libraries(player_random-walk
  Threads::Threads
  ${Boost_LIBRARIES}
  )
set_property(TARGET player_random-walk PROPERTY CXX_STANDARD 14)

if(WIN32)
  if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
    include_directories(../extlibs/openssl/include)
  else()
    include_directories(../extlibs/openssl/include64)
  endif()
  add_definitions(-D_WIN32_WINNT=0x0500 -D_WINSOCK_DEPRECATED_NO_WARNINGS)
endif()
