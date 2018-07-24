cmake_minimum_required (VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
project (opensplice_minimal)

include (MacroOpenSplice)
include (osmacros)

include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(--std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

find_package (OpenSplice REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${OpenSplice_INCLUDE_DIRS}
  lib/boilerplate/template
  include
  gen
)

ADD_LIBRARY(GEN_SRC 
  gen/Boilerplate.cpp
  gen/BoilerplateDcps.cpp
  gen/BoilerplateDcps_impl.cpp
  gen/BoilerplateSplDcps.cpp
)

TARGET_LINK_LIBRARIES (GEN_SRC 
 ${OpenSplice_LIBRARIES}
)

include(lib/boilerplate/types/Template/CMakeLists.txt)
include(lib/boilerplate/types/Sensor/CMakeLists.txt)

ADD_LIBRARY (MGR_SRC
    lib/boilerplate/template/CheckStatus.cpp
    lib/boilerplate/template/Participant.cpp
    ${Sensor}
    ${Template}
)

TARGET_LINK_LIBRARIES (MGR_SRC
 ${OpenSplice_LIBRARIES}
 GEN_SRC
)

ADD_EXECUTABLE (pubsub_1 
    src/pubsub_1.cpp
)

TARGET_LINK_LIBRARIES (pubsub_1
    GEN_SRC
    MGR_SRC
    ${OpenSplice_LIBRARIES}
)

ADD_EXECUTABLE (pubsub_2 
    src/pubsub_2.cpp
)

TARGET_LINK_LIBRARIES (pubsub_2
    GEN_SRC
    MGR_SRC
    ${OpenSplice_LIBRARIES}
)
