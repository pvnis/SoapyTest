########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.14)
project(srsran)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

# Configure supported build types and default build type
set(supported_build_types "Release" "RelWithDebInfo" "Debug")
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Build type not specified, using default: ${default_build_type}")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Select build type" FORCE)
else()
  # Check if build type is supported
  if(NOT "${CMAKE_BUILD_TYPE}" IN_LIST supported_build_types)
    message(SEND_ERROR "Unsupported build type '${CMAKE_BUILD_TYPE}'. Supported build types: ${supported_build_types}")
  else()
    message(STATUS "The build type is ${CMAKE_BUILD_TYPE}")
  endif()
endif()

# Add possible values for build type in cmake-gui (drop-down widget, for convenience only)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${supported_build_types})

########################################################################
# Compiler specific setup
########################################################################
macro(ADD_CXX_COMPILER_FLAG_IF_AVAILABLE flag have)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(${flag} ${have})
    if (${have})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
    endif (${have})
endmacro(ADD_CXX_COMPILER_FLAG_IF_AVAILABLE)

# Make sure no instance of abstract class is left without a destructor
ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-Wnon-virtual-dtor HAVE_NON_VIRTUAL_DTOR)

# Make sure all overridden methods are marked as override
ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-Wsuggest-override HAVE_SUGGEST_OVERRIDE)

# Avoid shadow variables which can be caused due to C code ported into C++
ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-Wshadow HAVE_SHADOW)

# Set compiler flags for different build types.
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb -O0 -DDEBUG_MODE -DBUILD_TYPE_DEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -ggdb -DBUILD_TYPE_RELWITHDEBINFO")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-trapping-math -fno-math-errno -DBUILD_TYPE_RELEASE")

if(ENABLE_GPROF)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
endif(ENABLE_GPROF)

if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  if (HAVE_SSE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -funroll-loops")
  endif (HAVE_SSE)
endif (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug")

if (ENABLE_ASAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    # Note: When using ASAN, we need to ensure the use of RPATH instead of RUNPATH via "-Wl,--disable-new-dtags"
    # While RPATH is default, some systems (e.g. Ubuntu 18.04 and 20.04) use RUNPATH by default, which is non-transitive.
    # Since ASAN intercepts dlopen(), by which it replaces the dynamic string token "$ORIGIN" to its own location,
    # the RF plugins won't be found when using ASAN + RUNPATH in the top-level executable.
    ADD_CXX_COMPILER_FLAG_IF_AVAILABLE("-Wl,--disable-new-dtags" HAVE_RPATH_FORCE)
endif (ENABLE_ASAN)

if (ENABLE_TSAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
    add_definitions(-DENABLE_TSAN)
endif (ENABLE_TSAN)

if (FORCE_DEBUG_INFO)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif (FORCE_DEBUG_INFO)

if (ENABLE_GCOV)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif (ENABLE_GCOV)

if(NOT ASSERT_LEVEL STREQUAL "MINIMAL")
    # asserts are enabled.
    add_definitions(-DASSERTS_ENABLED)
    if((ASSERT_LEVEL STREQUAL "PARANOID") OR
       ((ASSERT_LEVEL STREQUAL "AUTO") AND (${CMAKE_BUILD_TYPE} STREQUAL "Debug")))
        add_definitions(-DPARANOID_ASSERTS_ENABLED)
        message(STATUS "Assertion level set to PARANOID")
    else()
        message(STATUS "Assertion level set to NORMAL")
    endif()
endif()

if(NOT EXIT_TIMEOUT STREQUAL "AUTO")
    message(STATUS "Manually set exit timeout. timeout=${EXIT_TIMEOUT}s")
    add_compile_definitions(TERM_TIMEOUT_S=${EXIT_TIMEOUT})
endif()

########################################################################
# Compiler launcher setup
########################################################################

# Enable ccache if not already enabled
find_program(CCACHE_EXECUTABLE ccache)
mark_as_advanced(CCACHE_EXECUTABLE)
if (CCACHE_EXECUTABLE)
    foreach (LANG C CXX)
        if (NOT DEFINED CMAKE_${LANG}_COMPILER_LAUNCHER AND NOT CMAKE_${LANG}_COMPILER MATCHES ".*/ccache$")
            message(STATUS "Enabling ccache for ${LANG}")
            set(CMAKE_${LANG}_COMPILER_LAUNCHER ${CCACHE_EXECUTABLE} CACHE STRING "")
        endif ()
    endforeach ()
endif ()

include(CTest)
execute_process(COMMAND sed -i "s|MemoryCheckCommandOptions: |MemoryCheckCommandOptions: --verbose --trace-children=yes --time-stamp=yes --leak-check=full --show-leak-kinds=all --show-reachable=yes --exit-on-first-error=yes --error-exitcode=22 --suppressions=${CMAKE_SOURCE_DIR}/.memcheck-suppressions|" DartConfiguration.tcl
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})


########################################################################
# Find dependencies
########################################################################
# Soapy
find_package(SoapySDR)
if (SoapySDR_FOUND)
    include_directories(${SoapySDR_INCLUDE_DIRS})
    link_directories(${SoapySDR_LIBRARY_DIRS})
endif (SoapySDR_FOUND)


########################################################################
# Build utility executable
########################################################################
add_executable(SoapySDRUtil
    SoapySDRUtil.cpp
    SoapySDRProbe.cpp
    SoapyRateTest.cpp
)

target_link_libraries(SoapySDRUtil ${SoapySDR_LIBRARIES})

target_include_directories(SoapySDRUtil PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/external)

set_target_properties(SoapySDR PROPERTIES INTERFACE_COMPILE_OPTIONS "")


install(TARGETS SoapySDRUtil DESTINATION ${CMAKE_INSTALL_BINDIR})

#install man pages for the application executable
install(FILES SoapySDRUtil.1 DESTINATION ${CMAKE_INSTALL_MANDIR}/man1)
