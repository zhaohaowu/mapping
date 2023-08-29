
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

if (BUILD_COMMPILER_TYPE MATCHES GCC)
    set(MDC_SDK /usr/local/mdc_sdk/dp_gea/mdc_cross_compiler)
    set(CMAKE_C_COMPILER ${MDC_SDK}/bin/aarch64-target-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER ${MDC_SDK}/bin/aarch64-target-linux-gnu-g++)
elseif (BUILD_COMMPILER_TYPE MATCHES LLVM)
    set(MDC_SDK /usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler)
    set(CMAKE_C_COMPILER ${MDC_SDK}/bin/clang)
    set(CMAKE_CXX_COMPILER ${MDC_SDK}/bin/clang++)
else()
    set(MDC_SDK /usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler)
    set(CMAKE_C_COMPILER ${MDC_SDK}/bin/clang)
    set(CMAKE_CXX_COMPILER ${MDC_SDK}/bin/clang++)
endif()

set(CMAKE_FIND_ROOT_PATH ${MDC_SDK}/sysroot/ ${CMAKE_FIND_ROOT_PATH})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
