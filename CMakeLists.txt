cmake_minimum_required(VERSION 3.16)
project(SoapyESPSDR VERSION 0.1.0 LANGUAGES CXX)

find_package(SoapySDR 0.7.0 CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

add_library(SoapyESPSDR SHARED SoapyESPSDR.cpp)
target_compile_features(SoapyESPSDR PRIVATE cxx_std_17)
target_compile_options(SoapyESPSDR PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(SoapyESPSDR PRIVATE SoapySDR Threads::Threads ZLIB::ZLIB)
set_target_properties(SoapyESPSDR PROPERTIES
    PREFIX "libsoapy_"
    OUTPUT_NAME "espsdr")

include(GNUInstallDirs)
install(TARGETS SoapyESPSDR LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI_VERSION})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR}/SoapyESPSDR)
