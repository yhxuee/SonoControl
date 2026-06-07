cmake_minimum_required(VERSION 3.20)
# Keep VERSION in sync with include/version.hpp (single source of truth for the
# app version stamped into the GUI About box and the Windows .exe Details tab).
project(SonoControlCpp VERSION 2.0.0 LANGUAGES CXX)

# Compile the Windows version resource (VS_VERSIONINFO) into the executables so
# the .exe Properties -> Details tab shows a real version. RC is a no-op on
# non-Windows toolchains, so guard the whole thing behind WIN32. version.rc does
# #include "version.hpp"; the targets it is attached to already expose include/
# on their INCLUDE_DIRECTORIES (via sonocontrol_core), and CMake forwards those
# to the RC compiler with the right per-toolchain flag syntax, so no manual
# -I/-I flag wrangling is needed here.
if (WIN32)
    enable_language(RC)
    set(SONOCONTROL_VERSION_RC "${CMAKE_CURRENT_SOURCE_DIR}/resources/version.rc")
else()
    set(SONOCONTROL_VERSION_RC "")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Windows/MSVC compatibility:
# Qt checks __cplusplus and requires MSVC to report it correctly. /utf-8
# also avoids C4819 when source files contain Unicode UI text.
if (MSVC)
    add_compile_options(/Zc:__cplusplus /utf-8)
endif()

option(SONOCONTROL_BUILD_GUI "Build the Qt Widgets GUI executable" ON)

set(SONOCONTROL_CORE_SOURCES
    src/protocol.cpp
    src/hyus_protocol.cpp
    src/hyus_net.cpp
    src/config_io.cpp
    src/pid.cpp
    src/temperature.cpp
    src/serial_port.cpp
    src/udp_socket.cpp
    src/logger.cpp
    src/experiment.cpp
)

add_library(sonocontrol_core STATIC ${SONOCONTROL_CORE_SOURCES})
target_include_directories(sonocontrol_core PUBLIC include)

# Debug builds can use simulated temperature/ultrasound transports.
# Non-Debug builds hard-disable simulation at compile time.
target_compile_definitions(sonocontrol_core PUBLIC
    "$<$<CONFIG:Debug>:SONOCONTROL_DEBUG_SIM=1>"
    "$<$<NOT:$<CONFIG:Debug>>:SONOCONTROL_DEBUG_SIM=0>"
)

if (WIN32)
    target_link_libraries(sonocontrol_core PUBLIC ws2_32)
endif()

if (MSVC)
    target_compile_options(sonocontrol_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(sonocontrol_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(sonocontrol src/main.cpp ${SONOCONTROL_VERSION_RC})
target_link_libraries(sonocontrol PRIVATE sonocontrol_core)

set_target_properties(sonocontrol PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/bin"
)

if (MSVC)
    target_compile_options(sonocontrol PRIVATE /W4 /permissive-)
else()
    target_compile_options(sonocontrol PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (SONOCONTROL_BUILD_GUI)
    find_package(Qt6 COMPONENTS Widgets QUIET)
    if (Qt6_FOUND)
        set(QT_WIDGETS_LIB Qt6::Widgets)
    else()
        find_package(Qt5 COMPONENTS Widgets QUIET)
        if (Qt5_FOUND)
            set(QT_WIDGETS_LIB Qt5::Widgets)
        endif()
    endif()

    # Qt Network is OPTIONAL — only required for the in-app monitoring web
    # server. Some MinGW Qt installs (including older snapshots of qtbase) do
    # not ship the Network module; the rest of the GUI must keep building.
    # When found we set SONOCONTROL_WEB_SERVER=1 and add web_server.cpp to the
    # sources; main_gui.cpp guards every reference to the WebServer class
    # behind that define so the symbol is genuinely absent on builds without
    # the module instead of being declared-but-not-linked.
    set(SONOCONTROL_HAS_NETWORK FALSE)
    set(QT_NETWORK_LIB "")
    if (Qt6_FOUND)
        find_package(Qt6 COMPONENTS Network QUIET)
        if (Qt6Network_FOUND)
            set(QT_NETWORK_LIB Qt6::Network)
            set(SONOCONTROL_HAS_NETWORK TRUE)
        endif()
    elseif (Qt5_FOUND)
        find_package(Qt5 COMPONENTS Network QUIET)
        if (Qt5Network_FOUND)
            set(QT_NETWORK_LIB Qt5::Network)
            set(SONOCONTROL_HAS_NETWORK TRUE)
        endif()
    endif()

    if (QT_WIDGETS_LIB)
        # Fail early if CLion/CMake mixes incompatible Windows ABIs, for example
        # MSVC cl.exe + Qt mingw_64, or MinGW g++ + Qt msvc2022_64.
        string(TOLOWER "${CMAKE_PREFIX_PATH};${Qt6_DIR};${Qt5_DIR}" _sonocontrol_qt_hint_lower)
        if (MSVC AND _sonocontrol_qt_hint_lower MATCHES "mingw")
            message(FATAL_ERROR "Incompatible Qt/compiler setup: MSVC compiler is selected, but Qt MinGW was found. In CLion Toolchains select Qt's MinGW g++.exe, then delete the old build directory or reset CMake cache.")
        endif()
        if (MINGW AND _sonocontrol_qt_hint_lower MATCHES "msvc")
            message(FATAL_ERROR "Incompatible Qt/compiler setup: MinGW compiler is selected, but Qt MSVC was found. Use Qt mingw_64 or switch CLion Toolchains to Visual Studio/MSVC.")
        endif()

        set(CMAKE_AUTOMOC ON)
        set(SONOCONTROL_GUI_SOURCES src/gui/main_gui.cpp)
        if (SONOCONTROL_HAS_NETWORK)
            list(APPEND SONOCONTROL_GUI_SOURCES src/gui/web_server.cpp)
        else()
            message(WARNING "Qt Network was not found. The GUI will build, but the in-app monitoring web server will be disabled at compile time. Install Qt's Network module to enable it.")
        endif()
        add_executable(sonocontrol_gui ${SONOCONTROL_GUI_SOURCES} ${SONOCONTROL_VERSION_RC})
        target_link_libraries(sonocontrol_gui PRIVATE sonocontrol_core ${QT_WIDGETS_LIB} ${QT_NETWORK_LIB})
        if (SONOCONTROL_HAS_NETWORK)
            target_compile_definitions(sonocontrol_gui PRIVATE SONOCONTROL_WEB_SERVER=1)
        endif()
        set_target_properties(sonocontrol_gui PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin"
            RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin"
            RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/bin"
            RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/bin"
            WIN32_EXECUTABLE TRUE
        )
        if (MSVC)
            target_compile_options(sonocontrol_gui PRIVATE /W4 /permissive-)
        else()
            target_compile_options(sonocontrol_gui PRIVATE -Wall -Wextra -Wpedantic)
        endif()
        install(TARGETS sonocontrol_gui RUNTIME DESTINATION bin)
    else()
        message(WARNING "Qt Widgets was not found. The CLI executable will be built, but sonocontrol_gui is skipped. Install Qt 6/5 and reconfigure CMake to build the GUI.")
    endif()
endif()

install(TARGETS sonocontrol RUNTIME DESTINATION bin)
