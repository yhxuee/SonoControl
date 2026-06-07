#ifndef SONOCONTROL_VERSION_HPP
#define SONOCONTROL_VERSION_HPP

// Single source of truth for the application version. Consumed by:
//   * src/gui/main_gui.cpp  (QApplication::setApplicationVersion / About box)
//   * resources/version.rc  (Windows VS_VERSIONINFO -> .exe Details tab)
//   * CMakeLists.txt keeps project(VERSION ...) in sync by hand.
// This header is included by both C++ and the Windows RC compiler, so it must
// contain nothing but preprocessor macros (no C++ syntax) and use a classic
// include guard rather than #pragma once for maximum windres/rc compatibility.

#define SONOCONTROL_VERSION_MAJOR 2
#define SONOCONTROL_VERSION_MINOR 0
#define SONOCONTROL_VERSION_PATCH 0
#define SONOCONTROL_VERSION_STR   "2.0.0"

#endif // SONOCONTROL_VERSION_HPP
