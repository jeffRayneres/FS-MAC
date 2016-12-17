INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_FSMAC fsmac)

FIND_PATH(
    FSMAC_INCLUDE_DIRS
    NAMES fsmac/api.h
    HINTS $ENV{FSMAC_DIR}/include
        ${PC_FSMAC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    FSMAC_LIBRARIES
    NAMES gnuradio-fsmac
    HINTS $ENV{FSMAC_DIR}/lib
        ${PC_FSMAC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(FSMAC DEFAULT_MSG FSMAC_LIBRARIES FSMAC_INCLUDE_DIRS)
MARK_AS_ADVANCED(FSMAC_LIBRARIES FSMAC_INCLUDE_DIRS)

