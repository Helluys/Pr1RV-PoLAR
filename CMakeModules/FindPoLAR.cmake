# - Find PoLAR
# Find PoLAR includes and library
# This module defines
#  PoLAR_INCLUDE_DIR, where to find PoLAR headers
#  PoLAR_INCLUDE_DIRS, which is the same as PoLAR_INCLUDE_DIR.
#  PoLAR_LIBRARY_DIR, where to find the libraries
#  PoLAR_LIBRARY_DIRS, which is the same as PoLAR_LIBRARY_DIR.
#  PoLAR_LIBRARIES, the libraries needed to use PoLAR.
#  PoLAR_FOUND, If false, do not try to use PoLAR.
 
FIND_PATH(PoLAR_INCLUDE_DIR
    NAMES PoLAR/Viewer.h
    PATHS /usr/include /usr/local/include ${PoLAR_INSTALL_DIR}/include $ENV{PoLAR_INSTALL_DIR}/include  $ENV{PoLAR_BUILD_DIR}/../include 
 )
 
SET(PoLAR_NAMES ${PoLAR_NAMES} libPoLAR.a libPoLAR.so libPoLAR.dylib PoLAR.lib)
FIND_PATH(PoLAR_LIBRARY_DIR
    NAMES ${PoLAR_NAMES}
    PATHS /usr/lib64 /usr/lib /usr/local/lib64 /usr/local/lib 
    ${PoLAR_INSTALL_DIR}/lib64 ${PoLAR_INSTALL_DIR}/lib ${PoLAR_INSTALL_DIR}/bin
    $ENV{PoLAR_INSTALL_DIR}/lib64 $ENV{PoLAR_INSTALL_DIR}/lib $ENV{PoLAR_INSTALL_DIR}/bin
    $ENV{PoLAR_BUILD_DIR}/lib64 $ENV{PoLAR_BUILD_DIR}/lib $ENV{PoLAR_BUILD_DIR}/bin
    )

IF (PoLAR_LIBRARY_DIR AND PoLAR_INCLUDE_DIR)
      set(PoLAR_INCLUDE_DIRS ${PoLAR_INCLUDE_DIR})
      set(PoLAR_LIBRARY_DIRS ${PoLAR_LIBRARY_DIR})
      set(PoLAR_LIBRARIES PoLAR
      )
ENDIF ()

# handle the QUIETLY and REQUIRED arguments and set PoLAR_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PoLAR
                                  DEFAULT_MSG PoLAR_INCLUDE_DIRS PoLAR_LIBRARY_DIRS PoLAR_LIBRARIES)

mark_as_advanced(PoLAR_INCLUDE_DIRS PoLAR_INCLUDE_DIR PoLAR_LIBRARY_DIRS PoLAR_LIBRARY_DIR PoLAR_LIBRARY_DIR PoLAR_LIBRARIES)
