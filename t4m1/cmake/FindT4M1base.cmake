# Find the native T4M1BASE headers and libraries.
#
#  T4M1BASE_LIBRARIES    - List of libraries when using nana.
#  T4M1BASE_FOUND        - True if nana found.

# Look for the library.
FIND_LIBRARY(T4M1BASE_LIBRARY NAMES FOSSSimT4M1base HINTS ${CMAKE_SOURCE_DIR}/lib)
MARK_AS_ADVANCED(T4M1BASE_LIBRARY)

# handle the QUIETLY and REQUIRED arguments and set T4M1BASE_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(T4M1BASE DEFAULT_MSG T4M1BASE_LIBRARY)

IF(T4M1BASE_FOUND)
  SET(T4M1BASE_LIBRARIES ${T4M1BASE_LIBRARY})
ELSE(T4M1BASE_FOUND)
  SET(T4M1BASE_LIBRARIES)
ENDIF(T4M1BASE_FOUND)