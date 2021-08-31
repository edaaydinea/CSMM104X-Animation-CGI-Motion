# Find the native T3M1BASE headers and libraries.
#
#  T3M1BASE_LIBRARIES    - List of libraries when using nana.
#  T3M1BASE_FOUND        - True if nana found.

# Look for the library.
FIND_LIBRARY(T3M1BASE_LIBRARY NAMES FOSSSimT3M1base HINTS ${CMAKE_SOURCE_DIR}/lib)
MARK_AS_ADVANCED(T3M1BASE_LIBRARY)

# handle the QUIETLY and REQUIRED arguments and set T3M1BASE_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(T3M1BASE DEFAULT_MSG T3M1BASE_LIBRARY)

IF(T3M1BASE_FOUND)
  SET(T3M1BASE_LIBRARIES ${T3M1BASE_LIBRARY})
ELSE(T3M1BASE_FOUND)
  SET(T3M1BASE_LIBRARIES)
ENDIF(T3M1BASE_FOUND)