# FOSSSim Executable

append_files (Headers "h"   . ./StableFluids/)
append_files (Sources "cpp" . ./StableFluids/)

# Locate OpenGL
find_package (OpenGL REQUIRED)
if (OPENGL_FOUND)
  include_directories (${OPENGL_INCLUDE_DIR})
  set (FOSSSim_LIBRARIES ${FOSSSim_LIBRARIES} ${OPENGL_LIBRARIES})
else (OPENGL_FOUND)
  message (SEND_ERROR "Unable to locate OpenGL")
endif (OPENGL_FOUND)

# Locate GLUT
find_package (GLUT REQUIRED glut)
if (GLUT_FOUND)
  include_directories (${GLUT_INCLUDE_DIR})
  set (FOSSSim_LIBRARIES ${FOSSSim_LIBRARIES} ${GLUT_glut_LIBRARY})
else (GLUT_FOUND)
  message (SEND_ERROR "Unable to locate GLUT")
endif (GLUT_FOUND)

# RapidXML library is required
find_package (RapidXML REQUIRED)
if (RAPIDXML_FOUND)
  include_directories (${RAPIDXML_INCLUDE_DIR})
else (RAPIDXML_FOUND)
  message (SEND_ERROR "Unable to locate RapidXML")
endif (RAPIDXML_FOUND)

# TCLAP library is required
find_package (TCLAP REQUIRED)
if (TCLAP_FOUND)
  include_directories (${TCLAP_INCLUDE_PATH})
else (TCLAP_FOUND)
  message (SEND_ERROR "Unable to locate TCLAP")
endif (TCLAP_FOUND)

option (USE_PNG "Builds in support for exporting to png" OFF)
if (USE_PNG)
  find_package (PNG)
  if (PNG_FOUND)
    add_definitions (-DPNGOUT)
    add_definitions (${PNG_DEFINITIONS})
    include_directories (${PNG_INCLUDE_DIR})
    set (FOSSSim_LIBRARIES ${FOSSSim_LIBRARIES} ${PNG_LIBRARIES})
  endif (PNG_FOUND)
endif (USE_PNG)

find_package (T6M1base REQUIRED)
if (T6M1BASE_FOUND)
  set (FOSSSim_LIBRARIES ${T6M1BASE_LIBRARIES} ${FOSSSim_LIBRARIES})
else (T6M1BASE_FOUND)
  message (SEND_ERROR "Unable to locate T6M1 Base Library")
endif (T6M1BASE_FOUND)

add_executable (FOSSSim ${Headers} ${Templates} ${Sources})
target_link_libraries (FOSSSim ${FOSSSim_LIBRARIES})

INSTALL_TARGETS(/bin FOSSSim)
