#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CoMISo::CoMISo" for configuration "Release"
set_property(TARGET CoMISo::CoMISo APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(CoMISo::CoMISo PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/CoMISo.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/CoMISo.dll"
  )

list(APPEND _cmake_import_check_targets CoMISo::CoMISo )
list(APPEND _cmake_import_check_files_for_CoMISo::CoMISo "${_IMPORT_PREFIX}/lib/CoMISo.lib" "${_IMPORT_PREFIX}/bin/CoMISo.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
