find_path(
  bcm2835_INCLUDE_DIR
  NAMES bcm2835.h
  HINTS /usr/local/include)
find_library(
  bcm2835_LIBRARY
  NAMES bcm2835
  HINTS /usr/local/lib)
mark_as_advanced(bcm2835_INCLUDE_DIR bcm2835_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(bcm2835 REQUIRED_VARS bcm2835_INCLUDE_DIR
                                                        bcm2835_LIBRARY)

if(bcm2835_FOUND AND NOT TARGET bcm2835)
  add_library(bcm2835 UNKNOWN IMPORTED)
  set_target_properties(
    bcm2835
    PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES "C"
               IMPORTED_LOCATION "${bcm2835_LIBRARY}"
               INTERFACE_INCLUDE_DIRECTORIES "${bcm2835_INCLUDE_DIR}")
endif()
