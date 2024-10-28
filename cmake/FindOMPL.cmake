# FindOMPL.cmake

# Find the OMPL includes and library
set(OMPL_INCLUDE_DIR /usr/local/include/ompl-1.6)

find_library(OMPL_LIBRARY
  NAMES cmaes
  HINTS
    ${OMPL_ROOT}
    $ENV{OMPL_ROOT}
    /usr/local/lib
    /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OMPL
  REQUIRED_VARS OMPL_LIBRARY OMPL_INCLUDE_DIR
)

if(OMPL_FOUND)
  set(OMPL_INCLUDE_DIRS ${OMPL_INCLUDE_DIR})
  set(OMPL_LIBRARIES ${OMPL_LIBRARY})
  if(NOT TARGET OMPL::OMPL)
    add_library(OMPL::OMPL UNKNOWN IMPORTED)
    set_target_properties(OMPL::OMPL PROPERTIES
      IMPORTED_LOCATION "${OMPL_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${OMPL_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(OMPL_INCLUDE_DIR OMPL_LIBRARY)
