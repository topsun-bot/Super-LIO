# FindGlog.cmake - Find Google glog (libgoogle-glog-dev on Debian/Ubuntu)
find_path(GLOG_INCLUDE_DIR NAMES glog/logging.h
  PATHS /usr/include /usr/local/include)
find_library(GLOG_LIBRARY NAMES glog
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Glog DEFAULT_MSG GLOG_LIBRARY GLOG_INCLUDE_DIR)

mark_as_advanced(GLOG_INCLUDE_DIR GLOG_LIBRARY)

if(Glog_FOUND)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
  if(NOT TARGET glog::glog)
    add_library(glog::glog UNKNOWN IMPORTED)
    set_target_properties(glog::glog PROPERTIES
      IMPORTED_LOCATION "${GLOG_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}")
  endif()
endif()