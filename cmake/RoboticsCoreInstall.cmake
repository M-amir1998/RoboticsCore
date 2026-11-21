# Install rules and CMake package configuration:
#   find_package(RoboticsCore CONFIG REQUIRED)
#   target_link_libraries(app PRIVATE RoboticsCore::RoboticsCore)

if(NOT ROBOTICSCORE_INSTALL)
  return()
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

get_property(roboticscore_targets GLOBAL PROPERTY ROBOTICSCORE_LIBRARY_TARGETS)

install(
  TARGETS ${roboticscore_targets} roboticscore_eigen roboticscore_all
  EXPORT RoboticsCoreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/roboticscore
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY ${PROJECT_BINARY_DIR}/include/roboticscore
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.hpp")

if(ROBOTICSCORE_EIGEN_BUNDLED)
  install(DIRECTORY ${ROBOTICSCORE_EIGEN_SOURCE_DIR}/Eigen
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/eigen3)
  install(FILES ${ROBOTICSCORE_EIGEN_SOURCE_DIR}/COPYING.MPL2
          DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/eigen3)
endif()

set(config_install_dir ${CMAKE_INSTALL_LIBDIR}/cmake/RoboticsCore)

install(
  EXPORT RoboticsCoreTargets
  NAMESPACE RoboticsCore::
  DESTINATION ${config_install_dir})

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/RoboticsCoreConfig.cmake.in
  ${PROJECT_BINARY_DIR}/RoboticsCoreConfig.cmake
  INSTALL_DESTINATION ${config_install_dir})

write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/RoboticsCoreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion)

install(FILES ${PROJECT_BINARY_DIR}/RoboticsCoreConfig.cmake
              ${PROJECT_BINARY_DIR}/RoboticsCoreConfigVersion.cmake
        DESTINATION ${config_install_dir})

install(FILES ${PROJECT_SOURCE_DIR}/LICENSE DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/RoboticsCore)
