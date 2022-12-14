# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


set(CPACK_BINARY_DEB "OFF")
set(CPACK_BINARY_FREEBSD "OFF")
set(CPACK_BINARY_IFW "OFF")
set(CPACK_BINARY_NSIS "OFF")
set(CPACK_BINARY_RPM "OFF")
set(CPACK_BINARY_STGZ "ON")
set(CPACK_BINARY_TBZ2 "OFF")
set(CPACK_BINARY_TGZ "ON")
set(CPACK_BINARY_TXZ "OFF")
set(CPACK_BINARY_TZ "ON")
set(CPACK_BUILD_SOURCE_DIRS "/home/adrien/Téléchargements/spatialindex-src-1.9.3;/home/adrien/Téléchargements/spatialindex-src-1.9.3")
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_FILE "/usr/local/share/cmake-3.20/Templates/CPack.GenericDescription.txt")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_SUMMARY "spatialindex built using CMake")
set(CPACK_GENERATOR "TBZ2;TGZ")
set(CPACK_IGNORE_FILES "/\\.gitattributes;/\\.vagrant;/\\.DS_Store;/CVS/;/\\.git/;\\.swp$;~$;\\.\\#;/\\#;CMakeScripts/;_CPack_Packages;cmake_install.cmake;/bin/;/scripts/;/azure-pipelines.yml;.gitignore;.ninja*;HOWTORELEASE.txt;README;build/;CMakeFiles;CTestTestfile.cmake;/docs/build/;/doc/presentations/;package-release.sh;docker-package.sh;.gz2;.bz2")
set(CPACK_INSTALLED_DIRECTORIES "/home/adrien/Téléchargements/spatialindex-src-1.9.3;/")
set(CPACK_INSTALL_CMAKE_PROJECTS "")
set(CPACK_INSTALL_PREFIX "/home/adrien")
set(CPACK_MODULE_PATH "/home/adrien/Téléchargements/spatialindex-src-1.9.3/cmake/modules")
set(CPACK_NSIS_DISPLAY_NAME "libspatialindex 1.9.9")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_NSIS_PACKAGE_NAME "libspatialindex 1.9.9")
set(CPACK_NSIS_UNINSTALL_NAME "Uninstall")
set(CPACK_OUTPUT_CONFIG_FILE "/home/adrien/Téléchargements/spatialindex-src-1.9.3/CPackConfig.cmake")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/local/share/cmake-3.20/Templates/CPack.GenericDescription.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "spatialindex built using CMake")
set(CPACK_PACKAGE_FILE_NAME "spatialindex-src-1.9.3")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "libspatialindex 1.9.9")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "libspatialindex 1.9.9")
set(CPACK_PACKAGE_NAME "libspatialindex")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "libspatialindex Development Team")
set(CPACK_PACKAGE_VERSION "1.9.9")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "9")
set(CPACK_PACKAGE_VERSION_PATCH "9")
set(CPACK_RESOURCE_FILE_LICENSE "/home/adrien/Téléchargements/spatialindex-src-1.9.3/COPYING")
set(CPACK_RESOURCE_FILE_README "/usr/local/share/cmake-3.20/Templates/CPack.GenericDescription.txt")
set(CPACK_RESOURCE_FILE_WELCOME "/usr/local/share/cmake-3.20/Templates/CPack.GenericWelcome.txt")
set(CPACK_RPM_PACKAGE_SOURCES "ON")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_GENERATOR "TBZ2;TGZ")
set(CPACK_SOURCE_IGNORE_FILES "/\\.gitattributes;/\\.vagrant;/\\.DS_Store;/CVS/;/\\.git/;\\.swp$;~$;\\.\\#;/\\#;CMakeScripts/;_CPack_Packages;cmake_install.cmake;/bin/;/scripts/;/azure-pipelines.yml;.gitignore;.ninja*;HOWTORELEASE.txt;README;build/;CMakeFiles;CTestTestfile.cmake;/docs/build/;/doc/presentations/;package-release.sh;docker-package.sh;.gz2;.bz2")
set(CPACK_SOURCE_INSTALLED_DIRECTORIES "/home/adrien/Téléchargements/spatialindex-src-1.9.3;/")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/adrien/Téléchargements/spatialindex-src-1.9.3/CPackSourceConfig.cmake")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "spatialindex-src-1.9.3")
set(CPACK_SOURCE_TOPLEVEL_TAG "Linux-Source")
set(CPACK_STRIP_FILES "")
set(CPACK_SYSTEM_NAME "Linux")
set(CPACK_THREADS "1")
set(CPACK_TOPLEVEL_TAG "Linux-Source")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/home/adrien/Téléchargements/spatialindex-src-1.9.3/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
