cmake_minimum_required(VERSION 2.8) # 指定 cmake 的最小版本
project(TIA_SDK C CXX) # 设置项目名称为TIA_SDK，并指定项目采用c c++语言

#########################################################
# version 
# 设置TIA_SDK版本 SDKTIA_SDK_VERSION 1.1.2
set(TIA_SDK_VERSION_MAJOR 1)
set(TIA_SDK_VERSION_MINOR 1)
set(TIA_SDK_VERSION_PATCH 2)
set(TIA_SDK_VERSION ${TIA_SDK_VERSION_MAJOR}.${TIA_SDK_VERSION_MINOR}.${TIA_SDK_VERSION_PATCH})

##########################################################
# Detect wordsize:
# 检测字长 64位 or 32位
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MRPT_WORD_SIZE 64)
ELSE()
	SET(CMAKE_MRPT_WORD_SIZE 32)
ENDIF()

##################################################
#c++ 11
# 设置 C++ 编译选项 c++ 11 
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

##########################################################

# 系统为Linux or Windows
IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "Current platform: Linux")
	#Linux add -fPIC
	add_compile_options(-fPIC)
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "Current platform: Windows")
	add_compile_options("/utf-8")
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")
#or
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#####################################################
# add cmake module path
# CMAKE_MODULE_PATH cmake模块所在的路径
# CMAKE_CURRENT_SOURCE_DIR 当前处理的CMakeLists.txt所在的路径
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")  # 追加
set(SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#############################################################################
# include cmake file
include(common/ydlidar_base)
include(install_package)

#############################################################################
# Policy CMP0023 allows to mix old and new interfaces of target_link_libraries
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW) # We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0005 NEW) # Escape definitions (-D) strings
	if(POLICY CMP0053)
		cmake_policy(SET CMP0053 OLD) # Simplify variable reference and escape sequence evaluation.
	endif()
	if(POLICY CMP0037)
		cmake_policy(SET CMP0037 OLD)  # Allow defining target "test"
	endif()
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif()


#############################################################################
# option
option( BUILD_SHARED_LIBS "Build shared libraries." OFF)
option( BUILD_EXAMPLES "Build Example." ON)
# option( BUILD_CSHARP "Build CSharp." ON)
# option( BUILD_TEST "Build Test." ON)

############################################################################
# find package
FIND_PACKAGE(SWIG)
find_package(PythonInterp)
FIND_PACKAGE(PythonLibs)
find_package(GTest)

############################################################################
# include headers
# 将指定目录添加到编译器的头文件搜索路径之下
include_directories(.)
include_directories(core)
include_directories(src)

#############################################################################
# addd subdirectory
# 添加一个子目录core src 并构建该子目录core src 
add_subdirectory(core)
add_subdirectory(src)

##############################
#build examples
# 添加一个子目录samples并构建该子目录samples
if(BUILD_EXAMPLES)
add_subdirectory(samples)
endif()

#############################################################################
# PARSE libraries
include(common/ydlidar_parse)
include_directories(${SDK_INCS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

########################################################
## Create configure file for inclusion in library
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ydlidar_config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/ydlidar_config.h" )
set( GENERATED_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/ydlidar_config.h )


#############################################################################
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#############################################################################################
#shared library
if(BUILD_SHARED_LIBS)
ydlidar_add_library(${PROJECT_NAME} SHARED ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
else()
ydlidar_add_library(${PROJECT_NAME} STATIC ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
endif()

target_link_libraries(${PROJECT_NAME} ${SDK_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

#############################################################################################
#static library
#ydlidar_add_library(${PROJECT_NAME}_static STATIC ${SDK_SOURCES} ${SDK_HEADERS} ${GENERATED_HEADERS})
#SET_TARGET_PROPERTIES(${PROJECT_NAME}_static PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
#SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
#SET_TARGET_PROPERTIES(${PROJECT_NAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

#target_link_libraries(${PROJECT_NAME}_static ${SDK_LIBS})
#target_include_directories(${PROJECT_NAME}_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)


############################################################################
# enable test
enable_testing()

############################################################################
# build ydlidar sdk python version
# if(${SWIG_FOUND} AND ${PYTHONLIBS_FOUND})
#     message(STATUS "build python API....")
#     include_directories(python)
#     add_subdirectory(python)
# endif()

#############################################
# build ydlidar sdk c# 
# if(WIN32)
# if(BUILD_CSHARP)
#     message(STATUS "build csharp API....")
#     include_directories(csharp)
#     add_subdirectory(csharp)
# endif()
# endif()

##############################################################################
# test
# if(GTEST_FOUND AND BUILD_TEST)
#     message(STATUS "build test is ON.....")
#     include_directories(test)
#     add_subdirectory(test)
# endif()

###############################################################################
# append path
list(APPEND SDK_INCS ${CMAKE_INSTALL_PREFIX}/include/src
    ${CMAKE_INSTALL_PREFIX}/include)


###############################################################################
# install package
string(TOUPPER ${PROJECT_NAME} PROJECT_PKG_NAME)
install_package(
    PKG_NAME ${PROJECT_PKG_NAME}
    LIB_NAME ${PROJECT_NAME}
    VERSION ${TIA_SDK_VERSION}
    INSTALL_HEADERS ${SDK_HEADERS}
    INSTALL_GENERATED_HEADERS ${GENERATED_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    INCLUDE_DIRS ${SDK_INCS}
    LINK_LIBS ${SDK_LIBS}
    )
#########################################################################################
# install doc
install(DIRECTORY cmake/ DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_PKG_NAME})
install(FILES LICENSE.txt README.md DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_PKG_NAME}/)
install(DIRECTORY doc DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_PKG_NAME})
IF (WIN32)
ELSE()
INSTALL(PROGRAMS startup/initenv.sh DESTINATION startup)
ENDIF()

include_directories(${CMAKE_BINARY_DIR} )

##################################################################################
# make an uninstall target
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in)
add_custom_target(uninstall
    COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")



########################################################
### make package or cpack -C CPackConfig.cmake
# packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${TIA_SDK_VERSION})
set(CPACK_PACKAGE_VERSION ${TIA_SDK_VERSION})
set(CPACK_PACKAGE_CONTACT ${package-contact})
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF) # TODO: review packaging for linux boards
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "YDLIDAR SDK.")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Tony Yang")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${TIA_SDK_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${TIA_SDK_VERSION}")
set(CPACK_SOURCE_GENERATOR "ZIP;TBZ2")
set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
set(CPACK_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
set(CPACK_SET_DESTDIR true)
set(CPACK_OUTPUT_FILE_PREFIX  ${CMAKE_CURRENT_BINARY_DIR})
if ("${CMAKE_SYSTEM}" MATCHES "Linux")
        set(CPACK_GENERATOR "TBZ2")
        find_program(DPKG_PROGRAM dpkg)
        if (EXISTS ${DPKG_PROGRAM})
                list (APPEND CPACK_GENERATOR "DEB")
        endif()
else()
        set(CPACK_GENERATOR "ZIP")
endif()

include(CPack)

# Summary
include(cmake/script_show_final_summary.cmake REQUIRED)

