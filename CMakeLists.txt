
PROJECT(waypoint_navigation)

cmake_minimum_required(VERSION 2.6)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
find_package(PkgConfig)

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(/usr/include/eigen2)

ADD_LIBRARY(waypoint_navigation SHARED WaypointNavigation.cpp)
#TARGET_LINK_LIBRARIES(xsens xsensVendorLib) 
#${DFKI_TYPES_LIBS})

#need to link to some other libraries ? just add them here
#TARGET_LINK_LIBRARIES(driver png jpeg)

ADD_EXECUTABLE(WaypointNavigationTest WaypointNavigationTest.cpp)
TARGET_LINK_LIBRARIES(WaypointNavigationTest waypoint_navigation)
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS waypoint_navigation
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES WaypointNavigation.hpp DESTINATION include)

CONFIGURE_FILE(waypoint_navigation.pc.in waypoint_navigation.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/waypoint_navigation.pc DESTINATION lib/pkgconfig)



