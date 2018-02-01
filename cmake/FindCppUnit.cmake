# Find CppUnit includes and library
#
# This module defines
#  CppUnit_INCLUDE_DIRS
#  CppUnit_LIBRARIES, the libraries to link against to use CppUnit.
#  CppUnit_LIBRARY_DIRS, the location of the libraries
#  CppUnit_FOUND, If false, do not try to use CppUnit
#
# Copyright Â© 2007, Matt Williams
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

IF (CppUnit_LIBRARIES AND CppUnit_INCLUDE_DIRS)
	SET(CppUnit_FIND_QUIETLY TRUE) # Already in cache, be silent
ENDIF (CppUnit_LIBRARIES AND CppUnit_INCLUDE_DIRS)

IF (WIN32) #Windows
	SET(CppUnit_INCLUDE_SEARCH_DIRS
		${CppUnit_LIBRARY_SEARCH_DIRS}
		${CMAKE_INCLUDE_PATH}
		/usr/include/
		/usr/local/
		/opt/include/
	)
	
	SET(CppUnit_LIBRARY_SEARCH_DIRS
		${CppUnit_LIBRARY_SEARCH_DIRS}
		${CMAKE_LIBRARY_PATH}
		/usr/lib
		/usr/local/lib
		/opt/lib
	)
	FIND_PATH(CppUnit_INCLUDE_DIRS cppunit/Test.h ${CppUnit_INCLUDE_SEARCH_DIRS})
	FIND_LIBRARY(CppUnit_LIBRARIES cppunit PATHS ${CppUnit_LIBRARY_SEARCH_DIRS})
ELSE (WIN32) #Unix
	CMAKE_MINIMUM_REQUIRED(VERSION 2.4.7 FATAL_ERROR)
	FIND_PACKAGE(PkgConfig)
	PKG_SEARCH_MODULE(CppUnit cppunit)
	SET(CppUnit_INCLUDE_DIRS ${CppUnit_INCLUDE_DIRS})
	SET(CppUnit_LIBRARY_DIRS ${CppUnit_LIBDIR})
	SET(CppUnit_LIBRARIES ${CppUnit_LIBRARIES} CACHE STRING "")
ENDIF (WIN32)

#Do some preparation
SEPARATE_ARGUMENTS(CppUnit_INCLUDE_DIRS)
SEPARATE_ARGUMENTS(CppUnit_LIBRARIES)

SET(CppUnit_INCLUDE_DIRS ${CppUnit_INCLUDE_DIRS})
SET(CppUnit_LIBRARIES ${CppUnit_LIBRARIES})
SET(CppUnit_LIBRARY_DIRS ${CppUnit_LIBRARY_DIRS})

MARK_AS_ADVANCED(CppUnit_INCLUDE_DIRS CppUnit_LIBRARIES CppUnit_LIBRARY_DIRS)

IF (CppUnit_INCLUDE_DIRS AND CppUnit_LIBRARIES)
	SET(CppUnit_FOUND TRUE)
ENDIF (CppUnit_INCLUDE_DIRS AND CppUnit_LIBRARIES)

IF (CppUnit_FOUND)
	IF (NOT CppUnit_FIND_QUIETLY)
		MESSAGE(STATUS "  libraries : ${CppUnit_LIBRARIES} from ${CppUnit_LIBRARY_DIRS}")
		MESSAGE(STATUS "  includes  : ${CppUnit_INCLUDE_DIRS}")
	ENDIF (NOT CppUnit_FIND_QUIETLY)
ELSE (CppUnit_FOUND)
	IF (CppUnit_FIND_REQUIRED)
		MESSAGE(FATAL_ERROR "Could not find CppUnit")
	ENDIF (CppUnit_FIND_REQUIRED)
ENDIF (CppUnit_FOUND)
