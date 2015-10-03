# Copyright 2015, Nassime BLIN, CNRS
#
# This file is part of hpp-interactive.
# hpp-constraints is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-constraints is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-constraints. If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)

SET(PROJECT_NAME hpp-interactive)
SET(PROJECT_DESCRIPTION
	"Interactive path planning")
SET(PROJECT_URL "")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/interactive/planner.hh
	include/hpp/interactive/sixDOFMouseDriver.hh
  include/hpp/interactive/fwd.hh
)

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

SETUP_PROJECT()
ADD_REQUIRED_DEPENDENCY("hpp-core")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("gepetto-viewer-corba")
ADD_REQUIRED_DEPENDENCY("hpp-model")



PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
ADD_SUBDIRECTORY (src)
#ADD_SUBDIRECTORY (tests)

SETUP_PROJECT_FINALIZE()
