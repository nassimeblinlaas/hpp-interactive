# Copyright 2014, Florent Lamiraux, CNRS
#
# This file is part of hpp-constraints.
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
MESSAGE(STATUS ${PROJECT_NAME})
SET(LIBRARY_NAME ${PROJECT_NAME})
SET(SIGMA7_LIB "-L/home/nblin/sdk-3.6.0/lib/release/lin-x86_64-gcc -ldhd -ldrd -lusb-1.0")
SET(ZMQ_LIB "-lzmq")
ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    planner.cc
    sixDOFMouseDriver.cc
    ObjectCoordinate.cc
    ArmConfigurationForce.cc
    CollisionPair.cc
    Data.cc
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} gepetto-viewer-corba)

# Create and install executable running the corba server
ADD_EXECUTABLE (interactive-server
	  server.cc
		)

TARGET_LINK_LIBRARIES(interactive-server ${LIBRARY_NAME} ${SIGMA7_LIB} ${ZMQ_LIB})
TARGET_LINK_LIBRARIES(interactive-server gepetto-viewer-corba)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

INSTALL(TARGETS interactive-server DESTINATION ${CMAKE_INSTALL_BINDIR})
