#--------------------------------------------------------
# The CMakeLists.txt for:          lib_behaviors-boundary
# Author(s):                       Maximilian v. Neukirch
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
  BHV_SoftBoundary.cpp
)

SET(HEADERS
  BHV_SoftBoundary.h
) 

ADD_LIBRARY(BHV_SoftBoundary SHARED ${SRC})

TARGET_LINK_LIBRARIES(BHV_SoftBoundary
  -Wl,--start-group
  helmivp
  logic
  behaviors
  bhvutil
  geometry
  ivpbuild
  ivpcore
  mbutil
  -Wl,--end-group
)

