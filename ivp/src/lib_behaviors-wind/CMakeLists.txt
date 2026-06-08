#--------------------------------------------------------
# The CMakeLists.txt for:              lib_behaviors-wind
# Author(s):                       Maximilian v. Neukirch
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
  BHV_WindManeuverability.cpp
)

SET(HEADERS
  BHV_WindManeuverability.h
) 

ADD_LIBRARY(BHV_WindManeuverability SHARED ${SRC})

TARGET_LINK_LIBRARIES(BHV_WindManeuverability
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

