#--------------------------------------------------------
# The CMakeLists.txt for:       lib_behaviors-marine-sail
# Author(s):                       Maximilian v. Neukirch
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
  BHV_AvoidCollisionSail.cpp
)

SET(HEADERS
  BHV_AvoidCollisionSail.h
) 

ADD_LIBRARY(BHV_AvoidCollisionSail SHARED ${SRC})

TARGET_LINK_LIBRARIES(BHV_AvoidCollisionSail
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

