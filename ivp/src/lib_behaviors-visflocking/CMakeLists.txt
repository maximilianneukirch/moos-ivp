#--------------------------------------------------------
# The CMakeLists.txt for:       lib_behaviors-visflocking
# Author(s):                       Maximilian v. Neukirch
#--------------------------------------------------------

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
  BHV_VisFlocking.cpp
  VisionModel.cpp
)

SET(HEADERS
  BHV_VisFlocking.h
  VisionModel.h
) 

ADD_LIBRARY(BHV_VisFlocking SHARED ${SRC})

TARGET_LINK_LIBRARIES(BHV_VisFlocking
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

