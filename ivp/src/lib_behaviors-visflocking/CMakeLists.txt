#--------------------------------------------------------
# The CMakeLists.txt for:       lib_behaviors-visflocking
# Author(s):                       Maximilian v. Neukirch
#--------------------------------------------------------

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
  ${MOOS_LIBRARIES}
  geometry
  ivpbuild
  ivpcore
  mbutil
)

