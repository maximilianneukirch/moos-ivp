#--------------------------------------------------------
# The CMakeLists.txt for:      lib_behaviors-colregs-sail
# Author(s):    Mike Benjamin and Maximilian von Neukirch
#--------------------------------------------------------

SET(SRC
  BHV_AvdColregsV22Sail.cpp
  VelocityFilter.cpp
  AOF_R13.cpp
  AOF_R14.cpp
  AOF_R16.cpp
  AOF_CPA.cpp
)

SET(HEADERS 
  BHV_AvdColregsV22Sail.h
  VelocityFilter.h
  AOF_R13.h
  AOF_R14.h
  AOF_R16.h
  AOF_CPA.h
) 

ADD_LIBRARY(BHV_AvdColregsV22Sail SHARED ${SRC})

TARGET_LINK_LIBRARIES(BHV_AvdColregsV22Sail
   bhvutil
   mbutil
   helmivp
   behaviors
   geometry
   ivpbuild
   ivpcore
   logic
   )

