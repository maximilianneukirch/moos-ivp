#--------------------------------------------------------
# The CMakeLists.txt for:                pSimVisionServer
# Author(s):                      Maximilian von Neukirch
#--------------------------------------------------------

SET(SRC
  main.cpp
  FlockEvaluator.cpp
)

ADD_EXECUTABLE(uFlockEvaluator ${SRC})

TARGET_LINK_LIBRARIES(uFlockEvaluator
   ${MOOS_LIBRARIES}
   contacts
   apputil
   mbutil
   ufield
   behaviors
   bhvutil
   ivpbuild
   ivpcore
   logic
   geometry)