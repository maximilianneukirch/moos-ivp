#--------------------------------------------------------
# The CMakeLists.txt for:                pSimVisionServer
# Author(s):                      Maximilian von Neukirch
#--------------------------------------------------------

SET(SRC
  main.cpp
  SimVisionServer.cpp
)

ADD_EXECUTABLE(pSimVisionServer ${SRC})

TARGET_LINK_LIBRARIES(pSimVisionServer
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   geometry)