#--------------------------------------------------------
# The CMakeLists.txt for:                       VPFMerger
# Author(s):                      Maximilian von Neukirch
#--------------------------------------------------------

SET(SRC
  main.cpp
  VPFMerger.cpp
)

ADD_EXECUTABLE(uVPFMerger ${SRC})

TARGET_LINK_LIBRARIES(uVPFMerger
   ${MOOS_LIBRARIES}
   apputil
   mbutil)