#--------------------------------------------------------
# The CMakeLists.txt for:          uNodeReportDistributor
# Author(s):                      Maximilian von Neukirch
#--------------------------------------------------------

SET(SRC
  main.cpp
  NodeReportDistributor.cpp
)

ADD_EXECUTABLE(uNodeReportDistributor ${SRC})

TARGET_LINK_LIBRARIES(uNodeReportDistributor
   ${MOOS_LIBRARIES}
   apputil
   mbutil)