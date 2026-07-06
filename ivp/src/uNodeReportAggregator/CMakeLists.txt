#--------------------------------------------------------
# The CMakeLists.txt for:           uNodeReportAggregator
# Author(s):                      Maximilian von Neukirch
#--------------------------------------------------------

SET(SRC
  main.cpp
  NodeReportAggregator.cpp
)

ADD_EXECUTABLE(uNodeReportAggregator ${SRC})

TARGET_LINK_LIBRARIES(uNodeReportAggregator
   ${MOOS_LIBRARIES}
   apputil
   mbutil)