add_library(alberto_library SHARED
  hw1OpenCv.cpp hw1OpenCv.h)

target_link_libraries(alberto_library ${catkin_LIBRARIES})
target_link_libraries(alberto_library ${OpenCV_LIBS})
