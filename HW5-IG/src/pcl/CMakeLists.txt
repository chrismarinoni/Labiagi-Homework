add_library(bartolo_library SHARED
   bartolo.cpp bartolo.h)

target_link_libraries(bartolo_library ${catkin_LIBRARIES})
target_link_libraries(bartolo_library ${OpenCV_LIBS})
target_link_libraries(bartolo_library ${PCL_LIBRARIES})
