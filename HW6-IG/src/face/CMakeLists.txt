add_library(calogero_library SHARED
  calogero.cpp calogero.h)

target_link_libraries(calogero_library ${catkin_LIBRARIES})
target_link_libraries(calogero_library ${OpenCV_LIBS})
