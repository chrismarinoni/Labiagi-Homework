add_executable(alberto_node alberto_node.cpp)
add_executable(alberto_node_sub alberto_node_sub.cpp)
target_link_libraries(alberto_node alberto_library ${catkin_LIBRARIES})
target_link_libraries(alberto_node_sub alberto_library ${catkin_LIBRARIES})
