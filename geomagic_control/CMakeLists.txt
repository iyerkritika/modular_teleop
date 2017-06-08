cmake_minimum_required(VERSION 2.8.3)
project(geomagic_control)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg rosconsole tf urdf)

add_message_files(FILES PhantomButtonEvent.msg OmniFeedback.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs genmsg rosconsole tf urdf)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(joint_torque_calc_node src/torque_calc1.cpp)
add_executable(set_joint_torque_node src/set_torque.cpp)

target_link_libraries(joint_torque_calc_node HD HL HDU rt ncurses -lglut -lGL -lGLU ${catkin_LIBRARIES})
target_link_libraries(set_joint_torque_node HD HL HDU rt ncurses -lglut -lGL -lGLU ${catkin_LIBRARIES})

