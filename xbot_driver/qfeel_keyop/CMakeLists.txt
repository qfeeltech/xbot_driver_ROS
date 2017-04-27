cmake_minimum_required(VERSION 2.8.3)
project(qfeel_keyop)
find_package(catkin REQUIRED COMPONENTS ecl_exceptions
                                        ecl_threads
                                        ecl_time
                                        geometry_msgs
                                        std_msgs
                                        roscpp
					                    message_generation)




add_message_files(
  FILES
  location.msg
)
generate_messages(DEPENDENCIES
   std_msgs)  

catkin_package(CATKIN_DEPENDS 
                message_runtime)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(qfeel_keyop src/main.cpp src/keyop.cpp)
target_link_libraries(qfeel_keyop ${catkin_LIBRARIES})
add_dependencies(qfeel_keyop qfeel_keyop_generate_messages_cpp)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
