##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(qfeel_keyop keyop.cpp)
target_link_libraries(qfeel_keyop ${catkin_LIBRARIES})

install(TARGETS qfeel_keyop
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
