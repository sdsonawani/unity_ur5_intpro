cmake_minimum_required(VERSION 2.8.3)
project(roboteq_diff_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs nav_msgs)

add_message_files(FILES
                  AuxData.msg
                  ControlData.msg
                  ErrorData.msg
                  TuningData.msg
                 )

add_service_files(FILES
                  EMO.srv
                  RequestBrushActivate.srv
                  RequestBrushSpeed.srv
                  RequestChargeActivate.srv
                  ToggleClosedLoop.srv
                 )

generate_messages(DEPENDENCIES std_msgs nav_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs)
