execute_process(COMMAND "/home/federico/benzi_ws/src/build/kdl_parser/kdl_parser_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/federico/benzi_ws/src/build/kdl_parser/kdl_parser_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
