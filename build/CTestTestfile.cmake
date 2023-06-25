# CMake generated Testfile for 
# Source directory: /home/markus/kaercher
# Build directory: /home/markus/kaercher/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_cleaning_robot "/home/markus/kaercher/build/devel/lib/kaercher/test_kaercher")
set_tests_properties(test_cleaning_robot PROPERTIES  _BACKTRACE_TRIPLES "/home/markus/kaercher/CMakeLists.txt;84;add_test;/home/markus/kaercher/CMakeLists.txt;0;")
subdirs("gtest")
