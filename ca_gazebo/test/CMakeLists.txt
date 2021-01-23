find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

roslaunch_add_file_check(../launch)

add_rostest(create_tests.test)
