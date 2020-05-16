##############################################################################
# CUCKOO SEARCH
##############################################################################

add_compile_options(-std=c++11)

# find_package(cpprestsdk REQUIRED)
include_directories(include)
add_library(cuckoo_search_lib src/CuckooSearch.cpp)
# add_dependencies(cuckoo_search_lib ca_driver)
target_link_libraries(cuckoo_search_lib tbb)

install(TARGETS cuckoo_search_lib
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
