project(tests CXX)

add_maya_executable(NAME test_meshChecker
        SOURCE
        test_meshChecker.cpp
        )
target_include_directories(test_meshChecker SYSTEM
        PRIVATE ../src/
        )
target_link_libraries(test_meshChecker
        PRIVATE
        meshChecker
        )
