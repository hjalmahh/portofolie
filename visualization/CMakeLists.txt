find_package(imgui CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
set(THREEPP_USE_EXTERNAL_GLFW ON)

include(FetchContent)
FetchContent_Declare(
    threepp
    GIT_REPOSITORY https://github.com/markaren/threepp.git
    GIT_TAG a586dcfa0b123e475670bf8731d400c473426ec0
)
FetchContent_MakeAvailable(threepp)

set(PUBLIC_HEADERS
    include/visualization/robotscene.h

    include/visualization/imgui/widgets.h
    include/visualization/imgui/imguiwindowcontext.h

    include/visualization/imgui/windows/toolwindow.h
    include/visualization/imgui/windows/posedisplaywindow.h
    include/visualization/imgui/windows/robotcontrolwindows.h
    include/visualization/imgui/windows/trajectoryloggerwindow.h
    include/visualization/imgui/windows/worldobjectconfigurationwindow.h
)

add_library(visualization
    src/robotscene.cpp

    src/imgui/imguiwindowcontext.cpp

    src/imgui/windows/toolwindow.cpp
    src/imgui/windows/posedisplaywindow.cpp
    src/imgui/windows/robotcontrolwindows.cpp
    src/imgui/windows/trajectoryloggerwindow.cpp
    src/imgui/windows/worldobjectconfigurationwindow.cpp

    ${PUBLIC_HEADERS}
)

target_link_libraries(visualization PUBLIC threepp::threepp imgui::imgui assimp::assimp simulation PRIVATE glfw)
target_include_directories(visualization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/ PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/)

