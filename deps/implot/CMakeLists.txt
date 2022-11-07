add_library(
        implot
        implot/implot.h
        implot/implot_internal.h
        implot/implot.cpp
        implot/implot_items.cpp
        implot/implot_demo.cpp
)

target_link_libraries(implot PUBLIC imgui)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIMGUI_IMPL_OPENGL_LOADER_GLAD2")

target_include_directories(
        implot
        PUBLIC
        implot
)