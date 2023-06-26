add_library(stb STATIC
        stb/stb.h
        stb/stb_image.h
        stb/stb_implementation.cpp
        stb/stb_truetype.h
        stb/stb_vorbis.c
)

target_include_directories(stb PUBLIC ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(stb PROPERTIES LINKER_LANGUAGE CXX)