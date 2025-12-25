add_library(image_preproc src/brightness_correction.cpp src/gamma_corrector.cpp)
target_include_directories(image_preproc PUBLIC include)
target_link_libraries(image_preproc PUBLIC ${OpenCV_LIBS})
