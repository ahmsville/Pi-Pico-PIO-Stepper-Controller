cmake_minimum_required(VERSION 3.20)
include(pico_sdk_import.cmake)
project(My_pico_projects)

pico_sdk_init()

add_executable(main
        main.cpp
        )


# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/stepper.pio)






# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(main pico_stdlib)
# Add the standard library and hardware adc to the build
target_link_libraries(main pico_stdlib hardware_adc) 
target_link_libraries(main pico_stdlib hardware_pio) 
target_link_libraries(main pico_stdlib hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)

# Enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

