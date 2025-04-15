# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS "main.cpp" "dshot/dshot_esc_encoder.c" "dshot/dshot.cpp" "gpio/gpio_config.cpp" "adc/adc.cpp" "encoder/encoder.cpp" "irreceiver/irreceiver.cpp" "adccontinuos/adccontinuos.cpp" "mpu/mpu.cpp" "wifi/wifi.cpp" "websocket/websocket.cpp"       # list the source files of this component
    INCLUDE_DIRS "."
)

set(CMAKE_CXX_STANDARD 17)