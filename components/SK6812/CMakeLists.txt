idf_component_register(
    SRCS            "SK6812.c" "led_strip_rmt_ws2812.c" "led_rgb_hsv.c"
    INCLUDE_DIRS    "include"
    REQUIRES driver freertos struct_component
    PRIV_REQUIRES
)

#add_definitions(-DC3_TX)
add_definitions(-DC3_RX)
