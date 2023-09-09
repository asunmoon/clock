set( clock_srcs "src/clock.cpp")

idf_component_register(SRCS ${clock_srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES    spi_ws2812  DS1307-master)