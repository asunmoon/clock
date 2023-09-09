set( spi_ws2812b_srcs "src/spi_ws2812b.c")
 idf_component_register(SOSRC_DIRS  "${spi_ws2812b_srcs}"
                 INCLUDE_DIRS "include")