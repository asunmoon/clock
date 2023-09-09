set( DS1307_srcs "src/DS1307.c")

idf_component_register(SRCS ${DS1307_srcs}
                    INCLUDE_DIRS "include")