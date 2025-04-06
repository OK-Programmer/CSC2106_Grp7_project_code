set(srcs "main.cpp" "test_irremote.cpp")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS  ".")
