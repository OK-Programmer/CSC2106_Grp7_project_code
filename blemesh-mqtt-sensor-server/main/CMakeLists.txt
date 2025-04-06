set(srcs "main.cpp"
        "board.c" "DHT.cpp" "temperature.cpp")

idf_component_register(SRCS "${srcs}"
        INCLUDE_DIRS  ".")