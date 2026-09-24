include_directories(include)
aux_source_directory(./src srcDir)
add_library(sensorlib STATIC ${srcDir})