include_directories("${CMAKE_CURRENT_SOURCE_DIR}/internLibs")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

aux_source_directory(./src srcDir)
aux_source_directory(./internlibs/bosch_BME280 srcDir)
aux_source_directory(./internlibs/melexis_mlx90640 srcDir)

add_library(sensorlib STATIC ${srcDir})
