set(SOLAR_TEST_SUPPORT_DIR ${CMAKE_SOURCE_DIR}/src/tests/support)

add_executable(test_mpu6050_publisher
    ${SOLAR_TEST_SUPPORT_DIR}/test_main.cpp
    test_mpu6050_publisher.cpp
)

target_include_directories(test_mpu6050_publisher PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_mpu6050_publisher PRIVATE solar_sensors)
solar_enable_coverage(test_mpu6050_publisher)

add_test(NAME Mpu6050Publisher_StartFailsWithoutCallback COMMAND test_mpu6050_publisher --run Mpu6050Publisher_StartFailsWithoutCallback)
add_test(NAME Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch COMMAND test_mpu6050_publisher --run Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch)
add_test(NAME Mpu6050Publisher_StartInitialisesDevice_WhenWhoAmIMatches COMMAND test_mpu6050_publisher --run Mpu6050Publisher_StartInitialisesDevice_WhenWhoAmIMatches)
