#必填项，表明cmake的最低版本要求
cmake_minimum_required(VERSION 3.4.1)

#声明库名称为SerialPort, 类型为动态库SHARED, 对应的源文件SerialPort.c
add_library(SerialPort SHARED
            SerialPort.c)

# Include libraries needed for libserial_port lib
target_link_libraries(SerialPort
                      android
                      log)
