cmake_minimum_required(VERSION 3.1)
project( DisplayImage )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
 
add_executable( client client.cpp )
target_link_libraries( client ${OpenCV_LIBS} )
target_link_libraries(client Threads::Threads)
 
add_executable( server server.cpp )
target_link_libraries( server ${OpenCV_LIBS} )
target_link_libraries(server Threads::Threads)

add_executable( serial test_serial.cpp )
target_link_libraries( serial ${OpenCV_LIBS} )
target_link_libraries(serial Threads::Threads)
