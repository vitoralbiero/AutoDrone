cmake_minimum_required(VERSION 2.8)
project( objectTrack )
SET(COMPILE_DEFINITIONS -Werror)
SET(CMAKE_EXE_LINKER_FLAGS -lpthread)
SET(CMAKE_CXX_FLAGS "-pthread -I/usr/local/include -L/usr/local/lib -lwiringPi")
set (GL_SCENE_SOURCES
gl_scenes/models.c
gl_scenes/mirror.c
gl_scenes/yuv.c
gl_scenes/sobel.c
gl_scenes/square.c
gl_scenes/teapot.c)

#OPENCV
find_package( OpenCV REQUIRED )
link_directories( /home/pi/libfacerec-0.04) 

include_directories(/opt/vc/include)
include_directories(/opt/vc/include/EGL)
include_directories(/opt/vc/userland/host_applications/linux/libs/bcm_host/include)
include_directories(/opt/vc/userland/interface/vcos)
include_directories(/opt/vc/userland)
include_directories(/opt/vc/userland/interface/vcos/pthreads)
include_directories(/opt/vc/userland/interface/vmcs_host/linux)
include_directories(/home/pi/objectTrack)
include_directories(/home/pi/wiringPi)
add_executable(objectTrack RaspiCamControl.c RaspiCLI.c RaspiPreview.c objectTrack.cpp RaspiTex.c RaspiTexUtil.c tga.c ${GL_SCENE_SOURCES})
target_link_libraries(objectTrack m /opt/vc/lib/libmmal_core.so /opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so /opt/vc/lib/libbcm_host.so /opt/vc/lib/libGLESv2.so /opt/vc/lib/libEGL.so ${OpenCV_LIBS})
