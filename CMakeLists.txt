#Para acrescentar novos arquivos:
#Acrescente o .cpp em set(SOURCE_FILES ...) e o diretorio dos .h e .hpp em include_directories()

#Para incluir novas biliotecas:
#utilize pkg_check_modules(NOME pacotes) para defini-la, seguido de link_directories(${NOME_LIBRARY_DIRS}),
#acrescente ${NOME_INCLUDE_DIRS} em include_directories()
#e insira target_link_libraries(VSSS ${NOME_LIBRARIES}) no final do arquivo

cmake_minimum_required(VERSION 3.5)
project(VSSS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

#Encontra bibliotecas
find_package(PkgConfig)

pkg_check_modules(GTKMM gtkmm-3.0)
link_directories(${GTKMM_LIBRARY_DIRS})

pkg_check_modules(V4L2 libv4l2 libv4lconvert)
link_directories(${V4L2_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS date_time filesystem wserialization system serialization thread regex)

find_package(OpenCV 4.0.0 REQUIRED)

find_package(CUDA QUIET)

find_library(LIBXBEE libxbee.so libxbeep.so)

#Define arquivos a serem compilados. Novos arquivos devem ser inclusos aqui
set(SOURCE_FILES cc/main.cpp
    cc/controlGUI.cpp
    cc/filechooser.cpp
    cc/CamCap.cpp
    cc/StrategyGUI.cpp
    cc/Xbee.cpp
    cc/Messenger.cpp
    cc/jsonSaveManager.cpp
    cc/TestFrame.cpp
    cc/LS.cpp
    cc/vision/tag.cpp
    cc/vision/vision.cpp
    cc/vision/visionGUI.cpp
    cc/Geometry/Geometry.cpp
    cc/Strategy2/Robot2.cpp
    cc/Strategy2/Attacker.cpp
    cc/Strategy2/Goalkeeper.cpp
    cc/Strategy2/Defender.cpp
    cc/lib/json.hpp
    cc/Strategy2/Field.cpp
    cc/Strategy2/Strategy2.cpp
    pack-capture/capture/v4lcap.cpp
    pack-capture/capture/v4ldev.hpp
    pack-capture-gui/capture-gui/ImageView.cpp
    pack-capture-gui/capture-gui/V4LInterface-aux.cpp
    pack-capture-gui/capture-gui/V4LInterface-events.cpp
    cc/Strategy2/Strategy2.cpp
    pack-capture-gui/capture-gui/RobotGUI.cpp
    pack-capture-gui/capture-gui/ImageArt.cpp
    pack-capture-gui/capture-gui/ImageWarp.cpp
    cc/TestOnClick.cpp
    cc/VideoRecorder.cpp
    cc/Strategy2/UnivectorField.cpp cc/Strategy2/Adversary.hpp)

#Define diretorios com .h e .hpp. Novos diretorios devem ser inclusos aqui
include_directories(cc/
    cc/Fuzzy/
    cc/vision/
    cc/lib/
    cc/Strategy2/
    cc/Geometry/
    pack-capture/capture/
    pack-capture-gui/capture-gui/
    ${GTKMM_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${V4L2_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++17 -isystem /usr/include/sigc++-2.0 -isystem /usr/include/gtk-3.0/ \
    -Wall -Wextra -g -m64 -pipe -pthread -lpthread -lopencv_stitching \
    -lopencv_objdetect -lopencv_calib3d -lopencv_features2d \
    -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml \
    -lopencv_imgproc -lopencv_flann -lopencv_core -lboost_thread -lboost_system -lboost_filesystem")

if (CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
endif ()

add_executable(VSSS ${SOURCE_FILES})

target_link_libraries(VSSS ${GTKMM_LIBRARIES})
target_link_libraries(VSSS ${Boost_LIBRARIES})
target_link_libraries(VSSS ${OpenCV_LIBS})
target_link_libraries(VSSS ${V4L2_LIBRARIES})
target_link_libraries(VSSS ${LIBXBEE})
if (CUDA_FOUND)
    target_link_libraries(VSSS ${CUDA_LIBRARIES})
endif ()

