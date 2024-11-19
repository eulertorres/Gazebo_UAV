cmake_minimum_required(VERSION 3.0.2)
project(euler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Encontre os pacotes necessários
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  sensor_msgs  # Adicionado para o plugin da bateria
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)  # Adicionado para o plugin da bateria

## Definição do pacote catkin
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_aero_plugin ${PROJECT_NAME}_batt_plugin ${PROJECT_NAME}_mot_plugin ${PROJECT_NAME}_mot_plugin_v2
  CATKIN_DEPENDS roscpp gazebo_ros std_msgs sensor_msgs
  DEPENDS ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} system_lib  # Adicione dependências do sistema reais, se houver
)


## Incluir diretórios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  # Adicionado para o plugin da bateria
  {PROJECT_SOURCE_DIR}
)

## Linkar diretórios
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

link_libraries(
        ${GAZEBO_LIBRARIES}
        )

## Definições adicionais
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Criação das bibliotecas a partir dos plugins
add_library(${PROJECT_NAME}_aero_plugin SHARED src/euler_aero_plugin.cpp)
add_library(${PROJECT_NAME}_batt_plugin SHARED src/euler_batt_plugin.cpp)
add_library(${PROJECT_NAME}_mot_plugin SHARED src/euler_mot_plugin.cpp)
add_library(${PROJECT_NAME}_mot_plugin_v2 SHARED src/euler_mot_plugin_v2.cpp)
add_library(${PROJECT_NAME}_ardupilot SHARED src/euler_ardupilot.cpp)

## Linkagem das bibliotecas
target_link_libraries(${PROJECT_NAME}_aero_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_batt_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}  # Necessário se o plugin da bateria utilizar Boost
)

target_link_libraries(${PROJECT_NAME}_mot_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES} 
)

target_link_libraries(${PROJECT_NAME}_mot_plugin_v2
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES} 
)

target_link_libraries(${PROJECT_NAME}_ardupilot
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES} 
)

install(TARGETS euler_ardupilot DESTINATION ${GAZEBO_PLUGIN_PATH})

set(CMAKE_BUILD_TYPE Debug)

## Mensagens para depuração
message("GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
message("catkin_LIBRARIES: ${catkin_LIBRARIES}")

## Instalação das bibliotecas
install(TARGETS ${PROJECT_NAME}_aero_plugin ${PROJECT_NAME}_batt_plugin ${PROJECT_NAME}_mot_plugin ${PROJECT_NAME}_mot_plugin_v2 ${PROJECT_NAME}_ardupilot
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Instalação dos arquivos de cabeçalho
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
