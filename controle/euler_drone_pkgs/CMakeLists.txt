cmake_minimum_required(VERSION 3.0.2)
project(euler_aerodinamica_v1)

## Encontre os pacotes necessários
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
)

find_package(gazebo REQUIRED)

## Definição do pacote catkin
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} # Referência à biblioteca do projeto
  CATKIN_DEPENDS roscpp gazebo_ros std_msgs
  DEPENDS system_lib
)

## Incluir diretórios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Linkar diretórios
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Criação da biblioteca a partir do plugin
add_library(${PROJECT_NAME} SHARED src/euler_aero_plugin.cpp)

## Definir outras bibliotecas, se necessário
# add_library(fluid_resitance_plugin src/another_plugin.cpp)

## Linkagem das bibliotecas
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Linkagem de outras bibliotecas, se necessário
# target_link_libraries(fluid_resitance_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Mensagens para depuração
message("GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
message("catkin_LIBRARIES: ${catkin_LIBRARIES}")
