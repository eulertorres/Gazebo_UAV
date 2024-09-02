cmake_minimum_required(VERSION 3.0.2)
project(euler_helices_v2)

## Encontre os pacotes necessários
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
)

find_package(gazebo REQUIRED)
#find_package(ignition-math4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_wind_plugin
  CATKIN_DEPENDS roscpp gazebo_ros
  DEPENDS system_lib
)

## Incluir diretórios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  #${IGNITION-MATH_INCLUDE_DIRS}
)

## Linkar diretórios
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Definição do pacote catkin
catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros std_msgs
)

set(target_linking_LIBRARIES ${catkin_LIBRARIES} ${gazebo_dev_LIBRARIES})

## Criação da biblioteca a partir do plugin
add_library(${PROJECT_NAME} SHARED src/PropellerPlugin.cpp)

# fluid resitance plugin
add_library(fluid_resitance_plugin src/fluid_resitance.cpp)
target_link_libraries(fluid_resitance_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plugin vento
add_library(rotors_gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)
target_link_libraries(rotors_gazebo_wind_plugin ${target_linking_LIBRARIES} )

add_dependencies(rotors_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})
#list(APPEND targets_to_install rotors_gazebo_wind_plugin)

## Linkagem das bibliotecas
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

## Mensagens para depuração
message("GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
message("catkin_LIBRARIES: ${catkin_LIBRARIES}")
