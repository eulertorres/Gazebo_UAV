cmake_minimum_required(VERSION 3.0.2)
project(euler_aerodinamica_v1)

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
  LIBRARIES ${PROJECT_NAME}_aero_plugin ${PROJECT_NAME}_batt_plugin
  CATKIN_DEPENDS roscpp gazebo_ros std_msgs sensor_msgs
  DEPENDS ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}  # Adicione dependências do sistema reais, se houver
)


## Incluir diretórios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  # Adicionado para o plugin da bateria
  #${PROJECT_SOURCE_DIR}/mavlink2
)

## Linkar diretórios
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Definições adicionais
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Criação das bibliotecas a partir dos plugins
add_library(${PROJECT_NAME}_aero_plugin SHARED src/euler_aero_plugin.cpp)
add_library(${PROJECT_NAME}_batt_plugin SHARED src/euler_batt_plugin.cpp)

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

## Mensagens para depuração
message("GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
message("catkin_LIBRARIES: ${catkin_LIBRARIES}")

## Instalação das bibliotecas
install(TARGETS ${PROJECT_NAME}_aero_plugin ${PROJECT_NAME}_batt_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Instalação dos arquivos de cabeçalho
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
