cmake_minimum_required(VERSION 3.0.2)
project(fourlegs_griphis)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## Choreonoid のパスを設定している前提で find_package
find_package(Choreonoid REQUIRED COMPONENTS SimpleController)

## catkin の include ディレクトリを追加
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

## choreonoid のライブラリリンクディレクトリを追加
link_directories(${CHOREONOID_LIBRARY_DIRS})

catkin_package(
  # 依存関係などがあれば追記
)

## ChoreonoidのSimpleControllerビルド用マクロを使ってコントローラをビルド
choreonoid_add_simple_controller(FixedJointController FixedJointController.cpp)
choreonoid_add_simple_controller(JoystickWalkController JoystickWalkController.cpp)
choreonoid_add_simple_controller(GriphisWalker GriphisWalker.cpp)

## 必要あれば ROSのライブラリもリンク
target_link_libraries(FixedJointController
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES}
)
