cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 🔍 Найти зависимости
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 🧠 Генерация интерфейсов: сообщения и сервисы
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Time.msg"
  "msg/Sphere.msg"
  "srv/CircleDuration.srv"
  "action/MoveTurtle.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# 🧪 Тестирование и линтеры
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 📦 Завершение
ament_package()
