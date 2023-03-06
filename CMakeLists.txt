


#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

#find_library(SERIAL serial REQUIRED PATHS /home/pizzaboxlegion/orocos_ws/serial/build)
message(status $ENV{PIZZABOX_SERIAL_BUILD})
find_library(SERIAL serial REQUIRED PATHS $ENV{PIZZABOX_SERIAL_BUILD})

find_package(orocos_kdl)



# Creates a component library libpizzabox-<target>.so
# and installs in the directory lib/orocos/pizzabox/
#
orocos_component(microcontroller_driver 
                  src/microcontroller_driver.hpp 
                  src/microcontroller_driver.cpp
                  src/ft_sensor_stewart.hpp 
                  src/ft_sensor_stewart.cpp ) # ...you may add multiple source files

target_link_libraries(microcontroller_driver PUBLIC ${SERIAL} ${orocos_kdl_LIBRARIES})

#set(SERIAL_INCLUDE_DIRS /home/pizzaboxlegion/orocos_ws/serial/include)
message(status $ENV{PIZZABOX_SERIAL_INCLUDE})
include_directories($ENV{PIZZABOX_SERIAL_INCLUDE} ${orocos_kdl_INCLUDE_DIRS})
link_directories($ENV{PIZZABOX_SERIAL_INCLUDE} ${orocos_kdl_LIBRARY_DIRS})

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/pizzabox/ directory
#
# orocos_install_headers(roboclaw_driver.hpp roboclaw.h roboclaw_interface.hpp) # ...you may add multiple header files
orocos_install_headers(src/microcontroller_driver.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libpizzabox-plugin-<target>.so
# and installs in the directory lib/orocos/pizzabox/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
# orocos_plugin(pizzabox-plugin pizzabox-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libpizzabox-service-<target>.so
# and installs in the directory lib/orocos/pizzabox/plugins/
#
# orocos_service(pizzabox-service pizzabox-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libpizzabox-support-<target>.so and installs it in
# lib/
#
# orocos_library(pizzabox-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


