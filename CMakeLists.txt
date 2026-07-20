cmake_minimum_required(VERSION 3.16)

# Add local esp-zigbee-sdk to component search path
set(EXTRA_COMPONENT_DIRS /mnt/d/InnovatsiiEMSSensorsCode/esp-workspace/esp-zigbee-sdk/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(COMPONENTS main)

project(sensor-hub-zigbee)