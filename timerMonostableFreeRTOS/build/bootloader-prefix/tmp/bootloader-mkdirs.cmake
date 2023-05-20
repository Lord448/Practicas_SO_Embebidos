# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/lord448/esp/esp-idf/components/bootloader/subproject"
  "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader"
  "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader-prefix"
  "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader-prefix/tmp"
  "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader-prefix/src/bootloader-stamp"
  "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader-prefix/src"
  "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/lord448/Escritorio/STM32Workspaces/Practicas_SO_Embebidos/timerMonostableFreeRTOS/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
