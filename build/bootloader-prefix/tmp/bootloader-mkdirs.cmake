# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.3/components/bootloader/subproject"
  "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader"
  "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader-prefix"
  "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader-prefix/tmp"
  "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader-prefix/src"
  "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/USER/I5_ESP_IDF/Assignment/Assignment1/assignment1/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()