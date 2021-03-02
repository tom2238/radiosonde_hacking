# QtCreator STM32 with libopencm3 setup (without debugging now)

## Stlink utility
* Read/Erase/Write/Program or Debug tools for stm32 MCUs
* Download binaries [pkgs.org](https://pkgs.org/search/?q=stlink) or compile from source [Github](https://github.com/stlink-org/stlink)

## GCC ARM none eabi compiler
* Download from [developer.arm.com](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* Extract into $HOME or /usr directory
* Add compiler manualy or autodetect if compiler is present in /usr/bin/arm-none-eabi*
![QtCreator compiler](qt-creator-compiler.png?raw=true "QtCreator compiler")

## QtCreator STM32 kit
* Create new kit for Bare Metal device
* Unset Qt version and assign correct C/C++ compiler
![QtCreator STM32 kit](qt-creator-stm32-kit.png?raw=true "QtCreator STM32 kit")

## Import template into QtCreator
* File > New file or project
* Import project > Import existing project
* Select template location
![QtCreator project location](qt-creator-import-project.png?raw=true "QtCreator project location")

* Import template .c and .h files (without libopencm3 sources)
![QtCreator project files](qt-creator-import-files.png?raw=true "QtCreator project files")

* Finish import
* Add ../../libopencm3/include into <project name>.include, QtCreator can find libopencm3 libraries now

## Set project for STM32 kit
* Remove Desktop target and add STM32 kit build options
![QtCreator build](qt-creator-project-build.png?raw=true "QtCreator build")

* Add STM32 kit run options
![QtCreator run](qt-creator-project-run.png?raw=true "QtCreator run")
