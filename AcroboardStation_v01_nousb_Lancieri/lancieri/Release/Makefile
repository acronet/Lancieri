################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../services \
../services/console \
../services/datalogger \
../services/IDF \
../services/config \
../services/libemqtt \
../services/fw_update \
../services/taskman


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../services/config/config.c \
../services/datalogger/datalogger.c \
../services/fw_update/eeprom_manager.c \
../services/fw_update/fw_update.c \
../services/fw_update/hex_processor.c \
../services/IDF/IDF.c \
../services/libemqtt/libemqtt.c \
../services/taskman/taskman.c \
../lancieri.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
services/config/config.o \
services/datalogger/datalogger.o \
services/fw_update/eeprom_manager.o \
services/fw_update/fw_update.o \
services/fw_update/hex_processor.o \
services/IDF/IDF.o \
services/libemqtt/libemqtt.o \
services/taskman/taskman.o \
lancieri.o

OBJS_AS_ARGS +=  \
services/config/config.o \
services/datalogger/datalogger.o \
services/fw_update/eeprom_manager.o \
services/fw_update/fw_update.o \
services/fw_update/hex_processor.o \
services/IDF/IDF.o \
services/libemqtt/libemqtt.o \
services/taskman/taskman.o \
lancieri.o

C_DEPS +=  \
services/config/config.d \
services/datalogger/datalogger.d \
services/fw_update/eeprom_manager.d \
services/fw_update/fw_update.d \
services/fw_update/hex_processor.d \
services/IDF/IDF.d \
services/libemqtt/libemqtt.d \
services/taskman/taskman.d \
lancieri.d

C_DEPS_AS_ARGS +=  \
services/config/config.d \
services/datalogger/datalogger.d \
services/fw_update/eeprom_manager.d \
services/fw_update/fw_update.d \
services/fw_update/hex_processor.d \
services/IDF/IDF.d \
services/libemqtt/libemqtt.d \
services/taskman/taskman.d \
lancieri.d

OUTPUT_FILE_PATH +=lancieri.elf

OUTPUT_FILE_PATH_AS_ARGS +=lancieri.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+=  \
C:/Documents\ and\ Settings/marco/My\ Documents/Atmel\ Studio/AcroboardStation_v01_nousb_Lancieri/AcroboardStation_v01/Release/libAcroboardStation_v01.a

# AVR32/GNU C Compiler



















services/config/%.o: ../services/config/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DNDEBUG -D_SIM900_USART_POLLED -DNF_CPU=8000000  -I"../../AcroboardStation_v01" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/common/applications/user_application/user_board/config" -I"../../AcroboardStation_v01/src/ASF/xmega/utils" -I"../../AcroboardStation_v01/src/config" -I"../../AcroboardStation_v01/src/ASF/common/boards" -I"../../AcroboardStation_v01/src/ASF/xmega/utils/preprocessor" -I"../../AcroboardStation_v01/src/ASF/common/utils" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/src/ASF/common/boards/user_board" -I"../../AcroboardStation_v01/src/ASF/common/services/calendar" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/aes" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/cpu" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/pmic" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/rtc32" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/sleep" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/tc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/wdt" -I"../../AcroboardStation_v01/src/ASF/common/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/common/services/clock" -I"../../AcroboardStation_v01/src/ASF/common/services/delay" -I"../../AcroboardStation_v01/src/ASF/common/services/gpio" -I"../../AcroboardStation_v01/src/ASF/common/services/ioport" -I"../../AcroboardStation_v01/src/ASF/common/services/serial/xmega_usart" -I"../../AcroboardStation_v01/src/ASF/common/services/serial" -I"../../AcroboardStation_v01/src/ASF/common/services/sleepmgr" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/adc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/usart" -I"../../AcroboardStation_v01/src/ASF/common/services/twi" -I"../../AcroboardStation_v01/src/Config" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/twi" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/crc" -I".."  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -Wextra -mmcu=atxmega256a3bu -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

services/datalogger/%.o: ../services/datalogger/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DNDEBUG -D_SIM900_USART_POLLED -DNF_CPU=8000000  -I"../../AcroboardStation_v01" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/common/applications/user_application/user_board/config" -I"../../AcroboardStation_v01/src/ASF/xmega/utils" -I"../../AcroboardStation_v01/src/config" -I"../../AcroboardStation_v01/src/ASF/common/boards" -I"../../AcroboardStation_v01/src/ASF/xmega/utils/preprocessor" -I"../../AcroboardStation_v01/src/ASF/common/utils" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/src/ASF/common/boards/user_board" -I"../../AcroboardStation_v01/src/ASF/common/services/calendar" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/aes" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/cpu" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/pmic" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/rtc32" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/sleep" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/tc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/wdt" -I"../../AcroboardStation_v01/src/ASF/common/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/common/services/clock" -I"../../AcroboardStation_v01/src/ASF/common/services/delay" -I"../../AcroboardStation_v01/src/ASF/common/services/gpio" -I"../../AcroboardStation_v01/src/ASF/common/services/ioport" -I"../../AcroboardStation_v01/src/ASF/common/services/serial/xmega_usart" -I"../../AcroboardStation_v01/src/ASF/common/services/serial" -I"../../AcroboardStation_v01/src/ASF/common/services/sleepmgr" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/adc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/usart" -I"../../AcroboardStation_v01/src/ASF/common/services/twi" -I"../../AcroboardStation_v01/src/Config" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/twi" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/crc" -I".."  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -Wextra -mmcu=atxmega256a3bu -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

services/fw_update/%.o: ../services/fw_update/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DNDEBUG -D_SIM900_USART_POLLED -DNF_CPU=8000000  -I"../../AcroboardStation_v01" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/common/applications/user_application/user_board/config" -I"../../AcroboardStation_v01/src/ASF/xmega/utils" -I"../../AcroboardStation_v01/src/config" -I"../../AcroboardStation_v01/src/ASF/common/boards" -I"../../AcroboardStation_v01/src/ASF/xmega/utils/preprocessor" -I"../../AcroboardStation_v01/src/ASF/common/utils" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/src/ASF/common/boards/user_board" -I"../../AcroboardStation_v01/src/ASF/common/services/calendar" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/aes" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/cpu" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/pmic" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/rtc32" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/sleep" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/tc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/wdt" -I"../../AcroboardStation_v01/src/ASF/common/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/common/services/clock" -I"../../AcroboardStation_v01/src/ASF/common/services/delay" -I"../../AcroboardStation_v01/src/ASF/common/services/gpio" -I"../../AcroboardStation_v01/src/ASF/common/services/ioport" -I"../../AcroboardStation_v01/src/ASF/common/services/serial/xmega_usart" -I"../../AcroboardStation_v01/src/ASF/common/services/serial" -I"../../AcroboardStation_v01/src/ASF/common/services/sleepmgr" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/adc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/usart" -I"../../AcroboardStation_v01/src/ASF/common/services/twi" -I"../../AcroboardStation_v01/src/Config" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/twi" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/crc" -I".."  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -Wextra -mmcu=atxmega256a3bu -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

services/IDF/%.o: ../services/IDF/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DNDEBUG -D_SIM900_USART_POLLED -DNF_CPU=8000000  -I"../../AcroboardStation_v01" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/common/applications/user_application/user_board/config" -I"../../AcroboardStation_v01/src/ASF/xmega/utils" -I"../../AcroboardStation_v01/src/config" -I"../../AcroboardStation_v01/src/ASF/common/boards" -I"../../AcroboardStation_v01/src/ASF/xmega/utils/preprocessor" -I"../../AcroboardStation_v01/src/ASF/common/utils" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/src/ASF/common/boards/user_board" -I"../../AcroboardStation_v01/src/ASF/common/services/calendar" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/aes" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/cpu" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/pmic" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/rtc32" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/sleep" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/tc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/wdt" -I"../../AcroboardStation_v01/src/ASF/common/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/common/services/clock" -I"../../AcroboardStation_v01/src/ASF/common/services/delay" -I"../../AcroboardStation_v01/src/ASF/common/services/gpio" -I"../../AcroboardStation_v01/src/ASF/common/services/ioport" -I"../../AcroboardStation_v01/src/ASF/common/services/serial/xmega_usart" -I"../../AcroboardStation_v01/src/ASF/common/services/serial" -I"../../AcroboardStation_v01/src/ASF/common/services/sleepmgr" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/adc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/usart" -I"../../AcroboardStation_v01/src/ASF/common/services/twi" -I"../../AcroboardStation_v01/src/Config" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/twi" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/crc" -I".."  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -Wextra -mmcu=atxmega256a3bu -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

services/libemqtt/%.o: ../services/libemqtt/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DNDEBUG -D_SIM900_USART_POLLED -DNF_CPU=8000000  -I"../../AcroboardStation_v01" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/common/applications/user_application/user_board/config" -I"../../AcroboardStation_v01/src/ASF/xmega/utils" -I"../../AcroboardStation_v01/src/config" -I"../../AcroboardStation_v01/src/ASF/common/boards" -I"../../AcroboardStation_v01/src/ASF/xmega/utils/preprocessor" -I"../../AcroboardStation_v01/src/ASF/common/utils" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/src/ASF/common/boards/user_board" -I"../../AcroboardStation_v01/src/ASF/common/services/calendar" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/aes" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/cpu" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/pmic" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/rtc32" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/sleep" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/tc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/wdt" -I"../../AcroboardStation_v01/src/ASF/common/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/common/services/clock" -I"../../AcroboardStation_v01/src/ASF/common/services/delay" -I"../../AcroboardStation_v01/src/ASF/common/services/gpio" -I"../../AcroboardStation_v01/src/ASF/common/services/ioport" -I"../../AcroboardStation_v01/src/ASF/common/services/serial/xmega_usart" -I"../../AcroboardStation_v01/src/ASF/common/services/serial" -I"../../AcroboardStation_v01/src/ASF/common/services/sleepmgr" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/adc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/usart" -I"../../AcroboardStation_v01/src/ASF/common/services/twi" -I"../../AcroboardStation_v01/src/Config" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/twi" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/crc" -I".."  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -Wextra -mmcu=atxmega256a3bu -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

services/taskman/%.o: ../services/taskman/%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DNDEBUG -D_SIM900_USART_POLLED -DNF_CPU=8000000  -I"../../AcroboardStation_v01" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/common/applications/user_application/user_board/config" -I"../../AcroboardStation_v01/src/ASF/xmega/utils" -I"../../AcroboardStation_v01/src/config" -I"../../AcroboardStation_v01/src/ASF/common/boards" -I"../../AcroboardStation_v01/src/ASF/xmega/utils/preprocessor" -I"../../AcroboardStation_v01/src/ASF/common/utils" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/src/ASF/common/boards/user_board" -I"../../AcroboardStation_v01/src/ASF/common/services/calendar" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/aes" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/cpu" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/pmic" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/rtc32" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/sleep" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/tc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/wdt" -I"../../AcroboardStation_v01/src/ASF/common/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/common/services/clock" -I"../../AcroboardStation_v01/src/ASF/common/services/delay" -I"../../AcroboardStation_v01/src/ASF/common/services/gpio" -I"../../AcroboardStation_v01/src/ASF/common/services/ioport" -I"../../AcroboardStation_v01/src/ASF/common/services/serial/xmega_usart" -I"../../AcroboardStation_v01/src/ASF/common/services/serial" -I"../../AcroboardStation_v01/src/ASF/common/services/sleepmgr" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/adc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/usart" -I"../../AcroboardStation_v01/src/ASF/common/services/twi" -I"../../AcroboardStation_v01/src/Config" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/twi" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/crc" -I".."  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -Wextra -mmcu=atxmega256a3bu -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -DNDEBUG -D_SIM900_USART_POLLED -DNF_CPU=8000000  -I"../../AcroboardStation_v01" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/common/applications/user_application/user_board/config" -I"../../AcroboardStation_v01/src/ASF/xmega/utils" -I"../../AcroboardStation_v01/src/config" -I"../../AcroboardStation_v01/src/ASF/common/boards" -I"../../AcroboardStation_v01/src/ASF/xmega/utils/preprocessor" -I"../../AcroboardStation_v01/src/ASF/common/utils" -I"../../AcroboardStation_v01/src" -I"../../AcroboardStation_v01/src/ASF/common/boards/user_board" -I"../../AcroboardStation_v01/src/ASF/common/services/calendar" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/aes" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/cpu" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/pmic" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/rtc32" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/sleep" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/tc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/wdt" -I"../../AcroboardStation_v01/src/ASF/common/drivers/nvm" -I"../../AcroboardStation_v01/src/ASF/common/services/clock" -I"../../AcroboardStation_v01/src/ASF/common/services/delay" -I"../../AcroboardStation_v01/src/ASF/common/services/gpio" -I"../../AcroboardStation_v01/src/ASF/common/services/ioport" -I"../../AcroboardStation_v01/src/ASF/common/services/serial/xmega_usart" -I"../../AcroboardStation_v01/src/ASF/common/services/serial" -I"../../AcroboardStation_v01/src/ASF/common/services/sleepmgr" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/adc" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/usart" -I"../../AcroboardStation_v01/src/ASF/common/services/twi" -I"../../AcroboardStation_v01/src/Config" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/twi" -I"../../AcroboardStation_v01/src/ASF/xmega/drivers/crc" -I".."  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -Wall -Wextra -mmcu=atxmega256a3bu -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.8.1
	$(QUOTE)C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="lancieri.map" -Wl,--start-group -Wl,-lm -Wl,-lAcroboardStation_v01  -Wl,--end-group -Wl,-L"../../AcroboardStation_v01/Release" -Wl,-L"C:\DOCUME~1\marco\MYDOCU~1\ATMELS~1\ACROBO~4\ACROBO~1\Release"  -Wl,--gc-sections -mrelax -Wl,-section-start=.dl_boot=0x80000  -mmcu=atxmega256a3bu  
	@echo Finished building target: $@
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "lancieri.elf" "lancieri.hex"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "lancieri.elf" "lancieri.eep" || exit 0
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "lancieri.elf" > "lancieri.lss"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "lancieri.elf" "lancieri.srec"
	"C:\Program Files\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.1056\avr8-gnu-toolchain\bin\avr-size.exe" "lancieri.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "lancieri.elf" "lancieri.a" "lancieri.hex" "lancieri.lss" "lancieri.eep" "lancieri.map" "lancieri.srec" "lancieri.usersignatures"
	