################################################################################
# subdir Makefile
# kyChu@2019-3-9
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
./apps/main.c \
./apps/adis.c \
./apps/terminal.c \
./apps/uart.c \
./apps/kysocket.c \
./apps/estimator.c

OBJS += \
$(BuildPath)/apps/main.o \
$(BuildPath)/apps/adis.o \
$(BuildPath)/apps/terminal.o \
$(BuildPath)/apps/uart.o \
$(BuildPath)/apps/kysocket.o \
$(BuildPath)/apps/estimator.o

C_DEPS += \
$(BuildPath)/apps/main.d \
$(BuildPath)/apps/adis.d \
$(BuildPath)/apps/terminal.d \
$(BuildPath)/apps/uart.d \
$(BuildPath)/apps/kysocket.d \
$(BuildPath)/apps/estimator.d

OBJ_DIRS = $(sort $(dir $(OBJS)))

# Each subdirectory must supply rules for building sources it contributes
$(BuildPath)/apps/%.o: ./apps/%.c | $(OBJ_DIRS)
	@echo ' CC $<'
	$(CC) $(PLATFORM) $(DEFS) $(INCS) $(CFGS) -Os $(DBGS) -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
