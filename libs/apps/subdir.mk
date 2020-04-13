################################################################################
# subdir Makefile
# kyChu@2019-3-9
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
./apps/main.c \
./apps/terminal.c \
./apps/kysocket.c

OBJS += \
$(BuildPath)/apps/main.o \
$(BuildPath)/apps/terminal.o \
$(BuildPath)/apps/kysocket.o

C_DEPS += \
$(BuildPath)/apps/main.d \
$(BuildPath)/apps/terminal.d \
$(BuildPath)/apps/kysocket.d

OBJ_DIRS = $(sort $(dir $(OBJS)))

# Each subdirectory must supply rules for building sources it contributes
$(BuildPath)/apps/%.o: ./apps/%.c | $(OBJ_DIRS)
	@echo ' CC $<'
	$(CC) $(PLATFORM) $(DEFS) $(INCS) $(CFGS) -Os $(DBGS) -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
