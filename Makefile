RM := rm -rf

OBJS += \
./cli.o \
./final_project_pc.o \
./options.o \
./parse_command.o \
./udp.o 

CC := gcc

all: main-build

# Main-build Target
main-build: cnc

# Tool invocations
cnc: $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: C Linker'
	$(CC) -o "cnc" $(OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

%.o: ./%.c
	@echo 'Building file: $<'
	@echo 'Invoking: C Compiler'
	$(CC) -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


# Other Targets
clean:
	-$(RM) *.o *.d cnc
	-@echo ' '

.PHONY: all clean main-build
