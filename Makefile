#Exec Name
TARGET = balance
#TARGET = $(foo)

TOUCH 	 := $(shell touch *)
CC       = gcc
# compiling flags here
CFLAGS   = -Wall -I. -lrt
#
LINKER   = gcc -o
# # linking flags here
LFLAGS   = -Wall -I. -lm -lrt -pthread
#
# # change these to set the proper directories where each files shoould be
SRCDIR   = src
OBJDIR   = obj
INCDIR   = inc
BINDIR   = bin

SOURCES  := $(wildcard $(SRCDIR)/*.c)
INCLUDES := $(wildcard $(INCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)
rm = rm -f

$(BINDIR)/$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(LFLAGS) $(OBJECTS)
	@echo "Linking complete!"

# Touch to update system time for all compiling files
$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.c
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)
	@echo "Complied "$<" successfully!"

.PHONEY: clean
clean:
	@$(rm) $(OBJECTS)
	@echo "Cleanup complete!"

.PHONEY: remove
remove: clean
	@$(rm) $(BINDIR)/$(TARGET)
	@echo "Executable removed!"
