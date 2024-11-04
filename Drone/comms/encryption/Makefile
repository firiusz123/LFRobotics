# Makefile

# Define directories
INCDIR      = include
IMPLDIR     = src
EXAMPLEDIR  = examples
LIBDIR      = lib

# Define output library names
STATIC_LIB  = libencryptionlib.a
SHARED_LIB  = libencryptionlib.so

# Compiler and flags
CC          = gcc
CFLAGS      = -I$(INCDIR) -Wall -g -fPIC
LDFLAGS     = -L$(LIBDIR)

# List of object files
OBJECTS     = $(IMPLDIR)/encryption.o $(IMPLDIR)/key_generator.o $(IMPLDIR)/key_exchange.o $(IMPLDIR)/transmission.o

# Targets
.PHONY: all static shared example example_shared clean

all: static shared example example_shared

# Rule to build the static library
static: $(OBJECTS)
	@mkdir -p $(LIBDIR)
	ar rcs $(LIBDIR)/$(STATIC_LIB) $(OBJECTS)
	@echo "Static library created at $(LIBDIR)/$(STATIC_LIB)\n"

# Rule to build the shared library
shared: $(OBJECTS)
	@mkdir -p $(LIBDIR)
	$(CC) -shared -o $(LIBDIR)/$(SHARED_LIB) $(OBJECTS)
	@echo "Shared library created at $(LIBDIR)/$(SHARED_LIB)\n"

# Pattern rule to compile .c files to .o files
$(IMPLDIR)/%.o: $(IMPLDIR)/%.c $(INCDIR)/%.h
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to build the example program linked against the static library
example: $(EXAMPLEDIR)/main.o static
	$(CC) $(EXAMPLEDIR)/main.o -L$(LIBDIR) -lencryptionlib -o $(EXAMPLEDIR)/example
	@echo "Example program (static) created at $(EXAMPLEDIR)/example\n"

# Rule to build the example program linked against the shared library
example_shared: $(EXAMPLEDIR)/main.o shared
	$(CC) $(EXAMPLEDIR)/main.o -L$(LIBDIR) -lencryptionlib -Wl,-rpath=$(LIBDIR) -o $(EXAMPLEDIR)/example_shared
	@echo "Example program (shared) created at $(EXAMPLEDIR)/example_shared\n"

# Compile the example program object file
$(EXAMPLEDIR)/main.o: $(EXAMPLEDIR)/main.c $(INCDIR)/encryption.h $(INCDIR)/key_generator.h $(INCDIR)/key_exchange.h $(INCDIR)/transmission.h
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up all build artifacts
clean:
	rm -f $(IMPLDIR)/*.o $(EXAMPLEDIR)/*.o $(EXAMPLEDIR)/example $(EXAMPLEDIR)/example_shared
	rm -rf $(LIBDIR)
	@echo "Cleaned up build artifacts."
