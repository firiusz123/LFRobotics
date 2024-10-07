INCDIR = headers
IMPLDIR = impl
SRCDIR = src
LIBDIR = lib

TARGET = main
LIBRARY = encryptionlib.a

CC = gcc
CFLAGS = -I$(INCDIR) -Wall -g 

OBJECTS = $(IMPLDIR)/encryption $(IMPLDIR)/key_generator

$(LIBDIR)/$(LIBRARY): $(OBJECTS)
	mkdir -p $(LIBDIR)
	ar rcs $(LIBDIR)/$(LIBRARY) $(OBJECTS)
	
$(IMPLDIR)/encryption.o: $(IMPLDIR)/encryption.c $(INCDIR)/encryption.h
	$(CC) $(CFLAGS) -c $(IMPLDIR)/encryption.c -o $(IMPLDIR)/encryption.o

$(IMPLDIR)/key_generator.o: $(IMPLDIR)/key_generator.c $(INCDIR)/key_generator.h
	$(CC) $(CFLAGS) -c $(IMPLDIR)/key_generator.c -o $(IMPLDIR)/key_generator.o

$(TARGET): $(SRCDIR)/main.o $(LIBDIR)/$(LIBRARY)
	$(CC) $(SRCDIR)/main.o -L$(LIBDIR) -lmylib -o $(TARGET)

$(SRCDIR)/main.o: $(SRCDIR)/main.c $(INCDIR)/encryption.h $(INCDIR)/key_generator.h
	$(CC) $(CFLAGS) -c $(SRCDIR)/main.c -o $(SRCDIR)/main.o

clean:
	rm -f $(IMPLDIR)/*.o $(SRCDIR)/*.o $(TARGET) $(LIBDIR)/$(LIBRARY)