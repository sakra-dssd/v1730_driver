ALL: caenvmebabies v1730_init

CFLAGS = -Wall -O2 -DLINUX
LOADLIBES = -lm -lpthread -lCAENVME -lCAENDigitizer

libbabies.o : libbabies.h
libbbcaenvme.o : libbbcaenvme.h

MODULES = v792.o v1290.o v1730.o

caenvmebabies : caenvmebabies.o libbabies.o libbbcaenvme.o bbpid.o $(MODULES)

v1730_init : v1730_init.o v1730.o

clean :
	rm -f *.o
