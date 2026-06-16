all: fbx2

CFLAGS=-Wall -O2 -fomit-frame-pointer -funroll-loops
LIBS=-pthread -lm -lX11 -lXext

fbx2: fbx2.c
	cc $(CFLAGS) fbx2.c $(LIBS) -o fbx2
	strip fbx2

clean:
	rm -f fbx2
