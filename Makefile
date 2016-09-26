all: fbx2

CFLAGS=-Ofast -fomit-frame-pointer \
 -I/opt/vc/include \
 -I/opt/vc/include/interface/vcos/pthreads \
 -I/opt/vc/include/interface/vmcs_host \
 -I/opt/vc/include/interface/vmcs_host/linux \
 -L/opt/vc/lib
LIBS=-pthread -lrt -lm -lbcm_host

fbx2: fbx2.c
	cc $(CFLAGS) fbx2.c $(LIBS) -o fbx2
	strip fbx2

clean:
	rm -f fbx2
