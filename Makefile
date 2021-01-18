CC	    	= gcc
CXX     	= g++
CFLAGS	    = -g  -Wall -pthread
CFLAGS	       += -I$(RTMA)/include -I$(ROBOTINC) -D_USE_UNIX -DUSE_LINUX  -DTESTING -DUSERTMA
CXXFLAGS        = $(CFLAGS)
LDFLAGS         = -L$(RTMA)/lib -lRTMA
LDLIBS      = -lRTMA

#install: NetboxModule
#	ln -fs `pwd`/NetboxModule $(ROBOTBIN)/NetboxModule
#	$(ROBOTINC)/hlink

#compile: NetboxModule

NetboxModule: NetboxModule.o
	$(CXX) $(LDFLAGS) $(CXXFLAGS) -lm  $^ -o $@ $(LDLIBS)

clean:
	$(RM) *.o map *~ NetboxModule
