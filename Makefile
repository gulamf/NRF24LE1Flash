CC = g++
CPPFLAGS =
LDFLAGS =
INCLUDES = -I./include
SRCS = main.cpp src/HexUtils.cpp src/misc.cpp
OBJS = $(SRCS:.c=.o)
PROG = nrf24le1flash

$(PROG) : $(OBJS)
	$(CC) $(LDFLAGS) $(INCLUDES) -o $(PROG) $(OBJS)
.cpp.o:
	$(CC) $(CPPFLAGS) $(INCLUDES) -c $< -o $@
clean:
	$(RM) *.o *~ $(PROG)