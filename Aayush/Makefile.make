CXX = cc
SRCS = main.c
OBJS = $(SRCS:.c=.o)
all: bothoven

bothoven: $(OBJS)
	$(CXX) $(OBJS) -o bothoven

$(OBJS): $(SRCS)
	$(CXX) -c *.o

.PHONY: clean

clean:
	rm *.o
	rm bothoven