CXX = cc
SRCS = adjacency.c, dStar.c, dStarRequirement.c, DynamicQueue.c, main.c, mapRun.c
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