CXX = gcc
SRCS = adjacency.c, dStar.c, dStarRequirement.c, DynamicQueue.c, main.c, mapRun.c
OBJS=$(subst .cc,.o,$(SRCS))

all: bothoven

bothoven: $(OBJS)
	$(CXX) -o wormhole $(OBJS) 

.PHONY: clean

clean:
	rm $(OBJS)
	rm bothoven