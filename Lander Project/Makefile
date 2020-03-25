CC = g++
BINDIR = ./bin/
SRCDIR = ./src/

PROG = lander
OBJS = $(BINDIR)lander.o $(BINDIR)lander_graphics.o $(BINDIR)orbiter_class.o $(BINDIR)dynamics.o
CPPFLAGS = -lGL -lGLU -lglut

$(PROG): $(OBJS)
	mv *.o $(BINDIR)
	$(CC) -o $(PROG) $(OBJS) $(CPPFLAGS) 
	
	
$(BINDIR)lander.o: $(SRCDIR)lander.cpp $(SRCDIR)lander.h
	$(CC) $(CPPFLAGS) -c $(SRCDIR)lander.cpp

$(BINDIR)lander_graphics.o: $(SRCDIR)lander_graphics.cpp $(SRCDIR)lander.h
	$(CC) $(CCSW) $(CPPFLAGS) -c $(SRCDIR)lander_graphics.cpp

$(BINDIR)orbiter_class.o: $(SRCDIR)orbiter_class.cpp $(SRCDIR)orbiter_class.h
	$(CC) $(CCSW) $(CPPFLAGS) -c $(SRCDIR)orbiter_class.cpp

$(BINDIR)dynamics.o: $(SRCDIR)dynamics.cpp $(SRCDIR)dynamics.h
	$(CC) $(CCSW) $(CPPFLAGS) -c $(SRCDIR)dynamics.cpp
	
clean:
	rm $(BINDIR)*.o
	rm $(PROG)