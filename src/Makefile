default : all

#CXXFLAGS = -Wall -O2 -Dcimg_verbosity=3
#CXXFLAGS = -Wall -O3
CXXFLAGS = -Wall
LDFLAGS = -L/usr/lib/ -lpthread -lX11

BDM.o : BDM.cpp BDM.h
	g++ ${CXXFLAGS} -c BDM.cpp
	
Block.o : Block.cpp Block.h
	g++ ${CXXFLAGS} -c Block.cpp
	
BlockMatching.o : BlockMatching.cpp BlockMatching.h
	g++ ${CXXFLAGS} -c BlockMatching.cpp

Edges.o : Edges.cpp Edges.h
	g++ ${CXXFLAGS} -c Edges.cpp
	
File.o : File.cpp File.h
	g++ ${CXXFLAGS} -c File.cpp

Math.o : Math.cpp Math.h
	g++ ${CXXFLAGS} -c Math.cpp

Node.o : Node.cpp Node.h
	g++ ${CXXFLAGS} -c Node.cpp

Point.o : Point.cpp Point.h
	g++ ${CXXFLAGS} -c Point.cpp
	
SearchWindow.o : SearchWindow.cpp SearchWindow.h
	g++ ${CXXFLAGS} -c SearchWindow.cpp

Thresholding.o : Thresholding.cpp Thresholding.h
	g++ ${CXXFLAGS} -c Thresholding.cpp
	
Tuple.o : Tuple.cpp Tuple.h
	g++ ${CXXFLAGS} -c Tuple.cpp
	
main.o : main.cpp
	g++ ${CXXFLAGS} -c main.cpp

me : main.o BDM.o Block.o BlockMatching.o Edges.o File.o Math.o Node.o Point.o SearchWindow.o Thresholding.o Tuple.o
	g++ ${CXXFLAGS} main.o BDM.o Block.o BlockMatching.o Edges.o File.o Math.o Node.o Point.o SearchWindow.o Thresholding.o Tuple.o ${LDFLAGS} -o me
	
all : g++ *.cpp -o me `pkg-config --cflags --libs opencv`



