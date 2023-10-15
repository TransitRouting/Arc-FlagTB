CC=g++ -fopenmp
FLAGS=-std=c++17 -pipe
OPTIMIZATION=-march=native -O3
DEBUG=-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer
RELEASE=-ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG


all: TPRelease ArcTBRelease NetworkRelease ULTRARelease 
# all: TPDebug ArcTBDebug NetworkDebug ULTRADebug

NetworkDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o Network Network.cpp

NetworkRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o Network Network.cpp

ULTRADebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o ULTRA ULTRA.cpp

ULTRARelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o ULTRA ULTRA.cpp

ArcTBDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o ARCTB ArcTB.cpp

ArcTBRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o ARCTB ArcTB.cpp

TPDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o TP TP.cpp

TPRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o TP TP.cpp
