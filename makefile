all: map

map: main.cpp Graph.cpp Graph.h
	g++ -o map main.cpp Graph.cpp Graph.h -std=c++11 -lm -lpthread -lX11

