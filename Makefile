CXX = g++
CXXFLAGS = -std=c++11
UIFLAGS = -lsfml-graphics -lsfml-window -lsfml-system -lsfml-audio

run: FORCE
	$(CXX) main.cpp $(CXXFLAGS) -o $@

FORCE: ;


