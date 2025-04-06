.PHONY: all compile run clean

all: compile run clean

compile:
	g++ src/raytracer.cpp -o src/raytracer.exe -std=c++17 -pthread

run:
	./src/raytracer.exe

clean:
	del /Q src\raytracer.exe 2>nul || exit 0
