# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.

COMMON=-O2 -I ./inc -L . -std=c++11 -Wl,-rpath,'$$ORIGIN'

all:
	mkdir -p bin
	$(CXX) $(COMMON) ant.cc -lmujoco -o bin/ant
