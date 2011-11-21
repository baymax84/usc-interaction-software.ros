# make sure we default to all
all:

# setup cmake
any:
	@mkdir -p build
	cd build && cmake $(CMAKE_FLAGS) ..

remake:
	make clean;

#forward all other commands, calling 'any' first if necessary
%:
	if [ -r build ]; then cd build && make $@; else make any && make $@; fi

PACKAGE_NAME=$(shell basename $(PWD))

clean:
	-if [ -r build ]; then cd build && make clean; fi
	-rm -rf build
	-rm -rf lib
	-rm -rf bin
	-rm -rf mex
