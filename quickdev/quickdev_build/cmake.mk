# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)
ROS_MAKE_FLAGS=$(ROS_PARALLEL_JOBS) $(ROS_OPTIMIZATION_LEVEL)
NOBUILD := $(shell if [ -r ROS_NOBUILD ]; then echo 1; else echo 0; fi)

ifeq ($(NOBUILD),1)
all:
%:
	@echo "-- >> ROS_NOBUILD detected; ignoring package << --"
else
all:
	@echo "-- >> Attempting to compile default target [ $@ $(ROS_MAKE_FLAGS) ]..."
	@if [ ! -r build/Makefile ]; then \
		echo "-- >> Build dir not yet initialized..."; \
		make init; \
	fi
	@cd build && make $@ $(ROS_MAKE_FLAGS)
	@echo "-- << Done attempting to compile default target"

init:
	@echo "-- >> Initializing build..."
	@mkdir -p build
	@cd build && cmake $(CMAKE_FLAGS) ..
	@echo "-- << Done initializing build"

remake:
	@make init && make

force_remake:
	@echo "-- >> Rebuilding project..."
	@make clean && make
	@echo "-- << Done rebuilding project"

debclean:
	@-rm -rf build

proprietary:
	@echo "-- >> Removing any proprietary content..."
	@-cd build && make $@ $(ROS_MAKE_FLAGS)
	@echo "-- << Done removing any proprietary content"

debinstall:
	@touch ROS_NOBUILD
	@touch installed

distclean: clean
	@echo "Removing any binary files"
	@-rm -rf lib
	@-rm -rf bin

test:
	@make test-results

install:
	@-cd build && make $@ $(ROS_MAKE_FLAGS)
	@make proprietary
	@make debclean
	@make debinstall

#forward all other commands, calling 'init' first if necessary
%:
	@echo "-- >> Forwarding target [ $@ $(ROS_MAKE_FLAGS) ] to ./build ..."
	@if [ ! -r build ]; then \
		make init; \
	fi
	@cd build && make $@ $(ROS_MAKE_FLAGS)
	@echo "-- << Done forwarding target [ $@ ]"

PACKAGE_NAME=$(shell basename $(PWD))

clean:
	@echo "-- >> Cleaning project..."
	@-if [ ! -r build ]; then make init; fi
	@-cd build && make clean
	@-rm -rf build
	@echo "Removing any auto-generated docs"
	@-rm -rf docs
	@echo "Removing any auto-generated messages"
	@-rm -rf msg/cpp
	@echo "Removing any auto-generated services"
	@-rm -rf srv/cpp
	@echo "Removing any auto-generated dynamic reconfigure files"
	@-rm -rf cfg/cpp
	@-rm -rf cfg/*.cfgc
	@-rm -rf src/$(PACKAGE_NAME)
	@echo "-- << Done cleaning project"

include $(shell rospack find mk)/buildtest.mk
endif
