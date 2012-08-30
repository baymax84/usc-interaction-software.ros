# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)
ROS_MAKE_FLAGS=$(ROS_PARALLEL_JOBS) $(ROS_OPTIMIZATION_LEVEL)

all:
	@echo "-- >> Attempting to compile default target [ $@ $(ROS_MAKE_FLAGS) ]..."
	@if [ ! -r build ]; then \
		echo "-- >> Build dir not yet initialized..."; \
		make init && make $@ $(ROS_MAKE_FLAGS); \
	elif ! ( cd build && make $@ $(ROS_MAKE_FLAGS) ); then \
		echo "-- >> Build dir broken; trying to recover... << --"; \
		if ! ( make init && make $@ $(ROS_MAKE_FLAGS) ); then \
			make force_remake; \
		fi; \
	fi
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
	-rm -rf build

distclean: clean
	@echo "Removing any binary files"
	@-rm -rf build
	@-rm -rf lib
	@-rm -rf bin

test: all
	@if [ ! -r build ]; then \
		make init; \
	fi
	@make test-results

#forward all other commands, calling 'init' first if necessary
%:
	@echo "-- >> Forwarding target [ $@ $(ROS_MAKE_FLAGS) ] to ./build ..."
	@if [ ! -r build ]; then \
		make init; \
	fi
	@cd build; \
	if ( ! make $@ $(ROS_MAKE_FLAGS) ) && [ "$@" = "install" ]; then \
		echo "Warning: explicit install target not found; ignoring"; \
	fi
	@echo "-- << Done forwarding target [ $@ ]"

PACKAGE_NAME=$(shell basename $(PWD))

clean:
	@echo "-- >> Cleaning project..."
	@-if [ -r build ]; then cd build && make clean; fi
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
