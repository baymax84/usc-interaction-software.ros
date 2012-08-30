# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# make sure we default to all
all:
	rosmake --no-rosdep

remake:
	make clean && make

clean_stack:
	@rm -rf build

#forward all other commands, calling 'any' first if necessary
%:
	@echo "-- >> Building target [ $@ ] for all packages in stack [ $(STACK_NAME) ]..."
	@for package in $$(rosstack contents $(STACK_NAME)); do \
		echo "-- >> Building target [ $@ ] for package [ $$package ]"; \
		cd $$(rospack find $$package) && make $@; \
		echo "-- << Done building target [ $@ ] for package [ $$package ]"; \
	done

	@if [ "$@" = "clean" ]; then make clean_stack; fi

	@echo "-- << Done building target $@"

STACK_NAME=$$( basename $(PWD))

package_source: all
	$$(rospack find rosbuild)/bin/package_source.py $(CURDIR)
