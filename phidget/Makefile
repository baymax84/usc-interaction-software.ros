all: installed


INSTALL_DIR=$(shell rospack find phidget)
PKG_DIR=$(shell rospack find phidget)

patched:
	wget http://www.phidgets.com/downloads/libraries/Phidgetlinux_2.1.6.20090717.tar.gz
	tar -xvzf *.tar.gz
	cd Phidgetlinux/phidget21 && make && cd ../..
	mkdir -p $(INSTALL_DIR)/include
	cp Phidgetlinux/phidget21/phidget21.h $(INSTALL_DIR)/include
	mkdir -p $(INSTALL_DIR)/lib
	cp Phidgetlinux/phidget21/libphidget21.so $(INSTALL_DIR)/lib
	touch patched

installed: patched
	if [ ! -d $(PKG_DIR)/build ]; then mkdir -p $(PKG_DIR)/build; fi
	cd $(PKG_DIR)/build && cmake ..
	cd $(PKG_DIR)/build && make $(ROS_PARALLEL_JOBS) 

clean:
	rm -rf build bin patched Phidget* phidget include lib src msg/cpp

