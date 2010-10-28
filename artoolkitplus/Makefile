all: installed

PKG_DIR=$(shell rospack find artoolkitplus)
TARBALL = build/artkp.zip
TARBALL_URL = http://studierstube.icg.tu-graz.ac.at/handheld_ar/download/ARToolKitPlus_2.1.1.zip
SOURCE_DIR = build/artkp_src
#Annoyingly it's a zip, perhaps we should re-host a tarball...
UNPACK_CMD = mkdir -p artkp_src/artkp;cp artkp.zip artkp_src/artkp;cd artkp_src/artkp && unzip *.zip;rm artkp.zip;cd ../..
TARBALL_PATCH = mods.patch
include $(shell rospack find mk)/download_unpack_build.mk

artkp: $(SOURCE_DIR)/unpacked
	cp artkp.cmake $(SOURCE_DIR)/artkp/CMakeLists.txt
	cd $(SOURCE_DIR)/artkp && cmake . && make
	cp -r $(SOURCE_DIR)/artkp/id-markers .
	mkdir -p $(PKG_DIR)/lib
	cp -r $(SOURCE_DIR)/artkp/lib/* lib
	mkdir -p $(PKG_DIR)/include/ARToolKitPlus
	cp -r $(SOURCE_DIR)/artkp/include/* include
	mkdir -p $(PKG_DIR)/src
	cp -r $(SOURCE_DIR)/artkp/src/* src
	cp -r $(SOURCE_DIR)/artkp/sample/multi/data data
	touch artkp
installed: artkp
	cd $(PKG_DIR)/build && cmake ..
	cd $(PKG_DIR)/build && make $(ROS_PARALLEL_JOBS)

clean:
	rm -rf data include id-markers lib src $(SOURCE_DIR) artkp bin/*

wipe: clean
	-rm -rf build
