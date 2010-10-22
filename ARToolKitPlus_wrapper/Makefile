all: artkp

TARBALL = build/artkp.zip
TARBALL_URL = http://studierstube.icg.tu-graz.ac.at/handheld_ar/download/ARToolKitPlus_2.1.1.zip
SOURCE_DIR = build/artkp_src
UNPACK_CMD = mkdir -p artkp_src/artkp;cp artkp.zip artkp_src/artkp;cd artkp_src/artkp && unzip *.zip;rm artkp.zip;cd ../..
TARBALL_PATCH = mods.patch
include $(shell rospack find mk)/download_unpack_build.mk

artkp: $(SOURCE_DIR)/unpacked
	mkdir -p src
	touch artkp
clean:
	-rm -rf src $(SOURCE_DIR) artkp
wipe: clean
	-rm -rf build
