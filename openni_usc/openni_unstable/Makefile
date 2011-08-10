all: installed

INSTALL_DIR=$(shell rospack find openni_unstable)
PKG_DIR=$(shell rospack find openni_unstable)

patched:
	git clone https://github.com/OpenNI/OpenNI.git openni-unstable
	cd openni-unstable/Platform/Linux-x86/CreateRedist
	./RedistMaker
	cd ../../../..
	tar -xvjf openni-unstable/Platform/Linux-x86/CreateRedist/Final/OpenNI-Bin-Linux64*.tar.bz2
	touch patched
installed: patched

clean:
	rm -rf build bin patched openni_unstable* include lib src
	
