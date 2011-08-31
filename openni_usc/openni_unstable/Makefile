all: installed

INSTALL_DIR=$(shell rospack find openni_unstable)
PKG_DIR=$(shell rospack find openni_unstable)

patched:
	git clone https://github.com/OpenNI/OpenNI.git openni-unstable
	touch patched

installed: patched
	cd openni-unstable/Platform/Linux-x86/CreateRedist && ./RedistMaker
	tar -xvjf openni-unstable/Platform/Linux-x86/CreateRedist/Final/OpenNI-Bin-Linux64*.tar.bz2
	touch installed

clean:
	rm -rf build bin patched openni-unstable Include Documentation Jar insatlled Samples mainpage.dox install.sh Lib Bin GPL.txt LGPL.txt
	
