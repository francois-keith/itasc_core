# This Makefile is here for 'rosmake' like systems. In case you don't use
# ROS, it will create a build directory for you and build the package with
# default settings. It will install it at the same location as the RTT is installed.
ifdef ROS_ROOT
EXTRA_CMAKE_FLAGS=-DCMAKE_INSTALL_PREFIX=`rospack find itasc_core`/install
default: install_itasc_core
include $(shell rospack find mk)/cmake.mk
install_itasc_core: all
	cd build; ${MAKE} install
else
$(warning This Makefile builds this package with default settings)
all:
	mkdir -p build
	cd build ; cmake .. -DINSTALL_PATH=orocos && make
	echo -e "\n Now do 'make install' to install this package.\n"
install: all
	cd build ; make install
endif