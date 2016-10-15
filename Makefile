

.PHONY: build

build:
	git submodule update --init --recursive
	mkdir -p build
	rm -f config/gazebo/models/x340/x340.sdf
	cd build; cmake ..; make

run: build
	./build/gcs

clean:
	rm -rf build
	rm -rf build_firmware
	rm -rf tmp



# Build the included firmware for the purpose of simulation
build_firmware:
	git submodule update --init --recursive
	mkdir -p build_firmware
	cd build_firmware; cmake ../lib/Firmware -DCONFIG="posix_sitl_default"; make; make sitl_gazebo

# Starts an empty sim
sim: build_firmware build
	./scripts/start_gazebo.sh
