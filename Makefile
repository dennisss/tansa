

.PHONY: build

build:
	git submodule update --init
	mkdir -p build
	cd build; cmake ..; make

run: build
	./build/gcs

clean:
	rm -rf build
	rm -rf build_firmware
	rm -rf tmp



# Build the included firmware for the purpose of simulation
build_firmware:
	git submodule update --init
	mkdir -p build_firmware
	cd build_firmware; cmake ../lib/Firmware -DCONFIG="posix_sitl_default"; make; make sitl_gazebo

# Starts an empty sim
sim: build_firmware build
	./scripts/start_sim.sh
