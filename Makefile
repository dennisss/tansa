

.PHONY: build

build:
	git submodule update --init
	mkdir -p build
	cd build; cmake ..; make

run: build
	./build/bridge

clean:
	rm -rf build
