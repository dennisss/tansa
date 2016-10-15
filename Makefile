

.PHONY: build

build:
	git submodule update --init --recursive
	mkdir -p build
	cd build; cmake ..; make

run: build
	./build/gcs

clean:
	rm -rf build
