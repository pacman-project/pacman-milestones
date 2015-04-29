
BUILD_DIR = cmake.build
INSTALL_PREFIX = /usr/local

.PHONY: all build install clean xcode

all: build

build: $(BUILD_DIR)/Makefile
	$(MAKE) -C $(BUILD_DIR)

# Separate Makefile creation from default command, to avoid
# overwriting the CMake install prefix with the default one
# defined above.
$(BUILD_DIR)/Makefile:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_PREFIX) ..

install: build
	$(MAKE) -C $(BUILD_DIR) install

clean:
	rm -fr $(BUILD_DIR)
	rm -fr build

xcode: clean
	mkdir $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. -G Xcode -DCMAKE_CONFIGURATION_TYPES=Release -DCMAKE_INSTALL_PREFIX=$(INSTALL_PREFIX) -DCMAKE_BUILD_TYPE=Release
	open cmake.build/part-based-planner.xcodeproj

