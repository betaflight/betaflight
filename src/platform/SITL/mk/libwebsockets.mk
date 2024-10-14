# Set the build directory
BUILD_DIR = $(ROOT)/obj/main/libwebsockets

# Set CMake flags (you can add more options here)
CMAKE_FLAGS = -DCMAKE_BUILD_TYPE=Release

# Set the source directory (usually the current directory)
SRC_DIR = $(ROOT)/lib/main/libwebsockets

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/Makefile: $(BUILD_DIR)
	@cmake $(CMAKE_FLAGS) -S $(SRC_DIR) -B $(BUILD_DIR) -DLWS_WITH_SSL=OFF -DLWS_WITH_SHARED=OFF -DLWS_WITH_MINIMAL_EXAMPLES=OFF -DLWS_WITHOUT_TESTAPPS=OFF -DLWS_WITHOUT_TEST_SERVER=OFF -DLWS_WITHOUT_TEST_SERVER_EXTPOLL=ON -DLWS_WITHOUT_TEST_PING=OFF -DLWS_WITHOUT_TEST_CLIENT=OFF

$(BUILD_DIR)/lib/libwebsockets.a: $(BUILD_DIR)/Makefile
	@cd $(BUILD_DIR) && $(MAKE)

src/main/drivers/serial_ws.c: $(BUILD_DIR)/lib/libwebsockets.a

clean: websocketsclean

websocketsclean: 
	@cd $(BUILD_DIR) && $(MAKE) clean

.PHONY: websocketsclean

$(TARGET_ELF): $(BUILD_DIR)/lib/libwebsockets.a

LD_FLAGS += -L$(ROOT)/obj/main/libwebsockets/lib -lwebsockets
