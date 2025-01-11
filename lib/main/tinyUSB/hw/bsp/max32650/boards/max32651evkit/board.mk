# Use the secure linker file
LD_FILE = $(BOARD_PATH)/max32651.ld

# Let the family script know the build needs to be signed
SIGNED_BUILD := 1
