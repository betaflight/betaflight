CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
export CC
export AR

all:
		$(CC) -g -o stmloader -I./ \
				loader.c \
				serial.c \
				stmbootloader.c \
				-Wall

clean:
		rm -f stmloader; rm -rf stmloader.dSYM
