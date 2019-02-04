# vim: set noet:

argb-ctrl: argb-spi-ctrl.c
	$(CC) $(CFLAGS) -o argb-ctrl argb-spi-ctrl.c

test: argb-ctrl
	./argb-ctrl -g 255 -r 0 -b 255

.PHONY: clean
clean:
	rm -f argb-ctrl


