SDCC=sdcc
SDLD=sdld
OBJECTS=main.ihx

.PHONY: all clean

all: $(OBJECTS)

clean:
	rm -f $(OBJECTS)

%.ihx: %.c
	$(SDCC) -lstm8 -mstm8 --out-fmt-ihx $(CFLAGS) $(LDFLAGS) $<
