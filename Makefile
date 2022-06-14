default: parser
CFLAGS=-O0 -gdwarf-3 -Istm32f413-drivers/ -Icn-cbor/include/ -Icn-cbor/src/
CBOR   =  cn-cbor/src/cn-cbor.c cn-cbor/src/cn-create.c
CBOR   += cn-cbor/src/cn-encoder.c  cn-cbor/src/cn-error.c  cn-cbor/src/cn-get.c
PARSER =  parser_test.c

%.o: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) $< -o $@

parser: parser_test.c $(CBOR)

clean:
	rm -f parser
