test : test.o io.o
        @gcc test.o io.o -o test -lpthread
test.o : test.c io.h
        @gcc -c test.c -o test.o
io.o : io.c io.h pinmap.h
        @gcc -c io.c -o io.o
clean :
        @rm *.o test
install :
        @cp itead.h io.h pinmap.h regmap.h /usr/local/include
        @gcc -shared -fpic -o libiteadIO.so io.c -lpthread
        @sudo mv libiteadIO.so /usr/local/lib