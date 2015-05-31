gcc -c sPcie.c -o sPcie.o -I../sPciDriver
gcc -c sPcieZerocopy.c -o sPcieZerocopy.o -I../sPciDriver
gcc PIO_test.c sPcie.o -o PIO_test -I../sPciDriver
gcc DMA_test.c sPcie.o -o DMA_test -I../sPciDriver
gcc DMA_test_unblocking.c sPcie.o -o DMA_test_unblocking -I../sPciDriver
gcc reset_test.c sPcie.o -o reset_test -I../sPciDriver
gcc INT_test.c sPcie.o -o INT_test -lpthread -I../sPciDriver
gcc INT_test_simple.c sPcie.o -o INT_test_simple -I../sPciDriver
gcc speed_test_memcpy.c sPcie.o  -o speed_test_memcpy -I../sPciDriver
gcc zerocopy_test.c sPcie.o sPcieZerocopy.o -o zerocopy_test -I../sPciDriver
gcc speed_test_double_buf.c sPcie.o sPcieZerocopy.o -o speed_test_double_buf -lpthread -I../sPciDriver
