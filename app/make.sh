clang -c sPcie.c -o sPcie.o -I../sPciDriver
clang -c sPcieZerocopy.c -o sPcieZerocopy.o -I../sPciDriver
clang PIO_test.c sPcie.o -o PIO_test -I../sPciDriver
clang DMA_test.c sPcie.o -o DMA_test -I../sPciDriver
clang DMA_test_unblocking.c sPcie.o -o DMA_test_unblocking -I../sPciDriver
clang reset_test.c sPcie.o -o reset_test -I../sPciDriver
clang INT_test.c sPcie.o -o INT_test -lpthread -I../sPciDriver
clang INT_test_simple.c sPcie.o -o INT_test_simple -I../sPciDriver
clang speed_test_memcpy.c sPcie.o  -o speed_test_memcpy -I../sPciDriver
clang zerocopy_test.c sPcie.o sPcieZerocopy.o -o zerocopy_test -I../sPciDriver
clang speed_test_double_buf.c sPcie.o sPcieZerocopy.o -o speed_test_double_buf -lpthread -I../sPciDriver
