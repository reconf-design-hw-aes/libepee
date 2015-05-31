partial_reconf.c is used for programming FPGA via PCIe.
The parameter of this program is the partial reconfiguration bitfile.

PIO_CHN_PRE is used to locate the registers. The base address of the PIO channel can be nonzero.
0x18000 is the value for channel #3 when PIO address is devided into 4 channels.

The bit file of Xilinx has a head and a tail. However, head & tail should not be write into FPGA. So the program will check the head 0xaa995566 and tail 0x0000000d, and then write the "true" bitstream into FPGA.
