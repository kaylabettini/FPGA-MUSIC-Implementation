Hi and welcome to my undergraduate thesis project titled: "FPGA Implementation of the Hardware-Friendly MUSIC Algorithm for Microphone Array Sound Source Localisation"

In this project I have 6 folders:

FPGA - This contains the HFMA block that was generated through using Simulink.

MATLAB - This contains almost all of the files that I used for simulations, testing, and benchmarking.

PI - This contains the python scripts that were run on the Raspberry Pi 5.

thesis - This is the entire Vivado project and it includes all .VHD files and external IP blocks used. This also includes the copy of the Ethernet MAC and hence a LICENSE file has been included next to it. 

The Ethernet MAC in FPGA/ethernet_mac/ is by C. Preuninger (Nexys-4-DDR-Ethernet-Mac), used under GPL-3.0. All other code is by the author unless noted. I'd like to thank C. Preuninger for this as it helped tremendously after I fried the SPI pins on the Raspberry Pi 
