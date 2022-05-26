# pmod_nav_pynq


## Not finished yet


This repository implements the driver to use the pmod_nav (link: https://digilent.com/reference/pmod/pmodnav/start) with the Pynq infrastructure.

To make the driver in a simple way, we have chosen to use the MicroBlaze subsystem (https://pynq.readthedocs.io/en/latest/pynq_libraries/pynq_microblaze_subsystem.html) that integrates Pynq.

For this purpose, a c driver has been made in the pmod_nav directory to collect the IMU values. It is incomplete and will be completed. This repository (pmod_nav directory, pmod_nav.bin, pmod_nav.py and Makefile) must be included in ~/pynq/lib/pmod/.

Once it is in that directory it must be compiled by doing a make. 

To test it I have to upload the jupyter project and check that it gets from the sensor.
