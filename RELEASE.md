ADVimba Releases
==================

The latest untagged master branch and tagged releases can be obtained at
https://github.com/areaDetector/ADVimba.

Tagged prebuilt binaries can be obtained at
https://cars.uchicago.edu/software/pub/ADVimba.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============
R1-3 (October 2, 2020)
----
* Updated the version of the Vimba SDK being used to 4.0 (Windows) and 4.1 (Linux).
  Windows users will need to install Vimba 4.0 to get the correct drivers and DLLs.
  Linux users do not need to do anything because the 4.1 files are all included in ADVimba.
* Added automatic packet size negotiation for GigE cameras in the constructor.
  Previous cameras would default to jumbo packets, and if the network did not support that
  then streaming would fail until the packet size feature was decreased.
* The transport layer statistics are now updated each time an image is received, independent
  of whether the ReadStatus record is processed. Changed the OPI display to make that clear.
* Minor change to allow the driver to work on EPICS 3.14.12.

R1-2 (April 9, 2020)
----
* Fix problem with the packet and frame statistics records. 
  They were not updating because the DTYP needed to be changed from asynInt32 to asynInt64.
* Add .bob files for Phoebus Display Manager

R1-1 (January 5, 2020)
----
* Change VimbaFeature support for GenICam features from int (32-bit) to epicsInt64 (64-bit)
* Fixed Doyxgen comment errors in the driver.

R1-0 (October-20-2019)
----
* Initial release
