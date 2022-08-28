# micromeasure
Measurement microscope for UW CFL

The MicroMeasure project uses a repurposed DynaScope inspection microscope for examining microscopic creatures in water samples. The scope UI facilitates measurements of size, shape and area. The DynaScope has an XY stage with more than 150 mm of travel in each direction along with 1 um measurement resolution. According to its original calibration certificate it maintained an absolute position accuracy of +/- 3 um within that envelope. The optics of the Dynascope have been replaced with a USB microscope to provide a digital video feed. The position of the stage is tracked by a pair of linear optical encoders. The quadrature signals from the encoders are fed through a Veloc-IO Ace programmable logic controller (velocio.net) to convert the quadrature signals to absolute digital position.

Measurement accuracy of the scope does not depend on optical accuracy as the stage is used to position measurement points under the center of the field of view. Therefore, the point of measurement is always at the same spot in the FOV and optical distortions are unimportant. 

The GUI portion of the software utilizes Qt6 and is written in python. The Veloc-IO is programmed in ladder logic using software from the supplier.
