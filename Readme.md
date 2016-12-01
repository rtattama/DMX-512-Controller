# DMX-512-Controller
Build device capable of acting as either a controller or device for a timing intensive asynchronous communications interface based on the DMX512-A protocol with EF1 topology. The PC transmitter will accept commands from a PC via an RS-232 interface and will continuously  transmit a serial stream to control up to 512 devices on a RS-485 communication bus. The PC receiver will forward data received from devices on a communications bus and send these to the PC with the RS-232 interface. Devices on the bus will extract information out of the asynchronous data stream and will control one or more devices. They will also send an acknowledgement to the controller when requested.
