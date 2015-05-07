This code was used to  test communication between two RFM22 modules. WinAVR (AVRDude) was used to compile and write the code.  Makefiles are set up for serial programming - check out this tutorial for more info: http://www.sparkfun.com/commerce/tutorial_info.php?tutorials_id=142.

In my testing, I used two 3.3V Arduino Pro's (ATmega328), one programmed with the Receive code and one with the Transmit. Data is transmitted in 17-byte packets (16 data, 1 checksum). All of the configuration fuctions were derived from the examples posted on HopeRF's product page:
Transmit Demo: http://www.hoperf.com/upfile/RFM22_transmit_DEMO.pdf
Receive Demo: http://www.hoperf.com/upfile/RFM22_receive_DEMO.pdf

In testing, with a 17cm wire attached to the ANT pin, I got the modules to transmit about 150ft away from eachother, down a hallway. YMMV. I'd love to hear if you got it further!

There are a number of ways to configure and use these modules, and this example is by no means the best or only way to do so. I encourage you to try fiddling with the settings to tune the RFM22 for best operation with your project.

All code is released under the Creative Commons Share-alike v3.0 license. Feel free to use it commercially or otherwise. If you have any adjustments to make to the code, I'd love to hear about them! Drop me an email (jim@sparkfun.com).

Cheers!
-Jim