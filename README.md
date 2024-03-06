<h1>Antenna Tracker</h1>

Realtime GPS based antenna tracking ground station for your drone.

Raspberry Pi 4B is used for HDMI capture of Walksnail OSD and incoming Mavlink message reading via SiK radio to obtain realtime drone coordinates.
GPS coordinates are read from each OSD frame with OpenCV KNN algortihm.
It is possible to use either only Mavlink message reading, only OSD frame capturing and analyzing and both at the same time.

A Raspbbery Pi Pico is connected to the main computer (Raspberry Pi 4B or any other) and recieves realtime servo positions to track the drone with an optional GPS module for ground station coordinates

![image](https://github.com/EPoikans/AntennaTracker/assets/132155653/03a62f11-2f62-403d-aa54-7e81cd06a676)
