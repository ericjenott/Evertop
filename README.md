# Evertop: an ultra lower power, ultra long battery life solar PC

<div align="center">
  <img width = "23%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/testdrive2-1.JPG?raw=true">
  <img width = "16%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_angle1.JPG?raw=true">
</div>
<div align="center">
  <img width = "12%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_stand1.JPG?raw=true">
  <img width = "14%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_detached2.JPG?raw=true">
  <img width = "20%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/opendevice3.JPG?raw=true">
</div>

## Overview
Evertop is an e-ink portable PC that emulates an IBM XT with an 80186 processor and 1MB RAM. It can run DOS, Minix, and some other old 1980s operating systems.  It also runs Windows up to version 3.0.

### Loaded with built-in peripherals
It features a built in keyboard, external PS/2 keyboard and mouse ports, full CGA, Hercules, and MCGA graphics support, partial EGA and VGA support, PC speaker, Adlib, Covox, and Disney Sound Source audio output, built in speaker and headphone jack with volume adjustment wheel, one DB9 RS232 serial port, one TTL serial port, dual keyboard and mouse PS/2 ports, USB flash drive port for convenient file transfer, an RJ45 ethernet port, wifi, and LoRA radio (I wrote a LoRA realtime chat client in QBasic in about 15 minutes).  Bluetooth hardware is present but I have not yet implemented any features that use it, though I plan to add support for BT keyboard and mouse, and maybe serial over BT, IP over BT, audio over BT, and BT file transfer.  The keyboard can be easily detached on a short tether to make it convenient to prop up the device at an angle while placing the keyboard on a flat surface.  Or you can just plug in you own to the PS/2 port, as sometimes it's nice to be able to use a full sized keyboard, though the one I built into this device is big enough for normal typing.

### Charging options for every scenario
There are three ways you can power and charge this compputer: 1.) built in detachable solar panel, 2.) 2.5 - 20V DC input via internal buck/boost circuitry, or 3.) micro USB connector.  It can charge from all three sources simultaneously, and of course charge in every way while in use.  I also added a built in voltmeter to facilitate easy battery level and charge voltage monitoring.

### Hundreds or thousands of hours on a single charge
In power saving mode it can run between 200 hours on the low side and 500 hours or even much longer on the high side of CONSTANT interactive usage, NOT standby.  That should afford ample opportunity to find some sunny weather and recharge the batteries long before they start getting low.  I'm also going to port a simple native (non-emulated) text editor / word processor and e-reader that I made several years ago for a similar pocket device.  I'm hoping I can get over 1000 hours of constant use on a single charge with those, maybe even 5000-10000 hours on the e-reader app.  This also features a user initiated or automatic hibernate to disk and automatic total power shutoff via an IO controlled dual MOSFET circuit, so you can set it to hibernate and it will completely shut off power when idle, then you can un-hibernate and start back right where yo left off.  Internal peripherals can be shut off when not in use via physical switches on the front panel to ensure no power is being wasted. I've been using a 256G SD card to store floppy and hard disk images, larger cards might work but I haven't tested this yet.  Hard drive images can be up to 4G each.  Emulated systems can mount two floppies and two hard drives for a total of 8G per emulated system.  You can set up multiple emulated systems and choose which one to boot at startup, or just default to the previous choice.

### Software compatibility
Runs almost all IBM PC/XT compatible DOS software from the 1980s and early 90s.


## Sample Videos

### System startup and color inversion
<video src="https://github.com/ericjenott/Evertop/raw/refs/heads/main/videos/startup&inversion.mp4" width=180/>



