# Evertop: an ultra lower power, ultra long battery life solar PC

<div align="center">
  <img width = "34.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/testdrive2-1.JPG">
  <img width = "24%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_angle1.JPG">
</div>
<div align="center">
  <img width = "19.2%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_stand1.JPG">
  <img width = "21%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_detached2.JPG">
  <img width = "30%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/opendevice3.JPG">
</div>

<br>



## Overview
Evertop is an portable PC that emulates an IBM XT with an 80186 processor and 1MB RAM. It can run DOS, Minix, and some other old 1980s operating systems.  It also runs Windows up to version 3.0.  Because it runs on a powerful yet very low power microcontroller, uses an e-ink display, carries a sizeable battery, and implements extreme power saving measures, it can run for hundreds or even thousands of hours on a single charge.  Combine that with its built in solar panel, and you should be able to use it indefinitely off grid without ever having to worry about battery life.

<br>

### Loaded with built-in peripherals
It features a built in keyboard, external PS/2 keyboard and mouse ports, full CGA, Hercules, and MCGA graphics support, partial EGA and VGA support, PC speaker, Adlib, Covox, and Disney Sound Source audio output, built in speaker and headphone jack with volume control wheel, one DB9 RS232 serial port, one TTL serial port, dual keyboard and mouse PS/2 ports, USB flash drive port for convenient file transfer, an RJ45 ethernet port, wifi, and LoRA radio (I wrote a LoRA realtime chat client in QBasic in about 15 minutes).  Bluetooth hardware is present but I have not yet implemented any features that use it, though I plan to add support for BT keyboard and mouse, and maybe serial over BT, IP over BT, audio over BT, and BT file transfer.  The keyboard can be easily detached on a short tether to make it convenient to prop up the device at an angle while placing the keyboard on a flat surface.  Or you can just plug in you own to the PS/2 port, as sometimes it's nice to be able to use a full sized keyboard, though the one I built into this device is big enough for normal typing.

<br>

### Charging options for every scenario
There are three ways you can power and charge this compputer: 1.) built in detachable solar panel, 2.) 2.5 - 20V DC input via internal buck/boost circuitry, or 3.) micro USB connector.  It can charge from all three sources simultaneously, and of course charge in every way while in use.  I also added a built in voltmeter to facilitate easy battery level and charge voltage monitoring.

<br>

### Hundreds to thousands of hours on a single charge
In power saving mode it can run between 200 hours on the low side and 500 hours or even much longer on the high side of CONSTANT interactive usage, NOT standby.  That should afford ample opportunity to find some sunny weather and recharge the batteries long before they start getting low.  I'm also going to port a simple native (non-emulated) text editor / word processor and e-reader that I made several years ago for a similar pocket device.  I'm hoping I can get over 1000 hours of constant use on a single charge with those, maybe even 5000-10000 hours on the e-reader app.  This also features a user initiated or automatic hibernate to disk and automatic total power shutoff via an IO controlled dual MOSFET circuit, so you can set it to hibernate and it will completely shut off power when idle, then you can un-hibernate and start back right where yo left off.  Internal peripherals can be shut off when not in use via physical switches on the front panel to ensure no power is being wasted. I've been using a 256G SD card to store floppy and hard disk images, larger cards might work but I haven't tested this yet.  Hard drive images can be up to 4G each.  Emulated systems can mount two floppies and two hard drives for a total of 8G per emulated system.  You can set up multiple emulated systems and choose which one to boot at startup, or just default to the previous choice.

<br>

### Technology
Under the hood it's powered by an Espressif ESP32 microcontroller.  I started out using Fabrizio Di Vittorio's amazing and excellent FabGL's PCEmulator demo and have been adding, cutting, changing, fixing, and generally tampering for two years to get to this state.

<br>

### Compatibility
Runs almost all IBM PC/XT compatible DOS software from the 1980s and early 90s.

<br>

## New minimal version: "Evertop Min" 
For this version, I removed the built-in keyboard, variable voltage charging, solar panel, RJ45 ethernet, DB9 serial port, LoRA radio and half the battery capacity to reduce weight, material, and costs of parts and assembly.  It has the same e-ink display, dual keyboard/mouse PS/2 ports, built in speaker, headphone jack, volume control wheel, USB flash drive port, wifi networking, bluetooth, TTL serial port, and sd card slot.  Only charges via micro USB port.  Add your own PS/2 keyboard and an external solar panel and you're still good to go for basic long lasting off grid computing without the weight, bulk, and expensive of all those extra bells and whistles.

<div align="center">
  <img width = "19%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/smallkbd2.JPG">
  <img width = "24%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/connected_devices4.JPG">
  <img width = "26.4%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/excel.JPG">
</div>
<div align="center">
  <img width = "38.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/rightside2.JPG">
  <img width = "39.9%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/leftside2.JPG">
  <img width = "19.7%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/with_max2.jpg">
</div>

<br><br>

## Sample Videos
Note: if some videos won't play in Firefox, please try using Chrome.<br>
<b>Or</b> watch the hi-res full length versions on my Youtube playlist <a href="https://youtube.com/playlist?list=PLS5HRMZsVMLZf-PbHvJDVLgTP-tTvth55&si=df2rlJ7pIH9pm1Kg">here</a>.

<br><br>
### QBASIC "hello world"
https://github.com/user-attachments/assets/7936a16b-e707-4298-a39a-2964b3c653aa

<br><br>
### Space Quest 3 with Adlib sound and music
https://github.com/user-attachments/assets/2b20a7d1-0362-4848-8348-c91b98728131

<br><br>
### Minesweeper on Windows
https://github.com/user-attachments/assets/540fa992-51f2-4d51-8aa3-3e9dba048924

<br><br>
### Adlib Jukebox
https://github.com/user-attachments/assets/f23e2479-c99c-4e64-bbf9-22223a75d402

<br><br>
### Using a USB flash drive
https://github.com/user-attachments/assets/77839a7c-f4f0-4fe8-b198-c7e36b9bc334

<br><br>
### Networking: Ping and FTP
https://github.com/user-attachments/assets/5f67eb18-9b9b-4c4f-b5dd-cf022fadc6d1

<br><br>
### Networking: Web browsing
https://github.com/user-attachments/assets/8a99601b-c7f9-49f3-8a99-dfec2aed2d76

<br><br>
### Hibernating and resuming
https://github.com/user-attachments/assets/c9758c15-9f8d-4058-a61f-c34481873b1e

<br><br>
### Solar panel detaching and re-attaching



<br><br>
### System startup and color inversion
https://github.com/user-attachments/assets/8807fc27-6829-4491-978f-2921a8d432fe

<br><br>
### King's Quest 1
https://github.com/user-attachments/assets/46d44474-fa79-49ea-9949-d85c53864d76

<br><br>
### QEdit editing autoexec.bat
https://github.com/user-attachments/assets/c6fe3bd9-1a23-458f-8ec3-6ddc4d6cbe65

<br><br>
### Wolfenstein 3D
https://github.com/user-attachments/assets/e295904c-3d49-4982-8d65-b21779e48726

<br><br>
### Doom
https://github.com/user-attachments/assets/fc9ec4e4-5f15-47cf-980f-c31522a4abcc

<br><br>
### CP/M-86 with Turbo Pascal
https://github.com/user-attachments/assets/1cd02330-907e-408f-8e0b-55a8b3c6f605

<br><br>
### When and why to use "Color Emulation"
https://github.com/user-attachments/assets/255e8f95-449a-4774-9168-92bb78608fd5

<br><br>
### Planet X3


<br><br>
### ZZT version of King's Quest
https://github.com/user-attachments/assets/27d068f2-91de-4887-bba5-bcdd4b63fbf2



<br><br>
### Scott Adams's Pirate Adventure
https://github.com/user-attachments/assets/f46fa540-cb63-4454-81f6-fe7e84447212

<br><br>
### SimCity
https://github.com/user-attachments/assets/a8b7a0c0-d77a-4ccb-a051-0ceed19d8957



<br><br>
### Zork


<br><br>
### Minix with Colossal Cave Adventure
https://github.com/user-attachments/assets/3e4fb967-9d5a-4690-983b-c53c2af49256
















