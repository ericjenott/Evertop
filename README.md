# Evertop: an ultra lower power, ultra long battery life solar PC

<div align="center">
  <img width = "23%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/testdrive2-1.JPG">
  <img width = "16%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_angle1.JPG">
</div>
<div align="center">
  <img width = "12%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_stand1.JPG">
  <img width = "14%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_detached2.JPG">
  <img width = "20%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/opendevice3.JPG">
</div>

## Overview
Evertop is an portable PC that emulates an IBM XT with an 80186 processor and 1MB RAM. It can run DOS, Minix, and some other old 1980s operating systems.  It also runs Windows up to version 3.0.  Because it runs on a powerful yet very low power microcontroller, uses an e-ink display, carries a sizeable battery, and implements extreme power saving measures, it can run for hundreds or even thousands of hours on a single charge.  Combine that with its built in solar panel, and you should be able to use it indefinitely off-the-grid without ever having to worry about battery life.

### Loaded with built-in peripherals
It features a built in keyboard, external PS/2 keyboard and mouse ports, full CGA, Hercules, and MCGA graphics support, partial EGA and VGA support, PC speaker, Adlib, Covox, and Disney Sound Source audio output, built in speaker and headphone jack with volume adjustment wheel, one DB9 RS232 serial port, one TTL serial port, dual keyboard and mouse PS/2 ports, USB flash drive port for convenient file transfer, an RJ45 ethernet port, wifi, and LoRA radio (I wrote a LoRA realtime chat client in QBasic in about 15 minutes).  Bluetooth hardware is present but I have not yet implemented any features that use it, though I plan to add support for BT keyboard and mouse, and maybe serial over BT, IP over BT, audio over BT, and BT file transfer.  The keyboard can be easily detached on a short tether to make it convenient to prop up the device at an angle while placing the keyboard on a flat surface.  Or you can just plug in you own to the PS/2 port, as sometimes it's nice to be able to use a full sized keyboard, though the one I built into this device is big enough for normal typing.

### Charging options for every scenario
There are three ways you can power and charge this compputer: 1.) built in detachable solar panel, 2.) 2.5 - 20V DC input via internal buck/boost circuitry, or 3.) micro USB connector.  It can charge from all three sources simultaneously, and of course charge in every way while in use.  I also added a built in voltmeter to facilitate easy battery level and charge voltage monitoring.

### Hundreds or thousands of hours on a single charge
In power saving mode it can run between 200 hours on the low side and 500 hours or even much longer on the high side of CONSTANT interactive usage, NOT standby.  That should afford ample opportunity to find some sunny weather and recharge the batteries long before they start getting low.  I'm also going to port a simple native (non-emulated) text editor / word processor and e-reader that I made several years ago for a similar pocket device.  I'm hoping I can get over 1000 hours of constant use on a single charge with those, maybe even 5000-10000 hours on the e-reader app.  This also features a user initiated or automatic hibernate to disk and automatic total power shutoff via an IO controlled dual MOSFET circuit, so you can set it to hibernate and it will completely shut off power when idle, then you can un-hibernate and start back right where yo left off.  Internal peripherals can be shut off when not in use via physical switches on the front panel to ensure no power is being wasted. I've been using a 256G SD card to store floppy and hard disk images, larger cards might work but I haven't tested this yet.  Hard drive images can be up to 4G each.  Emulated systems can mount two floppies and two hard drives for a total of 8G per emulated system.  You can set up multiple emulated systems and choose which one to boot at startup, or just default to the previous choice.

### Technology
Under the hood it's powered by an Espressif ESP32 microcontroller.  I started out using Fabrizio Di Vittorio's amazing and excellent FabGL's PCEmulator demo and having been adding, cutting, changing, fixing, and generally tampering for two years to bring this to its current state.

### Compatibility
Runs almost all IBM PC/XT compatible DOS software from the 1980s and early 90s.


## Sample Videos

### System startup and color inversion
https://github.com/user-attachments/assets/274726d5-35ea-4eb5-89cf-f3aabaa0a709

<br><br>
### King's Quest 1
https://github.com/user-attachments/assets/13fd18fd-db5c-4421-96a0-fe79615b92be

<br><br>
### QEdit editing autoexec.bat
https://github.com/user-attachments/assets/c6fe3bd9-1a23-458f-8ec3-6ddc4d6cbe65

<br><br>
### CP/M with Turbo Pascal
https://github.com/user-attachments/assets/1cd02330-907e-408f-8e0b-55a8b3c6f605

<br><br>
### When and why you need to use the "Color Emulation" feature
https://github.com/user-attachments/assets/255e8f95-449a-4774-9168-92bb78608fd5

<br><br>
### Using a USB flash drive
https://github.com/user-attachments/assets/77839a7c-f4f0-4fe8-b198-c7e36b9bc334

<br><br>
### Planet X3
https://github.com/user-attachments/assets/f1afe4fd-e349-4572-9496-aea447b92e19


<br><br>
### ZZT version of King's Quest
https://github.com/user-attachments/assets/bbe96d6b-c1de-4b15-b2b7-966082788475


<br><br>
### QBASIC "hello world"
https://github.com/user-attachments/assets/962b7809-7b33-4a57-b45f-e7e4db120c3c


<br><br>
### Hibernating and resuming
https://github.com/user-attachments/assets/42f9476a-e2c7-45fd-87c0-54b6ee2dad6f


<br><br>
### Solar panel detaching and re-attaching
https://github.com/user-attachments/assets/7f31b225-3533-4bbe-8254-06b1acdfaacd


<br><br>
### Networking: Ping and FTP
https://github.com/user-attachments/assets/28a4ab93-af6e-4697-aeed-5b1789050d8f


<br><br>
### Doom
https://github.com/user-attachments/assets/dd6682bf-0aea-4d8d-ae58-24b684e38f66


<br><br>
### Networking: Web browsing
https://github.com/user-attachments/assets/ad9f7be2-3504-4a04-8956-48e816ac348d


<br><br>
### Space Quest 3
https://github.com/user-attachments/assets/b22c7ab6-d83d-4709-a84d-256b57cffdf3


<br><br>
### Adlib Jukebox
https://github.com/user-attachments/assets/2d5089bb-cebf-43e6-ae74-68db89a0d8d3


<br><br>
### Scott Adams's Pirate Adventure
https://github.com/user-attachments/assets/ce9783dd-4c6f-424a-856a-04abb533415a


<br><br>
### SimCity
https://github.com/user-attachments/assets/fcb67050-38f1-4a1c-a01d-8c2a09fdbfc0


<br><br>
### Zork
https://github.com/user-attachments/assets/5a589181-e170-4a2e-939b-188634d5aca1


<br><br>
### Wolfenstein 3D
https://github.com/user-attachments/assets/bc20bdcb-f14d-412a-a6b3-60eefe2f2ee0


<br><br>
### Minesweeper on Windows
https://github.com/user-attachments/assets/b7f956ff-74b3-41ea-ae83-52795d74841c


<br><br>
### Minix with Colossal Cave Adventure
https://github.com/user-attachments/assets/3e4fb967-9d5a-4690-983b-c53c2af49256
















