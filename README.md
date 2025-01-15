# Evertop: an ultra lower power, ultra long battery life solar PC

<div align="center">
  <img width = "23%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/testdrive2-1.JPG">
  <img width = "16%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_angle1.JPG">
</div>
<div align="center">
  <img width = "12.8%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_stand1.JPG">
  <img width = "14%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_detached2.JPG">
  <img width = "20%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/opendevice3.JPG">
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
[qbasic.webm](https://github.com/user-attachments/assets/225efe0d-2b90-4db3-8a54-ab3ef893bf0d)


<br><br>
### Space Quest 3 with Adlib sound and music
[sq3.webm](https://github.com/user-attachments/assets/c5009e01-66ea-4530-b80a-ca6bba2cad84)


<br><br>
### Minesweeper on Windows
https://github.com/user-attachments/assets/b7f956ff-74b3-41ea-ae83-52795d74841c

<br><br>
### Adlib Jukebox
[adlib_jukebox.webm](https://github.com/user-attachments/assets/77afcefa-28e7-49af-b235-64be55e5e011)


<br><br>
### Using a USB flash drive
https://github.com/user-attachments/assets/77839a7c-f4f0-4fe8-b198-c7e36b9bc334

<br><br>
### Networking: Ping and FTP
[ping&ftp.webm](https://github.com/user-attachments/assets/1dece6fd-4c53-47e5-9447-b6fbbb594350)


<br><br>
### Networking: Web browsing
[ping&ftp.webm](https://github.com/user-attachments/assets/5953b3ac-76ed-48e4-ad6b-3391908de6b6)


<br><br>
### Hibernating and resuming
[hibernate&restore.webm](https://github.com/user-attachments/assets/0994e4d7-eb75-49b8-af09-cb980382228f)


<br><br>
### Solar panel detaching and re-attaching
[solar_panel_setup.webm](https://github.com/user-attachments/assets/64aee97a-6224-4b30-9b58-15e1a4472caa)


<br><br>
### System startup and color inversion
https://github.com/user-attachments/assets/274726d5-35ea-4eb5-89cf-f3aabaa0a709

<br><br>
### King's Quest 1
https://github.com/user-attachments/assets/13fd18fd-db5c-4421-96a0-fe79615b92be

<br><br>
### QEdit editing autoexec.bat
https://github.com/user-attachments/assets/c6fe3bd9-1a23-458f-8ec3-6ddc4d6cbe65

<br><br>
### Wolfenstein 3D
https://github.com/user-attachments/assets/bc20bdcb-f14d-412a-a6b3-60eefe2f2ee0

<br><br>
### Doom
[doom.webm](https://github.com/user-attachments/assets/2f68af70-148d-4c9d-953a-2bd9d20c162e)


<br><br>
### CP/M-86 with Turbo Pascal
https://github.com/user-attachments/assets/1cd02330-907e-408f-8e0b-55a8b3c6f605

<br><br>
### When and why to use "Color Emulation"
https://github.com/user-attachments/assets/255e8f95-449a-4774-9168-92bb78608fd5

<br><br>
### Planet X3
https://github.com/user-attachments/assets/5619d1c5-ceab-4a71-88d1-a452a3150254

<br><br>
### ZZT version of King's Quest
https://github.com/user-attachments/assets/bbe96d6b-c1de-4b15-b2b7-966082788475

<br><br>
### Scott Adams's Pirate Adventure
[pirate_adventure.webm](https://github.com/user-attachments/assets/65f91223-6548-49b4-8285-87ed1f7a8a00)


<br><br>
### SimCity
https://github.com/user-attachments/assets/fcb67050-38f1-4a1c-a01d-8c2a09fdbfc0

<br><br>
### Zork
https://github.com/user-attachments/assets/5a589181-e170-4a2e-939b-188634d5aca1

<br><br>
### Minix with Colossal Cave Adventure
https://github.com/user-attachments/assets/3e4fb967-9d5a-4690-983b-c53c2af49256
















