# Evertop: an ultra lower power, ultra long battery solar PC

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
Evertop is a portable PC that emulates an IBM XT with an 80186 processor and 1MB RAM. It can run DOS, Minix, and some other old 1980s operating systems.  It also runs Windows up to version 3.0.  Because it's based on a powerful yet very low power microcontroller, uses an e-ink display, packs two 10,000mAh batteries, and implements extreme power saving measures, it can run for hundreds or even thousands of hours on a single charge.  Combine that with its built in solar panel, and you should be able to use it indefinitely off grid without ever having to worry about battery life.
<br>

### Loaded with built-in peripherals
It features a built in keyboard, external PS/2 keyboard and mouse ports, full CGA, Hercules, and MCGA graphics support, partial EGA and VGA support, PC speaker, Adlib, Covox, and Disney Sound Source audio output, built in speaker and headphone jack with volume control wheel, one DB9 RS232 serial port, one TTL serial port, dual keyboard and mouse PS/2 ports, USB flash drive port for convenient file transfer, an RJ45 ethernet port, wifi, and LoRA radio (I wrote a LoRA realtime chat client in QBASIC in about 15 minutes).  Bluetooth hardware is present but I have not yet implemented it in any features, though I plan to add support for BT keyboard and mouse, and maybe serial over BT, IP over BT, audio over BT, and BT file transfer.  The keyboard can be easily detached on a short tether to make it convenient to prop up the device at an angle while placing the keyboard on a flat surface.  Or you can just plug in your own to the PS/2 port, as sometimes it's nice to be able to use a full sized keyboard, though the one I built into this device is big enough for normal typing.
<br>

### Charging options for every scenario
There are three ways you can power and charge this compputer: 1.) built in detachable solar panel, 2.) 2.5 - 20V DC input via an internal buck/boost circuit, or 3.) micro USB connector.  It can charge from all three sources simultaneously, and of course charge in every way while in use.  I also added a built in voltmeter to facilitate easy battery level and charge voltage monitoring.
<br>

### Hundreds to thousands of hours on a single charge
In power saving mode it can run between 200 hours on the low side and 500 hours or in some cases even much longer of constant interactive use, <b><i>not</i></b> standby.  That should afford ample opportunity to find some sunny weather and recharge the batteries long before they start getting low.  I'm also going to port over a simple native (non-emulated) text editor / word processor and e-reader that I made several years ago for a similar pocket device.  I'm hoping I can get over 1000 hours of constant use on a single charge with those, maybe even 5000-10000 hours on the e-reader app.  Evertop also features optional user initiated or automatic hibernate to disk and automatic total power shutoff via an IO controlled dual MOSFET circuit, so you can set it to hibernate and it will completely shut off power when idle, then when you powre back on you can resume right where yo left off.  Internal peripherals can be shut off when not in use via physical switches on the front panel to ensure no power is being wasted.  The solar panel is rated at 6V and 6W, so theoretically it should be able to produce 1 amp with full sunlight.  The most I've ever seen it do is 700mA, but even that's enough to provide ten to fifty hours of constant use from each hour of sunlight.  And even on the dimmest cloudy days it still gets around 70-100mA, which at least gives can give one to five hours of use per hour of charge.
<br>

### Storage
I've been using a 256G SD card to store floppy and hard disk images, larger cards might work but I haven't tested one yet.  Hard drive images can be up to 4G each.  Emulated systems can mount two floppies and two hard drives for a total of 8G per emulated system.  You can set up multiple emulated systems and choose which one to boot at startup, or just default to the previous choice.
<br>

### Technology
Under the hood it's powered by an Espressif ESP32 microcontroller.  The display is a 5.83 inch 648x480 (yes, 648 with the 8) "fast refresh" model from good-display.com which uses no power when not refreshing.  I started out using Fabrizio Di Vittorio's amazing and excellent PCEmulator demo for his FabGL library and have been adding, cutting, changing, fixing, and generally tampering for two years to turn this into what it is today.  Enclosure is 3d printed matte PETG plastic.
<br>

### Compatibility
Runs almost all IBM PC/XT compatible DOS software from the 1980s and early 90s.
<br>

## New minimal version: "Evertop Min" 
For this version, I removed the built-in keyboard, variable voltage charging, solar panel, RJ45 ethernet, DB9 serial port, LoRA radio, volt meter and half the battery capacity to reduce weight, material, and costs of parts and assembly.  It still has the same e-ink display, dual keyboard/mouse PS/2 ports, built in speaker, headphone jack, volume control wheel, USB flash drive port, wifi networking, bluetooth, TTL serial port, sd card slot, and micro USB charging port.  And it runs the same firmware as its larger sibling.  Add your own PS/2 keyboard and an external solar panel and you're still good to go for basic long lasting off grid computing without the weight, bulk, and expense of all those wonderful bells and whistles.

<div align="center">
  <img width = "26.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/smallkbd2.JPG">
  <img width = "34%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/connected_devices4.JPG">
  <img width = "37%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/excel.JPG">
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
<div><img width=60 src="https://github.com/user-attachments/assets/5fa22022-bf06-4c9d-8cf1-ed0c71fa55c8" /></div>

https://github.com/user-attachments/assets/7936a16b-e707-4298-a39a-2964b3c653aa



<br><br>
### Space Quest 3 with Adlib sound and music
<div><img width=60 src="https://github.com/user-attachments/assets/82e9c201-96ba-4a23-9576-b5ada00116f7" /></div>

https://github.com/user-attachments/assets/2b20a7d1-0362-4848-8348-c91b98728131



<br><br>
### Minesweeper on Windows
<div><img width=60 src="https://github.com/user-attachments/assets/f1fc2659-ac04-4c34-980c-88f63df48484" /></div>

https://github.com/user-attachments/assets/540fa992-51f2-4d51-8aa3-3e9dba048924



<br><br>
### Adlib Jukebox
<div><img width=60 src="https://github.com/user-attachments/assets/c8a92e9c-eab9-4acf-978d-760e890d9fa2" /></div>

https://github.com/user-attachments/assets/f23e2479-c99c-4e64-bbf9-22223a75d402



<br><br>
### Using a USB flash drive
<div><img width=60 src="https://github.com/user-attachments/assets/651762d9-426b-49ad-8cb4-2cdc7b2aecb9" /></div>

https://github.com/user-attachments/assets/77839a7c-f4f0-4fe8-b198-c7e36b9bc334



<br><br>
### Networking: Ping and FTP
<div><img width=60 src="https://github.com/user-attachments/assets/601dd03c-0d77-4ec1-8307-b76d72c7923d" /></div>

https://github.com/user-attachments/assets/5f67eb18-9b9b-4c4f-b5dd-cf022fadc6d1



<br><br>
### Networking: Web browsing
<div><img width=60 src="https://github.com/user-attachments/assets/114475dc-eaad-4e1d-a4ab-3d190aa230e5" /></div>

https://github.com/user-attachments/assets/8a99601b-c7f9-49f3-8a99-dfec2aed2d76



<br><br>
### Hibernating and resuming
<div><img width=60 src="https://github.com/user-attachments/assets/6e18e1be-cc90-42f3-a545-62e2a8999ca3" /></div>

https://github.com/user-attachments/assets/c9758c15-9f8d-4058-a61f-c34481873b1e



<br><br>
### Solar panel detaching and re-attaching
<div><img width=60 src="https://github.com/user-attachments/assets/5d897835-51f1-47b4-bcbb-87023bcb50ec" /></div>

https://github.com/user-attachments/assets/3ee362a7-36d7-4355-b31c-2d30c2af94da



<br><br>
### System startup and color inversion
<div><img width=60 src="https://github.com/user-attachments/assets/fd269914-d7c6-41f8-ba09-b50392944180" /></div>

https://github.com/user-attachments/assets/8807fc27-6829-4491-978f-2921a8d432fe



<br><br>
### King's Quest 1
<div><img width=60 src="https://github.com/user-attachments/assets/12d81b80-13bb-4f15-9139-1087d0f94735" /></div>

https://github.com/user-attachments/assets/46d44474-fa79-49ea-9949-d85c53864d76



<br><br>
### QEdit editing autoexec.bat
<div><img width=60 src="https://github.com/user-attachments/assets/acd940e5-2030-4415-ab2c-4ad2efa77ddb" /></div>

https://github.com/user-attachments/assets/c6fe3bd9-1a23-458f-8ec3-6ddc4d6cbe65



<br><br>
### Wolfenstein 3D
<div><img width=60 src="https://github.com/user-attachments/assets/546131fb-ca1b-4ad8-acc7-720df0038656" /></div>

https://github.com/user-attachments/assets/e295904c-3d49-4982-8d65-b21779e48726



<br><br>
### Doom
<div><img width=60 src="https://github.com/user-attachments/assets/f79e5cf9-9837-47f0-9706-b3a232558d7a" /></div>

https://github.com/user-attachments/assets/fc9ec4e4-5f15-47cf-980f-c31522a4abcc



<br><br>
### CP/M-86 with Turbo Pascal
<div><img width=60 src="https://github.com/user-attachments/assets/d71eaacb-92f9-4bc3-baec-dd1b68bb983a" /></div>

https://github.com/user-attachments/assets/1cd02330-907e-408f-8e0b-55a8b3c6f605



<br><br>
### When and why to use "Color Emulation"
<div><img width=60 src="https://github.com/user-attachments/assets/83c75778-bd1e-41cc-9cf8-8109422c2773" /></div>

https://github.com/user-attachments/assets/255e8f95-449a-4774-9168-92bb78608fd5



<br><br>
### Planet X3 in Hercules graphics and VGA modes
<div><img width=60 src="https://github.com/user-attachments/assets/9b68da63-0ea2-4d47-b7fd-3e72d05a9275" /></div>

https://github.com/user-attachments/assets/d3770d11-64d8-491e-b530-e117e26e44bb



<br><br>
### ZZT version of King's Quest
<div><img width=60 src="https://github.com/user-attachments/assets/9dd0a693-d8e3-4094-badd-ff1a4859e9ca" /></div>

https://github.com/user-attachments/assets/27d068f2-91de-4887-bba5-bcdd4b63fbf2



<br><br>
### Scott Adams's Pirate Adventure
<div><img width=60 src="https://github.com/user-attachments/assets/cdee7b09-22fd-414b-a960-4c7b1f6e383b" /></div>

https://github.com/user-attachments/assets/f46fa540-cb63-4454-81f6-fe7e84447212



<br><br>
### SimCity
<div><img width=60 src="https://github.com/user-attachments/assets/2167c72f-0b0a-4a8b-97ed-9fb0a39fde6e" /></div>

https://github.com/user-attachments/assets/a8b7a0c0-d77a-4ccb-a051-0ceed19d8957



<br><br>
### Zork
<div><img width=60 src="https://github.com/user-attachments/assets/1d12efe1-577a-438d-a59a-0259012a46fe" /></div>

https://github.com/user-attachments/assets/8c195b5b-e7e3-49f2-b16d-81d6f7fbcded



<br><br>
### Minix with Colossal Cave Adventure
<div><img width=60 src="https://github.com/user-attachments/assets/4f26ca64-4a00-4bcc-9298-0007501bbc29" /></div>

https://github.com/user-attachments/assets/d3a6b4d2-5469-4d94-b6e4-5f47aeae23ec


<br><br>
## Sample Images
### Games
#### Doom
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/doom1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/doom2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/doom3.JPG">
</div>
<br>

#### Monkey Island
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/monkeyisland1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/monkeyisland2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/monkeyisland3.JPG">
</div>
<br>

#### Space Quest 1
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/spacequest1-1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/spacequest1-2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/spacequest1-3.JPG">
</div>
<br>

#### Space Quest 3
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/spacequest3-1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/spacequest3-2.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/spacequest3-3.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/spacequest3-4.JPG">
</div>
<br>

#### Wolfenstein 3D
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/wolf3d1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/wolf3d2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/wolf3d3.JPG">
</div>
<br>

#### Test Drive
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/testdrive2-2.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/testdrive1.JPG">
</div>
<br>

#### Prince of Persia
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/princeofpersia1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/princeofpersia2.JPG">
</div>
<br>

#### SimCity
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/simcity1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/simcity2.JPG">
</div>
<br>

#### The Black Cauldron
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/blackcauldron1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/blackcauldron2.JPG">
</div>
<br>

#### Commander Keen: Keen Dreams
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/keendreams1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/keendreams2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/keendreams3.JPG">
</div>
<br>

#### Commander Keen: Goodbye Galaxy!
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/keengalaxy1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/keengalaxy2.JPG">
</div>
<br>

#### King's Quest 1
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/kingsquest1-1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/kingsquest1-2.JPG">
</div>
<br>

#### King's Quest 4
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/kingsquest4-1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/kingsquest4-2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/kingsquest4-3.JPG">
</div>
<br>


#### Attack of the Petscii Robots
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/petsciirobots.JPG">
</div>
<br>

#### Planet X3
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/planetX3-1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/planetX3-2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/planetX3-3.JPG">
</div>
<br>

#### Police Quest 1
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/policequest1-1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/policequest1-2.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/policequest1-3.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/policequest1-4.JPG">
</div>
<br>

#### Windows Solitaire
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/windowssolitaire.JPG">
</div>
<br>
<br>

### Applications
#### Word for Windows
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/winword1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/winword2.JPG">
</div>
<br>

#### Excel
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/excel1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/excel2.JPG">
</div>
<br>

#### Word 5.5
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/word5_5.JPG">
</div>
<br>

#### Wordstar 7
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/wordstar1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/wordstar2.JPG">
</div>
<br>

#### MS Works
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/msworks1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/msworks2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/msworks3.JPG">
</div>
<br>

#### Scandisk
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/scandisk.JPG">
</div>
<br>

#### Adlib Jukebox
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/adlib_jukebox1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/apps/adlib_jukebox2.JPG">
</div>
<br>
<br>

### Networking
#### Wifi Setup
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/settingWifi1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/settingWifi2.JPG">
</div>
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/settingWifi3.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/settingWifi4.JPG">
</div>
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/settingWifi5.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/settingWifi6.JPG">
</div>
<br>

#### NE2000 driver -- DHCP client -- ping
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/NE2000.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/DHCP.JPG">
</div>
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/ping1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/ping2.JPG">
</div>
<br>

#### Web browsing
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/webbrowser1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/webbrowser2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/webbrowser3.JPG">
</div>
<br>

#### FTP and telnet
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/ftp1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/ftp2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/ftp3.JPG">
</div>
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/telnet1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/network/telnet2.JPG">
</div>
<br>
<br>

### System
#### Startup Screen
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/startup_screen.JPG">
</div>
<br>

#### Machine Selection
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/machine_selection.JPG">
</div>
<br>

#### Popup Menu: you can open it any time with a hotkey
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/pop_menu.JPG">
</div>
<br>

#### Audio Settings
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/audio_settings.JPG">
</div>
<br>

#### Using a USB flash drive
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/usb_flash_driver.JPG">
  <img width = "48%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/usb_flash_dir.JPG">
</div>
<br>

#### Hibernate and resume
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/hibernated.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/unhibernating1.JPG">
</div>
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/unhibernating2.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/unhibernated.JPG">
</div>
<br>

#### Transferring a file between Evertop and Book 8088 via RS232 serial connection with Procomm Plus
<div>
  <img width = "47%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/serial_xfer1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/system/serial_xfer2.JPG">
</div>
<br>
<br>

### Minix and its C compiler
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/minix/minixboot.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/minix/minix_c_compiler.JPG">
</div>
<br>
<br>

### Exterior Views
#### Right side showing earphone jack, USB flash drive, micro USB, PS/2, and DB9 RS232 serial ports
<div>
  <img width = "100%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/rightside.JPG">
</div>
<br>

#### Left side showing ethernet, variable voltage input, and TTL serial ports
<div>
  <img width = "100%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/ethernet&charger2.JPG">
</div>
<br>

#### SD card slot
<div>
  <img width = "100%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/sdcardslot1.JPG">
</div>
<br>

#### Standing up
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_angle1.JPG">
  <img width = "30%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_stand1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_rightside2.JPG">
</div>
<div>
  <img width = "32%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_leftside1.JPG">
  <img width = "32%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_rightside1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/standing_rightside3.JPG">
</div>
<br>


#### Unfolded
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/unfolded_front.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/unfolded_back.JPG">
</div>
<br>

#### Volt meter
<div>
  <img width = "60%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/voltmeter1.JPG">
</div>
<br>

#### Variable voltage charging: 3V, 9V, 13V, 19V
<div>
  <img width = "24%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/3v_charging1.JPG">
  <img width = "24%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/9V_charging1.JPG">
  <img width = "24%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/13v_charging1.JPG">
  <img width = "24%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/19v_charging1.JPG">
</div>  
<br>

#### Keyboard removed and keyboard cord storage
<div>
  <img width = "30%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_detached1.JPG">
  <img width = "31.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_detached2.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_detached3.JPG">
</div>  
<div>
  <img width = "63%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_cord_storage1.JPG">
  <img width = "36%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/keyboard_cord_storage2.JPG">
</div>  
<br>

#### Hacker friendly motherboard access
<div>
  <img width = "43.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/cpucover.JPG">
  <img width = "55.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/cpucover_opened.JPG">
</div>  
<div>
  <img width = "70%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/cpu.JPG">
</div>  
<br>

#### Current test points measurment in normal and power saving modes 
<div>
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/current1.JPG">
  <img width = "49%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/power_saving_current.JPG">
</div>  
<br>

#### Solar panel and removal
<div>
  <img width = "23%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_panel.JPG">
  <img width = "27%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_cord1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_cord2.JPG">
</div>  
<div>
  <img width = "14.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_plug_attached.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_plug_detached.JPG">
  <img width = "43%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/solar_panel_detached1.JPG">
</div> 
<br>


#### Using external keyboard and mouse 
<div>
  <img width = "70.25%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/extern_kbd_mouse5.JPG">
  <img width = "27.75%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/extern_kbd_mouse_rightside.JPG">
</div>  
<br>

#### All together and all apart: side by side comparison
<div>
  <img width = "49.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/games/testdrive2-1.JPG">
  <img width = "48.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/exterior/opendevice3.JPG">
</div>  
<br>

### Evertop Min - The same system, but minimized
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/spacequest3.JPG">
  <img width = "39%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/excel.JPG">
  <img width = "27%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/smallkbd1.JPG">
</div>  
<div>
  <img width = "27%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/smallkbd2.JPG">
  <img width = "28%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/dieboldkbd.JPG">
  <img width = "44%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/fullkeyboard&mouse1.JPG">
</div>  
<div>
  <img width = "29%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/rightside1.JPG">
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/leftside2.JPG">
  <img width = "37%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/frontside2.JPG">
</div>  
<div>
  <img width = "33%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/connected_devices1.JPG">
  <img width = "31.5%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/connected_devices4.JPG">
  <img width = "26%" src="https://github.com/ericjenott/Evertop/blob/main/images/min/with_max2.jpg">
</div>  

<br>











