
/* Ideas:
 *  Allow user to change the screen refresh rate.  For example, in some settings waiting for 3 seconds will allow a full screen
 *  draw within 3 seconds, but refreshing every 0.5 seconds may result in having to wait for 10 times as long for a full screen
 *  draw.
 *  
 *  Allow user to change the CPT (changed pixels threshold)
 *  
 *  Allow user to change the wait time and changedPixels lower limit before running instant refresh
 *  
 *  use a veriable similar to changedPixels to accumulate changed pixels over time for triggering instant refresh, rather than per frame.
 *  
 *  I could easily store some configuration values such as CPT, 4 and 2 pixel color definitions, auto refresh time, etc in a configuration file on the SD card
 *
 *  Could I use the ESP32's internal RTC to maintain time across light sleeps? 
 *  
 *  Set up two SPIFFS partitions: 1.) a read only boot drive that contains a small DOS installation with enough features for basic tasks and system recovery (command, qbasic, edit, sys, format, fdisk, etc.)  2.) a writeable partition the user can use to save work/data
 */

/*
  Created by Fabrizio Di Vittorio (fdivitto2013@gmail.com) - <http://www.fabgl.com>
  Copyright (c) 2019-2022 Fabrizio Di Vittorio.
  All rights reserved.


* Please contact fdivitto2013@gmail.com if you need a commercial license.


* This library and related software is available under GPL v3.

  FabGL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  FabGL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FabGL.  If not, see <http://www.gnu.org/licenses/>.
 */


 /* Instructions:

    - to run this system you need an ESP32 with PSRAM installed and an SD-CARD slot (ie TTGO VGA32 v1.4 or FabGL Development Board with WROVER)
    - open this with Arduino and make sure PSRAM is DISABLED
    - partition scheme must be: Huge App
    - compile and upload the sketch

 */

//20240420 available GPIO pins: (34,36,39 input only), (14 sd card SCK), (13 sd card cs), (15 scard MOSI), (21 on board LED/sleep indicator), (1, 3 usb uart), (2 sd card MISO), 0.  I can probably free up 4 by hard wiring EPD driver's CS pin to LOW and editing the GxEPD2 library to not use CS pin.
// so I can use 34,36,39 as input only, 1 and 3 if I disable USB UART at runtime, probably 4 if I hack it out of GxEPD2 library, 21 if I give up LED flashing, and 0.  3 inputs and 5 in/out, total 8 maybe available.
// I can probably use 34,36, or 39 for RxD on the serial port, freeing up pin 22.  
// https://en.wikipedia.org/wiki/RS-232
// RS232's DCE, DSR, RI, CTS, and RxD are all incoming, read only pins.  Only DTR, RTS/RTR, and TxD are output, write only pins.
// I currently only have a ttl-RS232 converter which only has TxD, RxD, CTS, RTS.  I can use rxd=36,txd=22,rts=23,cts=34
// now I can GPIO 1 and 3 if I disable USB UART at runtime, probably 4 if I hack it out of GxEPD2 library, 21 if I give up LED flashing.  4 in/out available.
// hacked GPIO 4 out of GxEP2 library and attached to ch375's INT pin.
// Theoretically I could not use the EPD's BUSY pin and just keep track of the timing myself to make sure I never write when I shouldn't thereby freeing up another IO pin.  But it sounds horrible.

// 20240929 I have left: GPIO 21 (on board LED), GPIO 1 & 3 (USB UART) that I can use at some degree of sacrifice.

/*
 * Urgent:
 * mouse can't be used with power saving.  unstable and freezes system.  Need to fix in ULP processor?
 * can't download new program with SD card mounted.  This started after I "fixed" the mounting method in fabutils.cpp. I'm temporarily fixing this by wiring GPIO2 to GPIO0(zero), wasting GPIO0(zero)
 * zmodem file transfer super unstable, constant bad crcs, almost unusable
 * can send files from evertop to teraterm quickly via Ymodem(1K), but not from teraterm to evertop
 * can send files from evertop to teraterm quickly via Xmodem(1K), but not from teraterm to evertop
 * seems that the screen updates are causing bytes to go unreceived while or around the time the screen is updating
 * pressing key to choose machine at startup breaks keyboard for later use
 */

 /* Serial port download file problem:
  *  8250 FIFO buffer is only 1 bit, so even about 1/10th ms delay causes a missed byte
  *  Updating e-ink screen takes about 61ms, but can be done by other core, so okay
  *  Writing to HD takes a few ms at least, so breaks inward data flow, and cannot be moved to other core for synchronicity's sake
  *  X/Ymodem downloads 10240 bytes before writing to HD.  This causes errors every 10240 bytes.  Teraterm<->Pcplus can recover, SecureCRT<->PcPlus cannot
  *  ZModem writes to disk multiple times per second, making download impossible for any programs.
  *  One option is to set up a large FIFO buffer in the 8250, and keep trigger INTRs until the buffer is empty
  *  Another is to use RTS/CTS
  */

/* 
 *  Might save lot of power by having a "single core" mode where video is rendered on the same core as the emulator, like my original method.  For mostly interactive use, when typing or moving mouse then waiting too see response, this wouldn't seem much slower.
 *  
 */


  /* to fix
   *  Pressing windows key locks up system if video_task writes to screen during user initiated update.  Need to "mutex" this problem
   *  Probably Alt-printscrn too.  And inverse? (menu key)
   */


   /* to do:
    *  Make BIOS video support level configurable in iBox.  Because VGA/EGA support are incomplete, some programs need to see that only MCGA or below is supported in order to avoid running in buggy or unusable EGA/VGA modes.  Other programs need to see that EGA or VGA is supported in order to run at all. 
    *  DONE: Fix bug where video task sometimes writes to screen some kind of partial image or something, but then it stays on the screen during future updates.  Maybe something to do with waiting for vsync?  Don't know. This didn't happen before using video_task, so it must be related to updating display while CPU is currently running, maybe currently writing to video RAM
    *  DONE: Fix bug where sometimes initial DOS screen background is partly or all black and needs refresh before returning to normal white
    *  DONE: Fix ibox bug where pressing "Enter" key on NO/CANCEL buttons (instead of "space" key) returns an "OK" button press result
    *  DONE: Fix speaker so that it isn't stuck at full voltage position, draining lots of power, all the time it isn't being used.
    *  DONE: Need to disable/pause video_task task when doing external screen clears/sharpen/refreshes etc.  Otherwise display sometimes locks up.
    *  ****** Hibernation files should be identified by something temporaly unique, like a UUID.  Otherwise once a new system is added, it might accidentally pick up a previously hibernated system's hibernation file and be ruined.
    *  Set up a super simple LPT Covox sound output device.  See https://github.com/mcgurk/Covox/blob/main/old/_old_esp32_dac_covox_very_simple.ino
    *     Set up the LTP1 ports in machine::writeport() to get the value written to 0x03c and dacwrite(25, value); 
    *  Might need an ibox menu option to choose between COVOX SPEECH THING and PC speaker because the two might not be able to be enabled at the same time.
    *  Move "mount disk" to first option in "windows key" ibox menu and pre-set it to default option.  Otherise accidental system/emulator reset is very likely.
    *  Remove as many mallocs from fabui as possible, change to fixed addresses in PSRAM
    *  Fix text width calculations for 8x16 fonts in ibox window size calcuations.
    *  DONE: Fix ibox file selection menu to not start with ".." selection highlighted, or else automatically start selection cursor(highlight) in file list window, so that the user doesn't have to press "TAB" to get there, even though there's no way for use to know the need to do this.
    *  Make CIVILIZATION work!
    *  Make wolf8086.exe work
    *  Make CASTLE ADVENTURE work!  Could it be writing to a port and then trying to read back the value, but instead getting 0xff, which causes random problems?
    *  Make Microsoft Adventure (both 40 and 80 column versions) work!
    *  ****** Restore support for ESP32 GPIO via ISA ports, as shown in FabGL QBASIC examples using IN and OUT commands
    *  Figure out how to run EPD at faster than 26Mhz using half-duplex mode.
    *  Possible to speed up SD card?
    *  Make a disk read cache in unused PSRAM.  Maybe option write cache warning the user that if enabled, could lose or corrupt data in the event of an unexpected power disruption.  Might be acceptable in some situations where data generation is not important, like gaming.  Also, there's a battery.  
    *  Make ibox floppy image selection menu remember last selection and default to same directy next time, saving navigation time/hassle
    *  DONE: Hack GxEPD library to not need CS pin for EPDs SPI connection, and physically tie EPD's CS pin to ground, thus freeing up one GPIO on ESP32
    *  Ditch GxEPD and use FabGL for all graphics?  Seems do-able, and would shrink program size; might increase speed.  See Evertop\screentest\GDEQ0583T81-Arduino
    *  DONE: Move emulated machine's video RAM into PSRAM.  No need for high-speed with EPD's slow refresh rate.  This will free up a lot of RAM for other things in emulator program
    *  Make Minix, Xenix, Coherent, Thoroughbred, and PCIX work.
    *  DONE: Add a power management circuit to fully shut off power after hibernating (and maybe report battery level?)
    *  Upgrade emulated 8250 UART to 16550
    *  Add an emulated 8087 copressor
    *  DONE: Add an Adlib or Soundblaster card (possible?  if so, surely lots of hard work.  Check other projects with emulated sound card like dosbox, 8086box, pce etc)
    *  DONE: Allow 2G hard disk images?  Seems like someone's done this.  I think BIOS needs editing to do so.
    *  DONE: Add an MPU-401 sound emulator from 8086tinyplusvcc
    *  DONE: Allow access to 1M RAM
    *  Upgrade to 286?  Bet it would be a huge amount of work
    *  DONE: Add an emulated network card (ne1000?) linked to ESP32 wifi, with Wifi configuration in ibox menu.
    *  Allow ibox menu access to USB thumb drives via CH376S chip?  This should be easy.
    *  DONE: Allow emulated machine direct access to USB thumb drives via CH376S chip?  Might be very difficult.  Book8086 does it.
    *  ****** Upgrade flash to 16MB, put a write only system drive C: and a read/write data drive D: in SPIFFS, mostly as backup for when SD card is not available or has problems.  Cram as much good stuff onto drive C: as possible: DOS (with QBASIC), MASM/TASM/NASM, ASIC, QB?, small fun games, NU, LIST.COM, Q.EXE, PKZIP/PKUNZIP, MSWORKS?, MSWORD?, CTMOUSE, DESQVIEW?, INFORM5.EXE + LIBS?, VBDOS? MS-QUICKC?  Want to leave at least 2M for writeable drive D:.  ESP32 program might fit in 2M.  2M program + 2M d: = 4M, leaving at most 8M for C:.  Will have to be very picky about what to put in C:.  Maybe DOS 5 is smaller than 6.22? 
    *  Fix hercules problems that show up in ZWDOS/ZWHERC and some games like planet X3
    *  Add support for more EGA modes
    *  Fix no text characters in EGA BIOS
    *  Add support for more VGA modes
    *  Fix bugs in bugs.txt
    *  Add CGA/EGA/MCGA graphics mode pallet shifting in ibox menu
    *  ******* Add sleep/hibernation parameter settings in ibox menu
    *  ******** Add hotkey settings in ibox menu, especially needed for keyboards that don't have "window" or "menu" keys
    *  Add options in ibox menu to set certain built in physical hardware peripherals' GPIO addresses to be used for built in hardware (such as mouse PS2 port or COM2 port) or to turn not use specific peripherals and free its GPIO ports to be used for other things
    *  Add Olivetti Grapics support?
    *  Upgrade EPD screen to 7-inch 800x480(600) version
    *  Show some useful information in the pixels to the right of column 639 on the EPD display.  Such as sytem status, battery level, etc.  CAPSLOCK! NUMLOCK! SCROLL LOCK!  And then don't turn on the light on the keyboard, to save power.  Disk access "light"?  Probably not worth the waste of power, but some users might find it comforting when having to wait.   ETH sleep/wake state.  
    *  Think of a way to take advantage of all the extra PSRAM.  FAT32 ramdisk that gets wiped every reboot?  Copy DOS and commonly used programs to it at system startup and add to path to increase speed.
    *  Add ibox feature to copy files from PC to SD over USB serial connection outside emulator.  This could be used to copy disc and diskette image files from PC without having to remove SD card.  Of course, a program on the PC would be necessary.
    *  Would it be possible for ibox menu to copy files from SD card directy into disk/disc image files?
    *  Add system interrupts to turn off mouse, keyboard, screen, com1, com2 so that we can programatically turn them off and temporarily use their GPIO ports for other things, then turn them back on when we're done doing those other things.
    *  Add ibox menu option to use bluetooth for something?  Serial over BT?  Keyboard?  Mouse?  Audio out?  Audio in?  BT file transfer?
    *  Set up a virtual LPT port that exists only in software to which various virtual devices can be connected such as Disney Sound Source, Covox Sound Master, printer (at least text printer), etc.  Devices could be selected via ibox menu.  Probably especially useful for sound output.
    *  DONE: add headphone jack
    *  It would be nice to be able to transfer files from modern PC to ESP32's SDCard via wifi.  Really nice if it was via SMB network share!  But web interface will work too.
    *  ******* Add "kilo" or an improved version of my "texor" text editor that I used for minipc2.7 to ibox menu for editing files directly on SD card/SPIFFS.  Hope I can use FabGL windowing system and mouse to make it good enough for basic word processing.
    *  ******* Show cursor (non-blinking) in ibox menus.  
    *  ****** Make everything highlight with bold borders when tab-selected in ibox menus, not only buttons.
    *  If hibernate in 40-column text mode, reverts to 80-column when unhibernating
    *  ****** Need to add a feature to format new SD cards and copy files from old card to new card.  This means using lots of PSRAM to buffer data, and be able to extract and insert cards with automatic unmounting and remounting.
    *  Bug: when pressing ctrl+alt+del during adlib play, last note keeps playing forever after reset until next time using adlib card.  Need to reset adlib/soundgen/i2s on machine reset
    *  Make adlib buffer and frequencies parameters that can be changed in ibox menu
    *  DONE: Make pc speaker and adlib on/off changeable in ibox menu
    *  Fix DOSMID.EXE can't detect Adlib card problem
    *  DONE: 8086tinyplusvcc_archive has a virtual LPT port virtually connected to a virtual Disney Audio device.  Maybe I should implement this, or implement some other virtual device on virtual LPT.  Like a printer that prints out graphics images to the SD card outside of the emulator.
    *  Maybe some way to copy files directly between SD card and disk images from within or without the emulated machine?
    *  Several midi playing programs detect adlib card but there is only silence when they play midis on it.  This is also the case in 8086tinyplusvcc_archive
    *  Why is the PC speaker square wave raspy using FabGL's SoundGenerator system, but pure and smooth when directly using ESP32?  Fixing this might drastically improve SoundGenerator's sound quality.
    *  Copy the debugger code from 8086tinyplusvcc_archive to create an extra-emulator debugger?  That would be cool.  A real ICE, actually.
    ×  Make an emulated mp3 player device that can be accessed through ISA IO port.  Software sends the MP3 to the device in segments and the ESP32 plays them.
    *  Add a sound font file (*.sf2) for the MIDI output device, such as in: https://github.com/earlephilhower/ESP8266Audio/tree/master/src/libtinysoundfont
    *  small sf2 files: https://github.com/earlephilhower/ESP8266Audio/raw/master/examples/PlayMIDIFromSPIFFS/data/1mgm.sf2   , 
    *  ******* Some ideas for extra-emulator bonus software: simple editor/word processor, e-reader, mp3 player.  The first two can be more power-saving than emulated software.  The reader can read more formats/languages than the emulated software.
    *  Figure out how to make 256mb.img\TRAN\TRAN.EXE work (currently doesn't output sound via pc speaker, but does on 8086tinyplusvcc)
    *  Fix DOS command line cursor always disappearing (problem started after implementing video_task)
    *  512mb.img\covox\kq6\install.exe doesn't detect Disney Sound Source.  Sierra.exe says' can't initialize audio hardware when it's selected.
    *  ****** Besides normal and bright(bold) fonts, try to make at least one other font, then use it when a character's attribute is different from most of the other characters on the screen.  This might help with some of those menus where you can't tell what you're selecting, like Windows 3.0 setup.exe (settings.exe?)
    *  DONE: 360K floppy drives transfer data at up to 250Kbps.  If I can connect the USB drive reader to a serial port and transfer data at 250Kbps, it's a fair approximation of a floppy drive experience.
    *  If sdcard root's mconfs.cfg file has a formatting problem, crashes into reboot loop.  Create startup option to ignore mconfs.txt file at start up.  That way the user can at least use ibox menu to correct/replace mconfs.txt file
    *  After booting up and getting DHCP address once, I can't get it again or do networking after a ctrl-alt-delete or "reb" reboot 
    *  DONE: See if I can use the USB drive to serial chip (CH375?) with the driver program in BOOK8088 to make it accessible via a drive letter and regular DOS commands.
    *  Cursor is getting drawn at least 1 frame earlier than the text that it should be "drawing".  Might be because I'm calling "drawCursor()" twice, once in redrawScreen() and once in video_task().  No.  I rarely call redrawScreen(), so shouldn't be caused by this.  drawCursor() uses display.fillRect() to draw cursor.  Actually, it's probablly because I'm directly setting screenDirty = true outside video_task() whenever cursor parameters change.
    *  DONE: Arachne 199 uses IMUL - need to implement IMUL!
    *  After using MTCP stack and then "iBox->restart emulator", get Guru Meditation Error: Core  1 panic'ed (LoadProhibited) in NE2000SendHandler(unsigned char*, unsigned short) at C:\Users\lewis\Desktop\项目\Temp\Evertop\PCEmulator_EPD_24/machine.cpp line 381.  That is esp_wifi_internal_tx(WIFI_IF_STA, data, count)
    *  I think I need to do something with the MPU401 IRQ/Interrupt
    *  Can I implement a virtual CD rom drive that can load *.ISO images via a DOS cdrom drive driver?
    *  Implement XMS from https://github.com/ecm-pushbx/8086tiny
    *  Enable ROM basic.  See https://github.com/adriancable/8086tiny/commit/8c9a82627d5c054ed6b84ce3e349a458b19b0c6c.  Also https://en.wikipedia.org/wiki/IBM_BASIC, "BASICA will not run on non-IBM computers (even so-called "100% compatible" machines) or later IBM models, because they lack the needed ROM BASIC".  Spent a few hours on this. Not trivial.  Shelve it for now. 20240706.
    *  Try fixing QNX/XENIX can't boot bug with: https://github.com/retrohun/8086tiny
    *  Try to fix Hercules slanted resolution bug with: https://8086tiny.freeforums.net/thread/11/hercules-emulation-support-resolutions.  Also https://github.com/adriancable/8086tiny/blob/8c9a82627d5c054ed6b84ce3e349a458b19b0c6c/bios_source/bios.asm lines 1068-1093, https://github.com/adriancable/8086tiny/blob/master/8086tiny.c lines 592-593
    *  Above Hercules problem is partially fixed.  But ZWDOS only displays pinyin input bar properly, screen contents are not shown or are jumbled.  
    *  DONE: In HGC mode, PX3 shows clearly but every 4th line (lines 3, 7, 11, 15 etc.) is missing (black).  256mb.img\games\prehisto\historik.exe has same missing lines problem, so it is not a "feature" of Planet X3.  Okay, figured it out.  Register number 9 of the HGC card is "number of scanlines per row minus one".  There is a popular 640x300 mode, where this is set to "2" (thus making number of scanlines per row "3"), as opposed to "3" (which would make number of scanlines per row "4") as in the 640x400 mode and 720x348 mode.  So need to upgrade my math in the pixel rendering algorithm!
    *  DONE: Implement 80186's IMUL as in https://github.com/ecm-pushbx/8086tiny/blob/master/8086tiny.c. 
    *  Implement 80186's BOUND as in https://github.com/ecm-pushbx/8086tiny/blob/master/8086tiny.c
	  *  Make CGA's color composite 640x200 and 320x200 color artifact modes translate to multiple patterns per pixell as my EGA does. See https://en.wikipedia.org/wiki/Color_Graphics_Adapter and https://en.wikipedia.org/wiki/Composite_artifact_colors
	  *  Make CGA's 160 × 100 "text mode" hack in 16 colors mode work with pixel patterning as EGA does.  See https://en.wikipedia.org/wiki/Color_Graphics_Adapter
	  *  Enable HGC 43-line display mode
	  *  Enable VGA/EGA 43/50 line text modes.  See https://groups.google.com/g/comp.os.msdos.programmer/c/lJBudgKWgdo
    *  Fix QBasic can't detect Hercules card even after running MSHERC.COM.  BIOS says "VID82" AND "VID83", and "bios.cpp:1385: INT 10h, AX=0x1112, BX=0x0800, unsupported INT 10h, AX = 1112" when trying to run "SCREEN 3" in QBasic.
    *  256m\games\sq3 graphics horrible in MCGA mode.  Seems like every other pixel is missing, making black vertical rows of missing pixels
    *  Noticed in version 25, MCGA mode seems to print a zebra line acorss the bottom of the screen, below the border line
    *  Any way to make creation of floppy and HD images faster?
    *  ******** FabUI startup menu edit machine options UI, if you press RETURN key on the "..." box of a floppy drive, UI freezes.
    *  HD20_mychoices.img's Wolf3d CGA version is much worse performance than other copies.  Super slow and graphics messy.
    *  For some programs, such as HD20_mychoices.img's Indy3 in CGA mode, screen constanly updates even when no pixels have changed, because the "Setting m_CGAMemoryOffset(L) = 0x00" and "CGA, 320x200 graphics mode" messages constantly coming out.
    *  QBASIC "SOUND 500,30" generates a very clean 500Hz tone, but other frequencies are raspy.  1000Hz is clean too.  1500Hz produces same as 500Hz sound.  Something's messed up here.  Sampling rate not right?
    *  HD20_mychoices.img's princ13 doesn't show graphics in game play when selecting HGC.  Last graphics mode message to come out is "CGA, 80 column text mode", and if you manually refresh screen, it shows the latest text screen (command prompt), although you can still hear the sounds of game play going on.
    *  After using HGC mode once in HD20_mychoices.img's princ13, running setup.exe shows chosen options in low-brightness so you can see what you're choosing, but normally your choice and other choices are all high-brightness, so you can't see.
    *  Try out Doom8088.  It can run on HP200LX in CGA mode
    *  ******** Before restoring from hibernate, check disk image files' dates to see if they're the same as when hibernating.  If different, abort un-hibernate and perform normal boot.  Otherwise disk images will be corrupted.
    *  I have an idea for a new kind of game.  A mix of treasure hunting / geo-caching / war driving / hacking.  At various places around a city, set up beacons that invide people to hack them.  If they succeed in hacking them, they get a reward or a clue to the next beacon.
    *  Allow a keypress to reset ESP32 or retry mounting SD card when "this system requires an SD card" error occurs.  Wait for 10-30 seconds for this input, then deep sleep / power off.
    *  DONE: SOMETIME BETWEEN VERSION 24 (adlib just added) and version 25 (ch375 adding), adlib and midi sound output became very quiet, turning up volume knob only produces a lot of background noise, sound quality is terrible.  Might be related to faster video memory scanning / screen updates core being busier ******
    *  DONE: STARTING IN VERSION 24 OR EARLIER, WIFI SCANNING OF AVAILABLE SSIDs failes, so cannot connect to new networks.
    *  Remove HTTPClient.h from code
    *  Add a HTTP server that the user can activate temporarily after power on that allows transferring of files (esp. disc images) via wifi to the SD card.
    *  Any way to allow a folder on the sdcard be mounted as a drive under DOS?
    *  Copied 65,612,757 bytes from gold aigo usb drive to d: in less than 24 hours.  Don't know how long really.  Started failing reading in less than 24 hours.
    *  Make a color screen with RA8875 chip that can be attached as an option when turning off the EPD screen's DIP switches and setting color screen mode in system software
    *  Allow the iBox menu system to access USB drive to mount disk images, copy disk images from SD card, etc.
    *  Make a way to duplicate SD cards for making backups.  Maybe using USB drive that is bigger than the SD card capacity?
    *  Make HGC 720x348 mode have the option to skip every 8th column or not show the left 45 and right 45 colums.  Some programs don't show content on the sides since their graphics data comes from CGA or other non-720 wide data.
    *  ****** Create the option to format unformatted sd cards, in case the user has an unformatted sd card but no other computer around that can format it.
    *  ****** Need a way to partition and format unformatted or non-FAT16 USB drives
    *  When ch375 has been on for a long time (don't know whether idleness is a factor or not), it seems to shut off.  Accessing it results in no response.  Ctrl-Alt-Del does not "wake" it up.  But pressing RESET button an restarting the emulator does wake it up.  Restarting emulator through iBox also wakes it up.  I should be able to "guess" at this state and run the steps that the system goes through to initialize the ch375 to "wake" it up.
    *  When ch375 is copying files (or any sending / receiving data maybe?), if we halt the system momentarily, such as opening iBox menu, then resume the system, it often (always?) messes up the chip.  Maybe need to check the chip's activity first, and wait until the driver it is ready to send another command byte but hasn't yet sent it before pausing the system.
    *  ******* Disable lighting up capslock/numlock/scrolllock keyboard LEDs, and instead put these markers on screen in the far right 8 pixel empty column.  Will save a lot of power.
    *  NO: Now that I have a good speaker and amplifier, consider going back to using sine wave instead of square wave.  It sounds much nicer.  Or make it a user choice in iBox menu.
    *  Enable hall effect sensor access via ISA bus
    *  Enable temperature sensor access via ISA bus
    *  DONE AND UNDONE: Add a built in ammeter display that can be briefly turned on to check current then turned off again?
    *  DONE: Since I have a GPIO programmable hardware power shutoff circuit, add a ISA output port to allow software to cut off power.
    *  Add ISA port to force system hard reset.
    *  Create ISA hardware ports to: hibernate, hard refresh screen, soft refresh screen, toggle back/foreground colors, open iBox menu, set esp32 cpu speed, set power saving mode, enter light sleep, enter deep sleep, other system functions
    *  Maybe make the iBox menu look more like an early 90's AMI style BIOS UI?
    *  Make my own PS2 keyboard with ultra power saving.  Consider BBQ10KBD as example, implement PS2 protocol.
    *  TRIED AND FAILED, IT SEEMS TO BE COMPILED INTO SD LIBRARY, MAYBE CAN RECOMPILE WITH ESP-IDF: Can I free GPIO13 from the SD card somehow, since I'm not using any other SPI devices on HSPI?
    *  ****** Ideally, I should be able to insert a new, blank, unformatted SD card, format it, create hard disk and floppy disk images on it, boot from read only disk image in SPIFFS, mount the disk images, use DOS's sys command to make them bootable, and then boot from them.
    *  ****** Fix screen not showing random jumbled characters at startup the way the original IBM PC should.
    *  DONE: If possible, make the screen react immediately, at least doing it's flashing reset sequence, or better yet show some kind of POST display, immediately after power on or reset, rather than first finding kbd, mouse, SD card, etc.  User will feel better seeing an instant response to power on or reset, rather than waiting and wondering.
    *  Crackling adlib/dss/midi sound output might be because getting sudden incorrect "0" (zero) or 255 samples between correct samples, making the speaker jump suddenly.  If this is the case, and if I can't fix the source of the problem, maybe I can at least add some logic to filter out these extreme differences and replace them with the previous sample.
    *  It would be great to be able to mount more than one partition on a USB drive.  Then I could have multipe 2G partitions, makeing for a lot of space available for storage on a drive.  E: through Z: drives is 22 drives, that would mean 44Gigs of storage space available on a singele USB drive!
    *  Manhole game seems to run way slower on version 28 than it did on early versions.  Check and compare.
    *  #1986\toysho~1.zip => TOYSHOP\START.COM.  Title screen constantly refreshes even though debug output says no pixels were updated.  This is because cursor_pos = 0, last_cursor_pos = 1920
    ×  1986\hi-qst~1.zip\hi-q.exe, holding shift + arrow key does not jump pieces as the instructions say.  Need to test this out on other platforms to see if it is a problem with my keyboard/key handling routines or if this game is just this way. 
    *  Maybe add an iBox menu option for keyboard click sound?  Might help with feedback since built in keyboard physical feedback kind of sucks.
    *  Make an ISA port API for connecting to / controlling wifi from within emulated machine's OS.
    *  Need to add label to volt meter saying it consumes about 13mA (verify number)
    *  DONE: Add code to end of hibernate() to go into deep sleep mode and alter the user on screen and audibly that system is in deep sleep mode and still wasting power, needs to be manuall shut off.
    *  ******* When I press a character key on the keyboard, I want to see the character print out on the screen as soon as possible, so I have optimized the display code to refresh ASAP after a keystroke.  But when it's not a character key, such as carriage return, refreshing as soon as possible causes the screen to refresh half way through the command prompt switching to the next line.  How to fix?  Check if character key or other key, and only respond so immeediately to character keys?
    *  ******* Ethernet port default setting should be to sleep n seconds (10? 15? 20? 60?) after last packet sent if no non-broadcast packets received in n seconds.  This default setting should be reconfigurable in ibox menu so that the ethernet port stays awake all the time, if user wants, such as when running a server.
    *  ******* Make the SPIFFS writable disk mountable from other emulated machines so that files can be easily copied between SPIFFS machine and sdcard machines
    *  ******* Need to run machine reset routine when rebooting from ctrl-alt-delete, otherwise lots of hardware peripherals don't get reset, which makes them malfunction after system reset.
    *  ******* For the different colored foreground text, I should try to make as many different fonts as possible, and use one for each different color
    *  DONE: For the different colored background text, I should try to make as many different backgrounds as possible, and use one for each color.
    *  Get GameOS to boot
    *  ******* At any startup menu, if no input for several minutes, power off.
    *  ******* When set to power off after n seconds/minutes idle, make sure this setting is also implemented during ibox menu sessions.
    *  Try overclocking ESP32
    *  MSDOS 6.22's fdisk.exe crashes emulator on exit/reboot with "80386+" and "8087 MATH Coprocessor" errors after running BIOS::syncTicksWithRTC().  This is not detrimental as a hard reboot (button press) reboots and then new partition settings can be used normally.
    *  ******** If a drive image is used in machine A which then hibernates, and is also used in machine B, after machine A hibernates and then machine B changes drive image, then machine A unhibernates, drive image will be corrupted.  Need to check for this situation and ask user if okay to delete hibernation file that is "locking" the disk image before it can be used in any other virtual machine.  Or maybe it can be loaded as "read only" if the user so chooses?
    *  Can I go to 4 hard drives instead of only 2?  (constexpr int DISKCOUNT = 6;?)
    *  Should try to re-mount the sdcard if it is temporarily removed or has error because of momentary low voltage
    *  After exiting scandisk.exe, text characters are blank, can only see cursor move on screen.  "mode 80" fixes it.
    *  Can use wifi to transmit screen pixels to a modern Windows client program over UDP to display screen on modern computer.  First just showing same as EPD 1-bit per pixel graphics, then expand to showing full VGA with color
    *  Set up ISA I/O ports to control ethernet adapter: initialize (uses much less power prior to init, so by not initing, can save power until eth port is needed), sleep, wake, enter low-power mode, leave low-power mode (enter high-power mode), 
    *  Make eth port low power mode configurable (on or off) in iBox menu
    *  ****** Make eth port default sleep mode configurable (on or off) in iBox menu
    *  ******* Make eth port sleep parameters, such as how long after no send activity, how long after no recv activity, configurable in iBox menu.
    *　******* A small design bug that affects the UX is that when networking mode is set to Ethernet, pins 23 is pulled low and then high in an effort to reset the ethernet port.  Leaving it pulled HIGH results in COM2's RTS pin being kept high, and this causes the RS232 pin 7 (RTS) to be kept at -5.5V, same as pin 3 (TX), so the mouse can't get it's 11V power it needs.  This happens even when the physical signal switch is set to COM2 instead of Ethernet.  
    *  Non-PC apps: Word processor (write), mp3 player (listen), e-reader (read), tinybasic (BASIC), Swieros (SWIEROS), file manager (including copying disk image files to/from sd card and USB drive, so user can make copies and backups of disk images with needing access to another computer.  Others to consider: ESPectrum, z-code game interpreter, Vic-20 emulator, Altair 8800 emulator, jpeg viewer, ESP32TinyCPC Amstrad emulator, serial terminal, telnet/ssh client, web browser (port microweb?), file browser, ESP32TinyChip8, zim file reader + wikipedia + gutenburg etc. 
    *  Can set the default processor speeds to whatever seems the best balance between power saving and user experience speed for various bootup systems.  For example, PC emulator needs 240Mhz.  Z-machine interpreter, Swieros, e-reader or Chip8 emulator might be OK with 10-40Mhz.  
    *  Ethernet port seems to not work after ctrl-alt-del style reboot.  Have to hard reset emulator/esp32 board to get it working again.
    *  Make 40x25 text mode bold font
    *  Make 40x25 text mode background color emulation
    *  Make 40x25 text mode foreground color emulation
    *  I should make the system firmware upgradeable from sd card and USB drive.  User puts a binary image with a certain name or name format, the system sees this at boot, or when prompted to by the user through the iBox menu, checks it's validity (version, crc, signature, etc.), and uses ESP32's OTA function to upgrade firmware.  This way someone with no other computer besides Evertop could use Evertop to connect to the internet, download new firmware image, and update firmware.  Totally standalone.
    *  I should make some arduino/esp32/esp8266 devkit boards that are programmed directly through serial port, that way Evertop users can still enjoy microcontroller programming even from an old DOS system.  Could use various BASIC flavors, or picoc C interpreter, or maybe even an expanded Swieros.  With Swieros, maybe could have the microcontroller only run the VM, and the program be compiled by a DOS compiler and then downloaded via serial port to the microcontroller's VM.
    *  Here is an esp32 program to flash a binary image file to another esp32/esp8266 over serial connection.  I should either port this to DOS or at least make it avaible in iBox Menu so users can flash BASIC/picoc/swieros/swiercc interpreters/vms to other esp32s to do microcontroller development directly from DOS. https://github.com/espressif/esp-serial-flasher/tree/master/examples/esp32_example
    *  ****** Text input field is too narrow in iBox menu for creating hard disk when inputting disk size.  Make wider.  Consider 4000000 size input (4GB) expressed in MB
    *  Get Coherent 2.3.43 OS to boot https://www.icl1900.co.uk/unix4fun/coherent/ftp/distrib/Coherent-2.3.43/ https://www.icl1900.co.uk/unix4fun/coherent/ftp/manuals/
    *  Make a soft-ICE like debugger that can be broken into with a hotkey or via iBox menu.  Won't cost any extra runtime resources when not opened, since we're an emulator anyway!  Could have options to run on-screen debugger or a serial port debugger, based on users's choice
    *  Use esp_reset_reason() to find out the reason for system reset (or power on), and if not a normal reason, display error message to user and wait confirmation rather than just saying "SYSTEM STARTING"  https://gist.github.com/kassane/7bdb782a1984d0c6581ae7b44e1fc0c2
    *  I could have a "cleaner" alternative to color emulation mode: When drawing a character, check if its background is a different color than that of the characters above it.  If so, draw a 1-pixel wide line at the top of the character currently being drawn.  Do the same for the left side.  Do the same as well for text color.  Four situations: txt and bg same: no line.  txt same, bg different: solid line.  txt different, bg same: 1-on 1-off dotted line.  txt and bg both different: 2-on 2-off dotted line.  
    *  Desperately need a way to copy files from sd card outside disk images into disk images, or from USB drive outside 2G DOS accessible partition into disk images, using only Evertop itself, with no other computer.  User needs to be able to maintain a huge software and data resource library on sd card(s) (and/or USB drives), and copy them from sdcards/usbdrives into their disk images for direct use in their emulated guest machines.
    *  Need to add unmount disk feature in iBox menu, otherwise its stuck in drive and can't reboot to hard drive.
    *  Should add boot order, such as A, C, or C, A, etc.  Currently it only boots from one selected drive, and if that's empty    ....   hmmm....  there's some difficult logic here, I'll have to consider this later when I have time.
    *  Make ISA port APIs for all iBox menus that make sense: shutdown, esp32 reboot, hibernate, light sleep (this way DOS programs can be written to be actively power saving), mount/unmount floppy disks, turn sound cards on/off, turn audio on/off, toggle color emulation mode, etc. 
    *  Can I make a LPT parallel port if I turn off enough peripherals and use (borrow) their GPIO pins? Reference https://blog.adafruit.com/2024/04/09/an-esp32-emulator-for-lpt-parallel-port-printers-vintagecomputing/.  Parallel port has 8 data pins (D0 - D7), and at least 8 other pins for printer status information, plus at least a GND pin, and quite likely a 5V and/or 12V voltage supply pin.  I don't think I can even get D0-D7 working directly.  Maybe consider a ttl COM1 serial to DB25 parallel port adapter using another esp32 or esp8266?  COM1 can go up to 3000000 baud, so it could be fast enough.  
    *  Or peraps I can barely manage an 8-bit parallel port.  I have 8 I/O ports I can use, (not including kbd i/o, but including mouse i/o) (10 if I can use COM1 ports 1 and 3).  
    *  Both Minix and myz80 z80 emulator use a "hardware scrolling mode" that doesn't work on Evertop.  On Minix this can be disabled in favor of software scrolling mode by pressing F3.  Need to fix things so this works in hardware scrolling mode.  Seems like it might be breaking or circumventing "wait for busy pin" on EPD.  This scrolling seems to be accomplished by successively incrementing CGAMemoryOffset by 80 bytes (1 line), such as: Setting m_CGAMemoryOffset(H) = 0x50, Setting m_CGAMemoryOffset(L) = 0xa0, Setting m_CGAMemoryOffset(H) = 0xf0.  This should work.  Since its not working, seems like it should be a simple fix.
    *  ZX Spectrum seem so to be wildly popular.  Add it!
    *  I hate that I can't play Bootle (Bootsector version of Wordle) since I barely support text character color emulation.  Any way to solve this systematically?
    *  Make all the BootOS/BootDev/Bootsector games work!
    *  GoodDisplay says to contact them for grayscale code:  https://www.good-display.com/product/440.html  Grayscale refresh takes 2 seconds.  Might be good to have an option grayscale display mode for viewing pictures.
    *  Consider centering resolutions that don't fill up the vertical screen in the middle instead of at the top.
    *  Seems I may have not properly implement LOCK prefix.  Check ecm-pushbox's 8086tiny.c implementation.
    *  It would be good to have a way to programmatically output strings from the guest system to the emulator's debugging serial output.  
    *  ****** Use or modify then use dosidle (https://github.com/galazwoj/dosidle/blob/master/DOSidle_251.asm) with a ESP32 sleep API via ISA I/O port to put ESP32 to light sleep when dosidle triggers cpu sleep, then wake it up again when dosidle's wake conditions are met.
       Better yet, study dosidle source code and figure out how to make the sleep decision directly in ESP32 code.
       For idler research, check:
       https://www.os2museum.com/wp/idle-dr-dos/
       https://www.vogons.org/viewtopic.php?t=25117
       https://forum.vcfed.org/index.php?threads/what-other-tools-like-dosidle-are-out-there.78388/
       https://www.bttr-software.de/forum/board_entry.php?id=16269
       https://github.com/OS2World/DOS-UTIL-DosIdle
       https://archive.org/details/dosidle
       https://www.vogons.org/viewtopic.php?t=43384
       https://github.com/galazwoj/dosidle/blob/master/DOSidle_251.asm
       MSDOS's power.exe
       Int 28h
       Track the frequency of Int 0x28 calls and sleep if they reach a certain high frequency.
       MAYBE BEST OF ALL INFO: https://www.os2museum.com/wp/idle-dr-dos/
       
    *  ****** Add a user configurable option to beep or click or some other noise immediately when starting to power on so that user gets some positive feedback
    *  ****** Add a LoRA chat client outside emulator in native mode (same as BASIC, text editor, e-reader, etc)
    *  There is a libzim zim file format read/write c++ library: https://wiki.openzim.org/wiki/Libzim, https://github.com/openzim/libzim.  Make a native ESP32 zim file reader so we can have great things like Wikipedia on SD card.  Important for survival / internet in a box (IIAB) use.
    *  Add EMS using PSRAM.  https://texelec.com/product/lo-tech-ems-2-mb/ https://www.lo-tech.co.uk/wiki/Lo-tech_2MB_EMS_Board https://www.lo-tech.co.uk/downloads/2MB-EMS-Board/LTEMM-r01.zip https://github.com/davidebreso/dosutils/tree/main/sxtemm https://github.com/karcherm/topemm
    *  Field for file input name for saving hard disc image when creating is too narrow, need to make at least 50% wider.
    * 
    * 
    * 
URGENT:
    * # 1. Make system setup menu accessible via <del> key during bootup.
    * 1.5 URGENT: fix power saving features, at least mouse.  Implement INT 28h.  Make power saving mode parameters adjustable in <del> menu and pop-menu (if easy).
    * Make power saving mode selectable via iBox menu instead of difficult hotkeys.
    * Make default power saving mode selectable in <del> menu.
    * 2. Allow assignment of other keys for system pop-up menu (default "Windows" key) and reverse color menu (default "Menu" key)
    * # 3. Make wifi SSID selection/password input part of <del> menu.  
    * 9. Make sure hibernate works with mouse.
    * 11. Make sound configuration menu more reasonable.  Toggles with on/off indicators. Especially because enabling already enabled sound card crashes emulator.
    * 16. Fix freeze up when "Enter" key is used to press "..." buttons in system edit window
    * 15. Fix mouse cursor invisible in ibox lists
    * # 16. Fix weirdness where you select and connect to Wifi in one place, then enable wifi for networking in another place.  These should be done in the same place.  Like when you select wifi for networking, it automatically goest to scan ssids, select, input password.
    * Text input field is too narrow in iBox menu for creating hard disk when inputting disk size.  Make wider.  Consider 4000 size input (4GB) expressed in MB
    * Fix or workaround for warped enclosure.  Maybe have to add adjustable foot that screws into NE corner big screw hole.
    * Hide wifi passwords
    * Add hibernate to popup menu
    * 3. Make wifi SSID selection/password input part of popup menu if easy.
    * change power saving mode via popup menu
 
    * 
    * 
    * 
    * Firmware/software fixes that must be done before shipping:

    * 
    * 
    * 4. Move any "preferences" that should be SD card specific (such as last booted system) on to a config file in SD card.
    * 5. Add credits - legal info - license info -especially FabGL credit, to <del> menu
    * 6. Make system to boot choosable from <del> menu, otherwise default boot directly to last chosen system.
    * 7. Add small write-only DOS hard drive in flash chip, with essential DOS programs like sys, format, fdisk, qbasic, edit, qbasic help, things essential for setting up new disks on SD card, and for basic useability if no SD card present.
    * 8. Add small read-write DOS hard drive in flash chip for user files when not using SD card.
    * 10. Make disk image files lockable so that they won't be unhibernated into another system when hibernated.
    * 
    * 12. Make applicable emulator options configurable per emulated machine rather than system wide.
    * 13. Add texor text editor
    * 14. Add e-reader


    * 
   * 
    * 
    * Subprojects to complete before shipping:
    * 1. SD card with at least 2 DOS systems plus Minix, Windows 3.0, Windows 3.1 beta, games, dev stuff, system utils, word processors, editors, etc.
    * 2. Default system, but last on startup menu, runs demo program from autoexec.bat, showing screen shots of various software/systems Evertop can run.
    * 3. In-depth video of Evertop's features
    * 4. 
    * 
    * 
    * Hardware changes before shipping:
    * 1. remove TTGO's GPIO 21 LED
    * 2. change flash chip to 16M
    * 
    * 
    * 
    * 
    * Add hotkey settings in ibox menu, especially needed for keyboards that don't have "window" or "menu" keys
    * Make everything highlight with bold borders when tab-selected in ibox menus, not only buttons.
    * FabUI startup menu edit machine options UI, if you press RETURN key on the "..." box of a floppy drive, UI freezes.
    



    * 
    * ****** Hibernation files should be identified by something temporaly unique, like a UUID.  Otherwise once a new system is added, it might accidentally pick up a previously hibernated system's hibernation file and be ruined.
    ****** Restore support for ESP32 GPIO via ISA ports, as shown in FabGL QBASIC examples using IN and OUT commands
    ****** Upgrade flash to 16MB, put a write only system drive C: and a read/write data drive D: in SPIFFS, mostly as backup for when SD card is not available or has problems.  Cram as much good stuff onto drive C: as possible: DOS (with QBASIC), MASM/TASM/NASM, ASIC, QB?, small fun games, NU, LIST.COM, Q.EXE, PKZIP/PKUNZIP, MSWORKS?, MSWORD?, CTMOUSE, DESQVIEW?, INFORM5.EXE + LIBS?, VBDOS? MS-QUICKC?  Want to leave at least 2M for writeable drive D:.  ESP32 program might fit in 2M.  2M program + 2M d: = 4M, leaving at most 8M for C:.  Will have to be very picky about what to put in C:.  Maybe DOS 5 is smaller than 6.22? 
    ******* Add sleep/hibernation parameter settings in ibox menu
    ******** 
    ******* Add "kilo" or an improved version of my "texor" text editor that I used for minipc2.7 to ibox menu for editing files directly on SD card/SPIFFS.  Hope I can use FabGL windowing system and mouse to make it good enough for basic word processing.
    ******* Show cursor (non-blinking) in ibox menus.  
    ****** 
    ****** Need to add a feature to format new SD cards and copy files from old card to new card.  This means using lots of PSRAM to buffer data, and be able to extract and insert cards with automatic unmounting and remounting.
    ******* Some ideas for extra-emulator bonus software: simple editor/word processor, e-reader, mp3 player.  The first two can be more power-saving than emulated software.  The reader can read more formats/languages than the emulated software.
    ****** Besides normal and bright(bold) fonts, try to make at least one other font, then use it when a character's attribute is different from most of the other characters on the screen.  This might help with some of those menus where you can't tell what you're selecting, like Windows 3.0 setup.exe (settings.exe?)
    ******** 
    ******** Before restoring from hibernate, check disk image files' dates to see if they're the same as when hibernating.  If different, abort un-hibernate and perform normal boot.  Otherwise disk images will be corrupted.
    ****** Create the option to format unformatted sd cards, in case the user has an unformatted sd card but no other computer around that can format it.
    ****** Need a way to partition and format unformatted or non-FAT16 USB drives
    ******* Disable lighting up capslock/numlock/scrolllock keyboard LEDs, and instead put these markers on screen in the far right 8 pixel empty column.  Will save a lot of power.
    ****** Ideally, I should be able to insert a new, blank, unformatted SD card, format it, create hard disk and floppy disk images on it, boot from read only disk image in SPIFFS, mount the disk images, use DOS's sys command to make them bootable, and then boot from them.
    ****** Fix screen not showing random jumbled characters at startup the way the original IBM PC should.
    ******* When I press a character key on the keyboard, I want to see the character print out on the screen as soon as possible, so I have optimized the display code to refresh ASAP after a keystroke.  But when it's not a character key, such as carriage return, refreshing as soon as possible causes the screen to refresh half way through the command prompt switching to the next line.  How to fix?  Check if character key or other key, and only respond so immeediately to character keys?
    ******* Ethernet port default setting should be to sleep n seconds (10? 15? 20? 60?) after last packet sent if no non-broadcast packets received in n seconds.  This default setting should be reconfigurable in ibox menu so that the ethernet port stays awake all the time, if user wants, such as when running a server.
    ******* Make the SPIFFS writable disk mountable from other emulated machines so that files can be easily copied between SPIFFS machine and sdcard machines
    ******* Need to run machine reset routine when rebooting from ctrl-alt-delete, otherwise lots of hardware peripherals don't get reset, which makes them malfunction after system reset.
    ******* For the different colored foreground text, I should try to make as many different fonts as possible, and use one for each different color
    ******* At any startup menu, if no input for several minutes, power off.
    ******* When set to power off after n seconds/minutes idle, make sure this setting is also implemented during ibox menu sessions.
    ******** If a drive image is used in machine A which then hibernates, and is also used in machine B, after machine A hibernates and then machine B changes drive image, then machine A unhibernates, drive image will be corrupted.  Need to check for this situation and ask user if okay to delete hibernation file that is "locking" the disk image before it can be used in any other virtual machine.  Or maybe it can be loaded as "read only" if the user so chooses?
    ****** Make eth port default sleep mode configurable (on or off) in iBox menu
    ******* Make eth port sleep parameters, such as how long after no send activity, how long after no recv activity, configurable in iBox menu.
    ******* A small design bug that affects the UX is that when networking mode is set to Ethernet, pins 23 is pulled low and then high in an effort to reset the ethernet port.  Leaving it pulled HIGH results in COM2's RTS pin being kept high, and this causes the RS232 pin 7 (RTS) to be kept at -5.5V, same as pin 3 (TX), so the mouse can't get it's 11V power it needs.  This happens even when the physical signal switch is set to COM2 instead of Ethernet.  
    ******     
    ****** Use or modify then use dosidle (https://github.com/galazwoj/dosidle/blob/master/DOSidle_251.asm) with a ESP32 sleep API via ISA I/O port to put ESP32 to light sleep when dosidle triggers cpu sleep, then wake it up again when dosidle's wake conditions are met.
    ****** Add a user configurable option to beep or click or some other noise immediately when starting to power on so that user gets some positive feedback
    ****** Add a LoRA chat client outside emulator in native mode (same as BASIC, text editor, e-reader, etc)

    * 
    * 
    */





    
    
    
    /* 
     *  There are some things that we love because they are weak and slow and make us wait.  Like Poloroid film.  Much of the fun is in waiting while
     *  the image slowly renders on the paper.
     *  
     *  What else?
     *  
     *  1. Old computers
     *  2. Complicated things that are just simple enough for one person to fully understand
     *  3. Mechanical clocks
     *  4. Locked boxes and clocks
     *  5. "Mousetrap" game style machines.  Like "The Incredible Machine" video game.  We can see it and understand how it works.
     */ 
//#pragma message "This sketch requires Tools->Partition Scheme = Huge APP"


#include <memory>
#include <time.h>
#include "soc/rtc.h"
//#include"Ap_29demo.h"
#include "esp32-hal-psram.h"
extern "C" {
#include "esp_spiram.h"
}
#include "esp_sntp.h"

#include <Preferences.h>
#include <WiFi.h>
#include "esp_private/wifi.h"
// https://gist.github.com/mic159/f546974d642f4189958423fbefe27902 see this as possible way to send/recv raw packets for ne2000 emulation passthrough
#include <HTTPClient.h>

#include "src/FabGL/src/fabgl.h"
fabgl::PS2Controller     PS2Controller;

#include "src/FabGL/src/inputbox.h"
#include "src/FabGL/src/fonts/font_sanserif_8x16.h"

#include "mconf.h"
#include "machine.h"
//#include <SD.h>

#include "bitmaps.h"

//wifi/bt stuff
#include "esp_bt_main.h"
#include "esp_bt.h"


// E-ink display stuff
// old GxEPD library includes 
//#include "src/GxEPD/src/GxEPD.h"
//#include "src/GxEPD/src/GxIO/GxIO_SPI/GxIO_SPI.h"
//#include "src/GxEPD/src/GxIO/GxIO.h"
//#include "src/GxEPD/src/GxGDEP015OC1/GxGDEP015OC1.h"    // 1.54" b/w
//#include "src/GxEPD/src/GxGDEH029A1/GxGDEH029A1.h" // 2.9" b/w
//#include "src/GxEPD/src/GxDEPG0750BN/GxDEPG0750BN.h" // 7.5" b/w

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0
// new GxEPD2 library includes
#include "src/GxEPD2/src/GxEPD2_BW.h"
#include "board_def.h"
//GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=D8*/ SS, /*DC=D3*/ 0, /*RST=D4*/ 2, /*BUSY=D2*/ 4)); // GDEH0154D67
GxEPD2_BW<GxEPD2_583_T8, GxEPD2_583_T8::HEIGHT> display(GxEPD2_583_T8(ELINK_SS, ELINK_DC, ELINK_RESET, ELINK_BUSY)); // GDEW0583T8
SPIClass * vspi = NULL;

#include "fonts/VGA8x16.h"
#include "fonts/VGA8x16Bold.h"
#include "fonts/BIOS8x8.h"

//#define DEFAULT_FONT Open_Sans_ExtraBold_12
#define DEFAULT_FONT VGA8x16
//#define DEFAULT_FONT BIOS8x8
const GFXfont *fonts[] = 
{
    //&Open_Sans_ExtraBold_12   // looks nice
    &VGA8x16,
    &BIOS8x8
};
#include "driver/spi_master.h"
//#include <SPI.h> //not needed with GxEPD2
//#include "Esp.h"

#include <NeoPixelBus.h>
const uint16_t PixelCount = 1; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 3;  // make sure to set this to the correct pin, ignored for Esp8266

#define colorSaturation 255

// four element pixels, RGBW
NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

// XMS Stuff
#define RAM_SIZE 0x10FFF0  //1114096 (1048576 + 65536) (1024 * 1024 + 65536)


// UART Pins for USB serial
#define UART_URX 3
#define UART_UTX 1
#define UART2_URX 36
#define UART2_UTX 22
#define UART2_CTS 34
#define UART2_RTS 23

// Networking mode stuff
#define NETWORKMODE_NONE 0
#define NETWORKMODE_WIFI 1
#define NETWORKMODE_ETH  2
int networkMode = NETWORKMODE_NONE; 
uint8_t ethMacAddr[6] = {0,0,0,0,0,0};

using std::unique_ptr;

using fabgl::StringList;
using fabgl::imin;
using fabgl::imax;



Preferences   preferences;
InputBox      ibox;
TaskHandle_t iboxVideoTaskHandle;
Machine     * machine;

bool colorDifferentiation = false;
extern bool need_fullscreen_refresh;
extern bool screenDirty;
extern uint32_t last_fullscreen_refresh;
extern bool waiting_for_epd_clean;
extern uint32_t last_screen_change_time;
extern uint16_t bg_color;
extern uint16_t fg_color;
extern uint32_t changedPixels;
extern uint32_t helpers;
extern TaskHandle_t videoTaskHandle;
extern TaskHandle_t m_taskHandle;
extern uint8_t * iBoxMemory;
bool videoTaskSuspended = false;
bool machineSuspended = false;
// custom Hercules resolutions:
extern uint16_t HGC_width;
extern uint16_t HGC_height;
extern int8_t HGC_scanlines;


// noinit! Used to maintain datetime between reboots
__NOINIT_ATTR static timeval savedTimeValue;


static bool wifiConnected = false;
static bool downloadOK    = false;

extern void PCapDataReceive(const unsigned char* data, unsigned int count);

static esp_err_t pkt_wifi2eth(void *buffer, uint16_t len, void *eb)
{
//    if (s_ethernet_is_connected) {
//        if (esp_eth_transmit(s_eth_handle, buffer, len) != ESP_OK) {
//            ESP_LOGE(TAG, "Ethernet send packet failed");
//        }
//    }
//    if(*(uint8_t*)(buffer) == 0x08 && *(uint8_t*)(buffer + 1) == 0xf9 && *(uint8_t*)(buffer + 2) == 0xe0)
//    {
//      printf("Got raw packet\n");
//      printf("Recv HEX: ");
//      for (int n = 0; n < len; n++)
//      {
//        printf("%02x ", *(char *)(buffer + n));
//      }
//      printf("\n");
//      printf("Recv DEC: ");
//      for (int n = 0; n < len; n++)
//      {
//        printf("%d ", *(unsigned char *)(buffer + n));
//      }
//      printf("\n");
//      printf("Recv ASCII: ");
//      for (int n = 0; n < len; n++)
//      {
//        if (*(char *)(buffer + n) >= 32 && *(char *)(buffer + n) < 127)
//        {
//          printf("%c", *(char *)(buffer + n));
//        }
//        else
//        {
//          printf(".");
//        }
//      }
//      printf("\n");
//    }
    PCapDataReceive((const unsigned char*)buffer, len);
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) 
  {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected to access point");
      esp_wifi_internal_reg_rxcb(WIFI_IF_STA, pkt_wifi2eth);
      break;
    default:
      break;  
  }
}

// try to connected using saved parameters
bool tryToConnect(bool setEventHandler = false)
{
  bool connected = WiFi.status() == WL_CONNECTED;
  if (!connected) 
  {
    char SSID[32] = "";
    char psw[32]  = "";
    if (preferences.getString("SSID", SSID, sizeof(SSID)) && preferences.getString("WiFiPsw", psw, sizeof(psw))) 
    {
      if (setEventHandler)
      {
        printf("setting Wifi.onEvent(WiFiEvent)\n");
        WiFi.onEvent(WiFiEvent);
      }
      ibox.progressBox("", "Abort", true, 200, [&](fabgl::ProgressForm * form) 
      {
        WiFi.begin(SSID, psw);
        for (int i = 0; i < 32 && WiFi.status() != WL_CONNECTED; ++i) 
        {
          if (!form->update(i * 100 / 32, "Connecting to %s...", SSID))
          {
            break;
          }
          delay(50);
          //delay(500);
//          if (i == 16)
//          {
//            WiFi.reconnect();
//          }
        }
        //connected = (WiFi.status() == WL_CONNECTED);
      });
      // show to user the connection state
      
      
//      if (!connected) 
//      {
//        if (!setEventHandler)
//        {
//          WiFi.disconnect();
//        }
//        ibox.message("", "WiFi Connection failed!");
//      }
//      else
//      {
//        Serial.print("Obtained IP address: ");
//        Serial.println(WiFi.localIP());
//        ibox.message("Obtained IP Address:", WiFi.localIP().toString().c_str());
//      }
    }
  }
  return connected;
}

bool checkWiFi()
{
//  ibox.setAutoOK(0);
//  char typingTest[64]  = "";
//  if (ibox.textInput("Typing Test", "Please type something:", typingTest, 63, "Cancel", "OK", true) == InputResult::Enter) 
//  {
//    ibox.message("", "Typing succeeded!");
//  }
//  else
//  {
//    ibox.message("", "Typing failed!");
//  }
//  return false;
//  int n = WiFi.scanNetworks();
//  Serial.println("Scan done");
//  if (n == 0) {
//      Serial.println("no networks found");
//  } else {
//      Serial.print(n);
//      Serial.println(" networks found");
//      Serial.println("Nr | SSID                             | RSSI | CH | Encryption");
//      for (int i = 0; i < n; ++i) {
//          // Print SSID and RSSI for each network found
//          Serial.printf("%2d",i + 1);
//          Serial.print(" | ");
//          Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
//          Serial.print(" | ");
//          Serial.printf("%4d", WiFi.RSSI(i));
//          Serial.print(" | ");
//          Serial.printf("%2d", WiFi.channel(i));
//          Serial.print(" | ");
//          switch (WiFi.encryptionType(i))
//          {
//          case WIFI_AUTH_OPEN:
//              Serial.print("open");
//              break;
//          case WIFI_AUTH_WEP:
//              Serial.print("WEP");
//              break;
//          case WIFI_AUTH_WPA_PSK:
//              Serial.print("WPA");
//              break;
//          case WIFI_AUTH_WPA2_PSK:
//              Serial.print("WPA2");
//              break;
//          case WIFI_AUTH_WPA_WPA2_PSK:
//              Serial.print("WPA+WPA2");
//              break;
//          case WIFI_AUTH_WPA2_ENTERPRISE:
//              Serial.print("WPA2-EAP");
//              break;
//          case WIFI_AUTH_WPA3_PSK:
//              Serial.print("WPA3");
//              break;
//          case WIFI_AUTH_WPA2_WPA3_PSK:
//              Serial.print("WPA2+WPA3");
//              break;
//          case WIFI_AUTH_WAPI_PSK:
//              Serial.print("WAPI");
//              break;
//          default:
//              Serial.print("unknown");
//          }
//          Serial.println();
//          delay(10);
//      }
//  }
//  Serial.println("");  

  wifiConnected =false;
  //wifiConnected = tryToConnect();
  //if (!wifiConnected) 
  {
    WiFi.disconnect();
    // configure WiFi?
    //ibox.setAutoOK(1, true);
    //if (ibox.message("WiFi Configuration", "Configure WiFi?", "No", "Yes") == InputResult::Enter) 
    //ButtonExt0
    //if (ibox.message("", "Press ENTER to configure WiFi") == InputResult::Enter) 
    {
      // repeat until connected or until user cancels
      //do 
      {
        // yes, scan for networks showing a progress dialog box
        int networksCount = 0;
        ibox.progressBox("", nullptr, false, 200, [&](fabgl::ProgressForm * form) 
        {
          form->update(0, "Scanning WiFi networks...");
          // int16_t scanNetworks(bool async = false, bool show_hidden = false, bool passive = false, uint32_t max_ms_per_chan = 300, uint8_t channel = 0);
          //networksCount = WiFi.scanNetworks(false, true, false, 1000, 0);
          networksCount = WiFi.scanNetworks();
          printf("networksCount = %d\n", networksCount);
        });

        // are there available WiFi?
        if (networksCount > 0)
        {
          // yes, show a selectable list
          StringList list;
          for (int i = 0; i < networksCount; ++i)
          {
            list.appendFmt("%s (%d dBm)", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
          }
          int s = ibox.menu("WiFi Configuration", "Please select a WiFi network", &list);

          // user selected something?
          if (s > -1) 
          {
            char SSID[32] = "";
            char psw[32]  = "";
            char psw2[32]  = "";
            preferences.getString("SSID", SSID, sizeof(SSID));
            preferences.getString("WiFiPsw", psw, sizeof(psw));
            if(WiFi.SSID(s) == String(SSID))
            {
              printf("setting psw2 = %s\n", psw);
              strcpy(psw2, psw);
            }
            // yes, ask for WiFi password
            //char psw[32] = "";
            ibox.setAutoOK(0);
            if (ibox.textInput("WiFi Configuration", "Enter WiFi password", psw2, 31, "Cancel", "OK", true) == InputResult::Enter) 
            {
              // user pressed OK, connect to WiFi...
              preferences.putString("SSID", WiFi.SSID(s).c_str());
              preferences.putString("WiFiPsw", psw2);
              wifiConnected = tryToConnect(true);
              // show to user the connection state
              if (wifiConnected)
              {
                ibox.message("", "Connection succeeded!");
              }
            }
            else
            {
              //break;
            }
          }
          else
          {
            //break;
          }
        }
        else 
        {
          // there is no WiFi
          ibox.message("", "No WiFi network found!");
          //break;
        }
        WiFi.scanDelete();

      }
      //while (!wifiConnected);
    }
//    else
//    {
//      WiFi.disconnect(true); 
//      WiFi.mode(WIFI_OFF); 
//      esp_wifi_stop(); 
//      btStop(); 
//      esp_bt_controller_disable();
//      esp_bluedroid_disable();
//    }
    ibox.setAutoOK(0);
  
  }
  return wifiConnected;
}




// handle soft restart
void shutdownHandler()
{
  // save current datetime into Preferences
  gettimeofday(&savedTimeValue, nullptr);
}


void updateDateTime()
{
  // Set timezone
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();

  // get datetime from savedTimeValue? (noinit section)
//  if (esp_reset_reason() == ESP_RST_SW)   // if reset reason was software reset using esp_restart()
//  {
//    // adjust time taking account elapsed time since ESP32 started
//    savedTimeValue.tv_usec += (int) esp_timer_get_time();
//    savedTimeValue.tv_sec  += savedTimeValue.tv_usec / 1000000;
//    savedTimeValue.tv_usec %= 1000000;
//    settimeofday(&savedTimeValue, nullptr);
//    return;
//  }

  checkWiFi();

//  if (checkWiFi()) 
//  {
//    // we need time right now
//    ibox.progressBox("", nullptr, true, 200, [&](fabgl::ProgressForm * form) 
//    {
//      sntp_setoperatingmode(SNTP_OPMODE_POLL);
//      sntp_setservername(0, (char*)"pool.ntp.org");
//      sntp_init();
//      for (int i = 0; i < 12 && sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED; ++i) 
//      {
//        form->update(i * 100 / 12, "Getting date-time from SNTP...");
//        delay(500);
//      }
//      sntp_stop();
//
//      savedTimeValue.tv_usec += (int) esp_timer_get_time();
//      savedTimeValue.tv_sec  += savedTimeValue.tv_usec / 1000000;
//      savedTimeValue.tv_usec %= 1000000;
//      settimeofday(&savedTimeValue, nullptr);
//      ibox.setAutoOK(2);
//      ibox.message("", "Date and Time updated.");
//      return;
//      
//      ibox.setAutoOK(2);
//      //ibox.message("", "Date and Time updated. Restarting...");
//      //esp_restart();
//    });
//
//  }
//  else 
//  {
//    // set default time
//    auto tm = (struct tm){ .tm_sec  = 0, .tm_min  = 0, .tm_hour = 8, .tm_mday = 14, .tm_mon  = 7, .tm_year = 84 };
//    auto now = (timeval){ .tv_sec = mktime(&tm) };
//    settimeofday(&now, nullptr);
//  }
}


// download specified filename from URL
bool downloadURL(char const * URL, FILE * file)
{
  Serial.println("inside downloadURL()");
  downloadOK = false;

  char const * filename = strrchr(URL, '/') + 1;
  ibox.progressBox("", "Abort", true, 380, [&](fabgl::ProgressForm * form) 
  {
    form->update(0, "Preparing to download %s", filename);
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.begin(URL);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) 
    {
      if (file) 
      {
        int tlen = http.getSize();
        int len = tlen;
        auto buf = (uint8_t*) SOC_EXTRAM_DATA_LOW; // use PSRAM as buffer
        WiFiClient * stream = http.getStreamPtr();
        int dsize = 0;
        while (http.connected() && (len > 0 || len == -1)) 
        {
          size_t size = stream->available();
          if (size) 
          {
            int c = stream->readBytes(buf, size);
            auto wr = fwrite(buf, 1, c, file);
            if (wr != c) 
            {
              dsize = 0;
              break;  // writing failure!
            }
            dsize += c;
            if (len > 0)
            {
              len -= c;
            }
            printf("read %d bytes, total %d of %d bytes\n", size, dsize, tlen);
            if (!form->update((int64_t)dsize * 100 / tlen, "Downloading %s (%.2f / %.2f MB)", filename, (double)dsize / 1048576.0, tlen / 1048576.0))
            {
              break;
            }
          }
        }
        downloadOK = (len == 0 || (len == -1 && dsize > 0));
      }
    }
    http.end();
  });

  return downloadOK;
}


int refreshes = 0;



//LUT download
//void lut(void)
//{
//  unsigned int count;  
//  {
//    display.epd2.writeCommand(0x20);             //vcom
//    for(count=0;count<44;count++)
//      {display.epd2.writeData(pgm_read_byte(&lut_vcom[count]));}
//    
//  display.epd2.writeCommand(0x21);             //red not use
//  for(count=0;count<42;count++)
//    {display.epd2.writeData(pgm_read_byte(&lut_ww[count]));}
//
//    display.epd2.writeCommand(0x22);             //bw r
//    for(count=0;count<42;count++)
//      {display.epd2.writeData(pgm_read_byte(&lut_bw[count]));}
//
//    display.epd2.writeCommand(0x23);             //wb w
//    for(count=0;count<42;count++)
//      {display.epd2.writeData(pgm_read_byte(&lut_wb[count]));}
//
//    display.epd2.writeCommand(0x24);             //bb b
//    for(count=0;count<42;count++)
//      {display.epd2.writeData(pgm_read_byte(&lut_bb[count]));}
//
//    display.epd2.writeCommand(0x25);             //vcom
//    for(count=0;count<42;count++)
//      {display.epd2.writeData(pgm_read_byte(&lut_ww[count]));}
//  }          
//}
//

//4 grayscale demo function
/********Color display description
      white  gray1  gray2  black
0x10|  01     01     00     00
0x13|  01     00     01     00
                                   ****************/

                                   
//void pic_display_4bit (uint32_t skip)
//{
//  //uint32_t skip = 88*6;
//  uint32_t i,j;
//  uint8_t temp1,temp2,temp3;
//
//    //old  data
//    display.epd2.writeCommand(0x10);        
//
//    for(i=0;i<4736;i++)                //4736*4  296*128 296*128=37888/2=18944=4736*4
//    //37288 pixels, 4 colors per pixel, each pixel 4 bits(half byte), so picture
//    //data size is 37888/2 = 18944
//    { 
//      temp3=0;
//      for(j=0;j<4;j++)  
//      {
//        temp1 = pgm_read_byte(&gImage_1[i*4+j]);
//        temp2 = temp1&0xF0 ;
//        if(temp2 == 0xF0)
//          temp3 |= 0x01;//white
//        else if(temp2 == 0x00)
//          temp3 |= 0x00;  //black
//        else if((temp2>0xA0)&&(temp2<0xF0)) 
//          temp3 |= 0x01;  //gray1
//        else 
//          temp3 |= 0x00; //gray2
//        temp3 <<= 1;  
//        temp1 <<= 4;
//        temp2 = temp1&0xF0 ;
//        if(temp2 == 0xF0)  //white
//          temp3 |= 0x01;
//        else if(temp2 == 0x00) //black
//          temp3 |= 0x00;
//        else if((temp2>0xA0)&&(temp2<0xF0))
//          temp3 |= 0x01; //gray1
//        else    
//          temp3 |= 0x00;  //gray2 
//        if(j!=3)          
//        temp3 <<= 1;  
//     }
//     display.epd2.writeData(temp3); 
//     if ((i) % 296 == 0)
//     {
//      for (int i = 0; i < skip; i++)
//      {
//        display.epd2.writeData(0x00);
//      }
//     }    
//    }
//    //new data
//    display.epd2.writeCommand(0x13);        
//
//    for(i=0;i<4736;i++)                //4736*4   152*152
//    { 
//      temp3=0;
//      for(j=0;j<4;j++)  
//      {
//        temp1 = pgm_read_byte(&gImage_1[i*4+j]);
//        temp2 = temp1&0xF0 ;
//        if(temp2 == 0xF0)
//          temp3 |= 0x01;//white
//        else if(temp2 == 0x00)
//          temp3 |= 0x00;  //black
//        else if((temp2>0xA0)&&(temp2<0xF0)) 
//          temp3 |= 0x00;  //gray1
//        else 
//          temp3 |= 0x01; //gray2
//        temp3 <<= 1;  
//        temp1 <<= 4;
//        temp2 = temp1&0xF0 ;
//        if(temp2 == 0xF0)  //white
//          temp3 |= 0x01;
//        else if(temp2 == 0x00) //black
//          temp3 |= 0x00;
//        else if((temp2>0xA0)&&(temp2<0xF0)) 
//          temp3 |= 0x00;//gray1
//        else    
//            temp3 |= 0x01;  //gray2
//        if(j!=3)        
//        temp3 <<= 1;        
//      
//     }  
//     display.epd2.writeData(temp3);
//     if ((i) % 296 == 0)
//     {
//        for (int i = 0; i < skip; i++)
//        {
//          display.epd2.writeData(0xff);
//        }
//     }           
//    }
//}














extern void lcd_chkstatus()
{
  uint32_t start = millis();
  while(!digitalRead(ELINK_BUSY));
  printf("busy %lums\n", millis() - start);
}

void EPD_sleep(void)
{
    display.epd2.writeCommand(0X50);  //VCOM AND DATA INTERVAL SETTING     
    display.epd2.writeData(0xf7); //WBmode:VBDF 17|D7 VBDW 97 VBDB 57    WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7  
    display.epd2.writeCommand(0X02);   //power off
    lcd_chkstatus();          //waiting for the electronic paper IC to release the idle signal
    display.epd2.writeCommand(0X07);   //deep sleep
    display.epd2.writeData(0xA5);
}

extern void EPD_W21_Init()
{
//  digitalWrite(RESET, LOW);
//  delay(10);//At least 10ms delay 
//  digitalWrite(RESET, HIGH);
//  delay(10);//At least 10ms delay 




    digitalWrite(ELINK_RESET, HIGH); // NEEDED for Waveshare "clever" reset circuit, power controller before reset pulse
    pinMode(ELINK_RESET, OUTPUT);
    delay(10); // NEEDED for Waveshare "clever" reset circuit, at least delay(2);
    digitalWrite(ELINK_RESET, LOW);
    delay(10);
    digitalWrite(ELINK_RESET, HIGH);
    delay(10);
}

extern void EPD_init()
{  
    EPD_W21_Init();     //Electronic paper IC reset 
    delay(100);         //reset IC and select BUS   

//something to do with grayscale testing
//    display.epd2.writeCommand(0x01);     //POWER SETTING
//    display.epd2.writeData (0x03);
//    display.epd2.writeData (0x00);      
//    display.epd2.writeData (0x2b);                                  
//    display.epd2.writeData (0x2b);   
//    display.epd2.writeData (0x13);
//
//    display.epd2.writeCommand(0x06);         //booster soft start
//    display.epd2.writeData (0x17);   //A
//    display.epd2.writeData (0x17);   //B
//    display.epd2.writeData (0x17);   //C 

    display.epd2.writeCommand(0x00);
    display.epd2.writeData(0x1F);
    
    display.epd2.writeCommand(0x04); //POWER ON
    delay(300);  
    lcd_chkstatus();
    display.epd2.writeCommand(0X50);     //VCOM AND DATA INTERVAL SETTING
    display.epd2.writeData(0x21);
    display.epd2.writeData(0x07);
}

extern void EPD_init_Fast(void)
{  
  EPD_W21_Init();     //Electronic paper IC reset 
  delay(100);         //reset IC and select BUS   
  display.epd2.writeCommand(0x00);
  display.epd2.writeData(0x1F); 
  display.epd2.writeCommand(0x04); //POWER ON
  delay(300);  
  lcd_chkstatus();
  display.epd2.writeCommand(0X50);     //VCOM AND DATA INTERVAL SETTING
  display.epd2.writeData(0x29);
  display.epd2.writeData(0x07);
  
  display.epd2.writeCommand(0xE0);
  display.epd2.writeData(0x02);
  display.epd2.writeCommand(0xE5);
  display.epd2.writeData(0x5A);
}

//extern void EPD_display_init(void)
//{  
//    EPD_W21_Init();   
//    delay(100);         //reset IC and select BUS 
//
//  display.epd2.writeCommand(0x01); // POWER SETTING
//  display.epd2.writeData (0x07);
//  display.epd2.writeData (0x07); // VGH=20V,VGL=-20V
//  display.epd2.writeData (0x3f); // VDH=15V
//  display.epd2.writeData (0x3f); // VDL=-15V
//  display.epd2.writeCommand(0x82); // VCOM DC Setting
//  display.epd2.writeData(0x24);  // VCOM
//  display.epd2.writeCommand(0x06); // Booster Setting
//  display.epd2.writeData(0x27);
//  display.epd2.writeData(0x27);
//  display.epd2.writeData(0x2F);
//  display.epd2.writeData(0x17);  
//
//  display.epd2.writeCommand(0x00);  //PANEL SETTING
//  display.epd2.writeData(0x1F); //KW: 3f, KWR: 2F, BWROTP: 0f, BWOTP: 1f
//
//  display.epd2.writeCommand(0x61); //tres
//  display.epd2.writeData (648 / 256); //source 648
//  display.epd2.writeData (648 % 256);
//  display.epd2.writeData (480 / 256); //gate 480
//  display.epd2.writeData (480 % 256);
//
//
//  display.epd2.writeCommand(0x15);
//  display.epd2.writeData(0x00);
//  display.epd2.writeCommand(0x50); //VCOM AND DATA INTERVAL SETTING
//  display.epd2.writeData(0x29);    // LUTKW, N2OCP: copy new to old
//  display.epd2.writeData(0x07);
//  display.epd2.writeCommand(0x60); //TCON SETTING
//  display.epd2.writeData(0x22);
//
//  display.epd2.writeCommand(0x65);
//  display.epd2.writeData(0x00);
//  display.epd2.writeData(0x00);  // 800*480
//  display.epd2.writeData(0x00);
//  display.epd2.writeData(0x00);  
// 
//  display.epd2.writeCommand(0x04); //POWER ON
//  delay(100);  
//  lcd_chkstatus();
//
////    display.epd2.writeCommand(0xE0);
////    display.epd2.writeData(0x02);
////    display.epd2.writeCommand(0xE5);
////    display.epd2.writeData(0x6E);
//}


extern void EPD_display_init(void)
{  
//  while(1)
//  {
//    delay(100);
//  } 
  EPD_W21_Init(); 
//  while(1)
//  {
//    delay(100);
//  }   
  delay(100);         //reset IC and select BUS 


// something for grayscale testing
//    display.epd2.writeCommand(0x01);     //POWER SETTING
//    display.epd2.writeData (0x03);
//    display.epd2.writeData (0x00);      
//    display.epd2.writeData (0x2b);                                  
//    display.epd2.writeData (0x2b);   
//    display.epd2.writeData (0x13);
//
//    display.epd2.writeCommand(0x06);         //booster soft start
//    display.epd2.writeData (0x17);   //A
//    display.epd2.writeData (0x17);   //B
//    display.epd2.writeData (0x17);   //C 

  display.epd2.writeCommand(0x06); // Booster Setting
  display.epd2.writeData(0x38);
  display.epd2.writeData(0x38);
  display.epd2.writeData(0x38);
  display.epd2.writeData(0x38); 

//  while(1)
//  {
//    delay(100);
//  }
  
//    display.epd2.writeData(0x3f);
//    display.epd2.writeData(0x3f);
//    display.epd2.writeData(0x3f);
//    display.epd2.writeData(0x3f);  

   
  display.epd2.writeCommand(0x00);  //PANEL SETTING
  display.epd2.writeData(0x1F); //KW: 3f, KWR: 2F, BWROTP: 0f, BWOTP: 1f

//  while(1)
//  {
//    delay(100);
//  }


  //delay(1000);
  //lcd_chkstatus();  //need this to prevent system shutoff?
//  printf("ready to send 0x04 command...\n");
//  delay(300);
//  printf("5\n");
//  delay(300);
//  printf("4\n");
//  delay(300);
//  printf("3\n");
//  delay(300);
//  printf("2\n");
//  delay(300);
//  printf("1\n");
//  delay(300);
//  printf("NOW!\n");
  display.epd2.writeCommand(0x04); //POWER ON   // this line seems to be causing system shutoff after a few more milliseconds.


//  while(1)
//  {
//    delay(100);
//  }
  
  delay(100);  
//  while(1)
//  {
//    delay(100);
//  }  
  lcd_chkstatus();
//  while(1)
//  {
//    delay(100);
//  } 
  display.epd2.writeCommand(0xE0);
  display.epd2.writeData(0x02);
  display.epd2.writeCommand(0xE5);
  display.epd2.writeData(0x6E);
}

//extern void EPD_display_init(void)
//{  
//    EPD_W21_Init();   
//    delay(100);         //reset IC and select BUS 
//
//
//// something for grayscale testing
////    display.epd2.writeCommand(0x01);     //POWER SETTING
////    display.epd2.writeData (0x03);
////    display.epd2.writeData (0x00);      
////    display.epd2.writeData (0x2b);                                  
////    display.epd2.writeData (0x2b);   
////    display.epd2.writeData (0x13);
////
////    display.epd2.writeCommand(0x06);         //booster soft start
////    display.epd2.writeData (0x17);   //A
////    display.epd2.writeData (0x17);   //B
////    display.epd2.writeData (0x17);   //C 
//
//    display.epd2.writeCommand(0x00);
//    display.epd2.writeData(0x1F);
//  
//    display.epd2.writeCommand(0x04); //POWER ON
//    delay(100);  
//    lcd_chkstatus();
//
//    display.epd2.writeCommand(0xE0);
//    display.epd2.writeData(0x02);
//    display.epd2.writeCommand(0xE5);
//    display.epd2.writeData(0x6E);
//}

extern void PIC_display_Clear(void)
{
  unsigned int i;
  //Write Data
  display.epd2.writeCommand(0x10);      
  for(i=0;i<38880;i++)       
  {
    display.epd2.writeData(0xff);  
  }
  display.epd2.writeCommand(0x13);      
  for(i=0;i<38880;i++)       
  {
    display.epd2.writeData(0xff);  
  }
  //Refresh
  display.epd2.writeCommand(0x12);   //DISPLAY REFRESH   
  delay(1);              //!!!The delay here is necessary, 200uS at least!!!     
  lcd_chkstatus();          //waiting for the electronic paper IC to release the idle signal
}

extern void refreshScreen()
{
  refreshes++;
  //EPD_display_init();
  if (refreshes >= 30)
  {
    //display.update(); // old GxEPD method
    //display.display(true); // new GxEPD2 method
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    refreshes = 0;
  }
  else
  {
    //display.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false, false); // old GxEPD method
    //display.display(true); // new GxEPD2 method
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
  }
  //display.powerDown(); //old GxEPD method
  //EPD_sleep();  // new GxEPD2 method
}

void displayInit()
{
  static bool isInit = false;
  if (isInit)
  {
    return;
  }
  isInit = true;
  
//  #define USEVSPI 1
//
//  #ifdef USEVSPI
//  #define SW_SCK 18
//  #define SW_MOSI 5
//  //display.epd2.init(SW_SCK, SW_MOSI, 115200, true, 10, false); // define or replace SW_SCK, SW_MOSI
//  #endif

  
  //display.init(115200);
  //initialize display (replaces display.init())
  //digitalWrite(ELINK_SS, HIGH);
  //pinMode(ELINK_SS, OUTPUT);
  digitalWrite(ELINK_DC, HIGH);
  pinMode(ELINK_DC, OUTPUT);
  //EPD_W21_Init();  // gpio pins to reset screen
  display.epd2.setParameters(115200, true, 10, false);
//  while(1)
//  {
//    delay(100);
//  }  
  EPD_init(); //EPD init
//  while(1)
//  {
//    delay(100);
//  }    
  PIC_display_Clear();//EPD Clear
//  while(1)
//  {
//    delay(100);
//  }    
  EPD_display_init();  //shutoff happens in this line
//  while(1)
//  {
//    delay(100);
//  }   
  // end display.init() replacement

//  //test 4 bit grayscale
//  for (int skip = 0; skip < 648; skip++)
//  {  
//    printf("skip = %d\n", skip);
//    pic_display_4bit(skip); //picture
//    lut(); //Power settings
//    display.epd2.writeCommand(0x12);     //DISPLAY REFRESH   
//    delay(100);      //!!!The delay here is necessary, 200uS at least!!!     
//    lcd_chkstatus();  
//  }

  
  //EPD_sleep();

        
  display.cp437(true);
  display.setTextWrap(false);
  //GxGDEH029A1_init();
  display.setRotation(0);
  //display.eraseDisplay(true);
  //display.eraseDisplay(false);
  
  display.setTextColor(GxEPD_BLACK, GxEPD_WHITE);
  //display.setTextColor(GxEPD_WHITE, GxEPD_BLACK);
  //display.fillScreen(GxEPD_BLACK);
  display.fillScreen(GxEPD_WHITE);
  //display.drawRect(0,0, 9 * 80 + 5, 16 * 25 + 5, GxEPD_BLACK);
  //display.setFont(&VGA8x16);
  //display.setTextSize(0);
  display.setCursor(0,0);
}











// return filename if successfully downloaded or already exist
char const * getDisk(char const * url)
{
  FileBrowser fb(SD_MOUNT_PATH);

  char const * filename = nullptr;
  if (url) 
  {
    if (strncmp("http://", url, 7) == 0 || strncmp("https://", url, 7) == 0) 
    {
      // this is actually an URL
      filename = strrchr(url, '/') + 1;
      if (filename && !fb.exists(filename, false)) 
      {
        printf("file %s doesn't exist\n", filename);
        // disk doesn't exist, try to download
        if (!checkWiFi())
        {
          Serial.println("checkWiFi() returned false");
          return nullptr;
        }
        auto file = fb.openFile(filename, "wb");
        Serial.println("calling downloadURL(url, file)");
        bool success = downloadURL(url, file);
        
        fclose(file);
        if (!success) 
        {
          fb.remove(filename);
          return nullptr;
        }
      }
    }
    else 
    {
      // this is just a file
      if (fb.filePathExists(url))
      {
        filename = url;
      }
    }
  }
  return filename;
}

// change power save mode via hotkey (ALT + CTRL + PRINTSCREEN (release CTRL first))
void sysReqCallback4()
{
  videoTaskSuspended = true;
  machineSuspended = true;
  char pwrmodes[][5]  = {"OFF", "FULL", "HALF"};
  //machine->powersaveEnabled = !machine->powersaveEnabled;
  machine->powersaveEnabled++;
  if(machine->powersaveEnabled > 2)
  {
    machine->powersaveEnabled = 0;
  }
  printf("Change power save mode to %s\n", pwrmodes[machine->powersaveEnabled]);
  display.setCursor(250,450);
  display.fillRect(250, 450, 150, 150, GxEPD_WHITE);
//  refreshScreen();
//  delay(200);
  display.setTextColor(GxEPD_BLACK);
  display.print("Power saving ");
  //display.print(machine->powersaveEnabled ? "ON" : "OFF");
  display.print(pwrmodes[machine->powersaveEnabled]);
  //refreshScreen();
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // partial update
  //delay(250);
  display.setTextColor(fg_color);
  //sysReqCallback();  // quick refresh    
  videoTaskSuspended = false;
  machineSuspended = false;
  return; 
}

// user pressed MENU KEY, invert fg/bg colors
void sysReqCallback3()
{
  //vTaskSuspend(videoTaskHandle); //doesn't work
  videoTaskSuspended = true;
  machineSuspended = true;
  //delay(50);
  //printf("Invert black/white fg/bg\n");
  machine->invertColors();
  //printf("redraw screen\n");
  printf("HGC_scanlines = %d\n", HGC_scanlines);
  printf("HGC_height = %d\n", HGC_height);
  printf("HGC_width = %d\n", HGC_width);
  machine->redrawScreen();
  //printf("draw borders\n");
  machine->drawBorders();
  //printf("done inverting\n"); 
  //display.displayStayOn(false);
  helpers = 0;
  //changedPixels = 0;
  
  
  //delay(500);
  
//  // quick refresh   
//  uint32_t startRefreshTime = millis();
//  printf("PIC_display_Clear()\n");
//  PIC_display_Clear();
//
//  printf("display.displayStayOn(false)\n");
//  display.displayStayOn(false);

  //printf("refresh used %lums\n", millis() - startRefreshTime);
  changedPixels = 9999999;
  screenDirty = true;
  //delay(1000);
  //vTaskResume(videoTaskHandle);
  //printf("done manually refreshing\n"); 
  need_fullscreen_refresh = false;
  last_fullscreen_refresh = millis();    //comment out for debugging, normally use   
  //waiting_for_epd_clean = true;
  last_screen_change_time = millis();
  helpers = 0; 
  need_fullscreen_refresh = false;
  last_fullscreen_refresh = millis();    //comment out for debugging, normally use   
  last_screen_change_time = millis(); 
  videoTaskSuspended = false;
  machineSuspended = false;
  //delay(10);
//  setCpuFrequencyMhz(10);  // valid values are 240, 160, 80, 40, 20, 10, 
//  delay(10000);
//  setCpuFrequencyMhz(240);
  //esp_sleep_enable_timer_wakeup(1000000); // light sleep for 5 seconds, then wake up again
  //gpio_wakeup_enable(GPIO_NUM_32, GPIO_INTR_LOW_LEVEL);
//  gpio_wakeup_enable(GPIO_NUM_33, GPIO_INTR_LOW_LEVEL);
//  esp_sleep_enable_gpio_wakeup();
//  ulp_run(0);
//  esp_light_sleep_start();
  
  
  //delay(10);
  //esp_intr_noniram_enable();   
//  pinMode(GPIO_NUM_33, INPUT);
//  pinMode(GPIO_NUM_32, INPUT);

  //esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  //PS2Controller::keyboard()->enableVirtualKeys(false, false); // don't use virtual keys
  //PS2Controller::begin(PS2Preset::KeyboardPort0, KbdMode::NoVirtualKeys);
  
  //PS2Controller::wakeup(GPIO_NUM_33, GPIO_NUM_32);
  //machine->resetKeyboard();
  
  //PS2Controller::enableRX(0);
  
  return; 
}

// user pressed SYSREQ (ALT + PRINTSCREEN) perform full screen clean and refresh, several seconds and flashing screen
void sysReqCallback2()
{
  printf("User initiated display.cleanWindow()\n");
  videoTaskSuspended = true;
  machineSuspended = true;



 // old way  
  
  
  printf("EPD_sleep()\n");
  EPD_sleep();
  
  printf("EPD_init()\n");
  EPD_init();

  printf("PIC_display_Clear()\n");
  PIC_display_Clear();

  printf("machine->redrawScreen()\n");
  machine->redrawScreen(); //temporarily disabled for debugging 20240714.  Disabling this solves the Hercules 640x300 line every third line black problem.

  printf("EPD_display_init()\n");
  EPD_display_init();

  printf("display.displayStayOn(false)\n");
  //display.display(false);

  // new way (cancelled)
  //epd_reset(1);
  //printf("machine->redrawScreen()\n");
  //machine->redrawScreen(); //temporarily disabled for debugging 20240714.  Disabling this solves the Hercules 640x300 line every third line black problem.
  
  display.displayStayOn(false);
  helpers = 0;
  changedPixels = 0;

  printf("done manually refreshing\n");     
  need_fullscreen_refresh = false;
  last_fullscreen_refresh = millis();    //comment out for debugging, normally use   
  //waiting_for_epd_clean = true;
  last_screen_change_time = millis();  
  videoTaskSuspended = false;
  machineSuspended = false;
  return; 
}

// user pressed "windows" key, quick full screen refresh, no flasing
void sysReqCallback()
{
//  printf("User initiated manual screen refresh\n");
//
//
//
//  //old slow way
//    uint32_t startRefreshTime = millis();
//    printf("PIC_display_Clear()\n");
//    PIC_display_Clear();
// 
//    printf("display.displayStayOn(false)\n");
//    display.displayStayOn(false);
//    
//    printf("refresh used %lums\n", millis() - startRefreshTime);
//    changedPixels = 0;
//  
//    //printf("done manually refreshing\n"); 
//    need_fullscreen_refresh = false;
//    last_fullscreen_refresh = millis();    //comment out for debugging, normally use   
//    //waiting_for_epd_clean = true;
//    last_screen_change_time = millis();
//    helpers = 0;



 

  videoTaskSuspended = true;
  machineSuspended = true;
  xTaskCreate(
    iboxVideoTask, /* Function to implement the task */
    "iboxVideoTask", /* Name of the task */
    //1024 * 2,  /* Stack size in words */
    1024 * 2,  /* Stack size in words */
    //this,  /* Task input parameter */
    NULL, /* Task input parameter */
    1,  /* Priority of the task */
    &iboxVideoTaskHandle  /* Task handle. */
  //   ,1  /* Core where the task should run,whichever one isn't running Machine::runTask() */ 
  ); 
  ibox.begin(648, 480, 2);
  
  ibox.setBackgroundColor(RGB888(255, 255, 255));  // WHITE bg
  ibox.onPaint = [&](Canvas * canvas) 
  {
    screenDirty = true;
  };

  //int s = ibox.menu("", "Select a command", "Restart Emulator;Restart Machine;Mount Disk;Continue");
  //int s = ibox.menu("", "Select a command", "Mount Disk;Continue;Restart Emulator;Restart Machine");
  // need to add unmount disk feature, otherwise its stuck in drive and can't reboot to hard drive.
  int s = ibox.menu("", "System Menu", "Mount Disk;Sound Settings;System Speed;Continue;Restart Machine;Restart Emulator;Hibernate;Power Off;Color Emulation");
  switch (s) 
  {
    // Mount Disk
    case 0:
    {
      int s = ibox.menu("", "Select Drive", "Floppy A (fd0);Floppy B (fd1)");
      if (s > -1) 
      {
        constexpr int MAXNAMELEN = 256;
        unique_ptr<char[]> dir(new char[MAXNAMELEN + 1] { '/', 'S', 'D', 0 } );
        //unique_ptr<char[]> dir(new char[MAXNAMELEN + 1]  { 0 } );
        unique_ptr<char[]> filename(new char[MAXNAMELEN + 1] { 0 } );
        if (machine->diskFilename(s))
        {
          strcpy(filename.get(), machine->diskFilename(s));
          //printf("%s\n", machine->diskFilename(s));
        }
        if (ibox.fileSelector("Select Disk Image", "Image Filename", dir.get(), MAXNAMELEN, filename.get(), MAXNAMELEN) == InputResult::Enter) 
        {
          //edit->setTextFmt("%s/%s", dir.get() + 4, filename.get()); // bypass "/SD/"
          char * filepath = dir.get() + 4; // bypass "/SD", but keep now leading "/"
          strcat(filepath, "/");
          strcat(filepath, filename.get());
          
          //strcpy(filename.get(), dir.get());
          printf("mounting %s\n", filepath);
          machine->setDriveImage(s, filepath);
        }
      }
      break;
    }
    
    case 1:  //manage sound output devices
    {
      int c = ibox.menu("", "Select Action", "Enable PC Spkr;Disable PC Spkr;Enable Adlib;Disable Adlib;Enable MIDI;Disable MIDI;Enable Disney;Disable Disney;Enable Covox;Disable Covox;Enable Sound;Disable Sound");
      switch(c)
      {
        //pc speaker
        case 0:
        {
          machine->m_soundGen.attach(&machine->m_squareWaveGen);
          machine->m_squareWaveGen.enable(true);
          break;
        }
        case 1:
        {
          machine->m_soundGen.detach(&machine->m_squareWaveGen);
          machine->m_squareWaveGen.enable(false);
          break;
        }

        //adlib
        case 2:
        {
          machine->m_soundGen.attach(&machine->m_adlibSoundGenerator);
          machine->m_adlibSoundGenerator.enable(true);
          break;
        }
        case 3:
        {
          machine->m_soundGen.detach(&machine->m_adlibSoundGenerator);
          machine->m_adlibSoundGenerator.enable(false);
          break;
        }

        //midi
        case 4:
        {
          machine->m_soundGen.attach(&machine->m_midiSoundGenerator);
          machine->m_midiSoundGenerator.enable(true);
          break;
        }
        case 5:
        {
          machine->m_soundGen.detach(&machine->m_midiSoundGenerator);
          machine->m_midiSoundGenerator.enable(false);
          break;
        }

        
        //disney
        case 6:
        {
          machine->m_soundGen.attach(&machine->m_disneySoundGenerator);
          machine->m_disneySoundGenerator.enable(true);
          break;
        }
        case 7:
        {
          machine->m_soundGen.detach(&machine->m_disneySoundGenerator);
          machine->m_disneySoundGenerator.enable(false);
          break;
        }

        /*
        //covox
        case 8:
        {
          machine->m_soundGen.attach(&machine->m_covoxSoundGenerator);
          machine->m_covoxSoundGenerator.enable(true);
          break;
        }
        case 9:
        {
          machine->m_soundGen.detach(&machine->m_covoxSoundGenerator);
          machine->m_covoxSoundGenerator.enable(false);
          break;
        }
        */
        
        //whole sound generator on/off
        case 10:
        {
          machine->m_soundGen.play(true);
          break;
        }
        case 11:
        {
          machine->m_soundGen.play(false);
          break;
        }
      }
      break;
    }
    
    case 2:
    {
      int c = ibox.menu("", "Select Speed (MHz)", "No Change;10;20;40;80;160;240");
      switch(c)
      {
          case 1:
            Serial.end();
            delay(500);
            setCpuFrequencyMhz(10);
            delay(500);
            Serial.begin(115200);
            delay(2000);
            break;
          case 2:
            Serial.end();
            delay(500);
            setCpuFrequencyMhz(20);
            delay(500);
            Serial.begin(115200);
            delay(2000);
            break;
          case 3:
            Serial.end();
            delay(500);
            setCpuFrequencyMhz(40);
            delay(500);
            Serial.begin(115200);
            delay(2000);
            break;
          case 4:
            Serial.end();
            setCpuFrequencyMhz(80);
            Serial.begin(115200);
            delay(1000);
            break;
          case 5:
            Serial.end();
            setCpuFrequencyMhz(160);
            Serial.begin(115200);
            delay(500);
            break;
          case 6:
            Serial.end();
            setCpuFrequencyMhz(240);
            Serial.begin(115200);
            delay(500);
            break;
          default:
            break;
      }
      break;
    }
    // "continue" just breaks, so just use "default"
    // Restart Emulator
        // Restart Machine
    case 4:  // restart emulated machine (like ctrl-alt-delete)
      machine->trigReset();
      break;    
    case 5:  // restart emulator (restart esp32)
//      for (int y = 0; y < 480; y++)
//      {
//        for (int x = 0; x < 648; x++)
//        {
//          display.drawPixel(x, y, GxEPD_WHITE);
//        }
//      }
//      lcd_chkstatus();
//      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
//      lcd_chkstatus();
      memset(machine->s_videoMemory, 0, VIDEOMEMSIZE);
      esp_restart();
      break;

    case 6: //hibernate
      machine->hibernate();
      break;

    case 7: // power off
      machine->powerOff();
      break;

    case 8: // toggle colorDifferentiation
      colorDifferentiation = !colorDifferentiation;
      break;

    // Continue
    default:
      break;
  }
  delay(500);
  ibox.end();
  delay(200);
  vTaskDelete(iboxVideoTaskHandle);
  delay(200);
  videoTaskSuspended = false;
  delay(200);
  //vTaskResume(m_taskHandle);
  //vTaskResume(videoTaskHandle);
  PS2Controller::keyboard()->enableVirtualKeys(false, false); // don't use virtual keys
  delay(200);
  //machine->graphicsAdapter()->enableVideo(true);  



  machine->redrawScreen();
  screenDirty = true;
  delay(200);
  machineSuspended = false;

//  InputBox ib;
//  //ib.begin(VGA_640x480_60Hz, 640, 480, 2);
//  ib.begin(640, 480, 2);
//  //ib.setBackgroundColor(RGB888(0, 0, 0));
//  ib.setBackgroundColor(RGB888(255, 255, 255));
//  ib.onPaint = [&](Canvas * canvas) 
//  {
//    canvas->selectFont(&fabgl::FONT_SANSERIF_8x16);
//    //canvas->setPenColor(RGB888(255, 255, 0));
//    canvas->setPenColor(RGB888(0, 0, 0));
//    canvas->drawText(85, 5, "InputBox Demo - www.fabgl.com - by Fabrizio Di Vittorio");
//    
//    //display.fillRect(0,0,100,100, GxEPD_BLACK);
//    //printf("paint event\n");
//    lcd_chkstatus();
//    //if(digitalRead(ELINK_BUSY))
//    {
//      display.displayFast(0, 0, 648, 480);
//    }
//  };
//  // setup automatic OK after 10 seconds
//  //ib.setAutoOK(10);
//
//  ib.message("Welcome!", "Welcome to FabGL InputBox demo!");
//  
//  ////////////////////////////////////////////////////
//  // Example of progress bar
//  InputResult r = ib.progressBox("Example of Progress Bar", "Abort", true, 200, [&](fabgl::ProgressForm * form) {
//    for (int i = 0; i <= 100; ++i) {
//      if (!form->update(i, "Index is %d/100", i))
//        break;
//        delay(40);
//    }
//    delay(800);
//  });
//  if (r == InputResult::Cancel)
//  {
//    ib.message("", "Operation Aborted");
//  }
//
//
//  ////////////////////////////////////////////////////
//  // Example of simple menu (items from separated strings)
//  int s = ib.menu("Simple Menu", "Click on one item", "Item number zero;Item number one;Item number two;Item number three");
//  ib.messageFmt("", nullptr, "OK", "You have selected item %d", s);
//
//  ////////////////////////////////////////////////////
//  // Example of simple menu (items from StringList)
//  fabgl::StringList list;
//  list.append("Option Zero");
//  list.append("Option One");
//  list.append("Option Two");
//  list.append("Option Three");
//  list.select(1, true);
//  s = ib.menu("Menu", "Click on an item", &list);
//  ib.messageFmt("", nullptr, "OK", "You have selected item %d", s);
//  //display.displayFast(0, 0, 648, 480);
//
//  ////////////////////////////////////////////////////
//  // Example of options selection box with OK button (items from separated strings)
//  list.clear();
//  list.append("Item 0");
//  list.append("Item 1");
//  list.append("Item 2");
//  for (bool loop = true; loop; ) 
//  {
//    ib.setupButton(0, "Add");
//    ib.setupButton(1, "Remove");
//    ib.setupButton(2, "Options", "Edit;Restore;Advanced", 50);
//    ib.setMinButtonsWidth(60);
//    auto r = ib.select("Items selection", "Select an item", &list, "Cancel", "OK");
//    switch (r) {
//      // OK button
//      case InputResult::Enter:
//        ib.messageFmt(nullptr, nullptr, "OK", "You have selected item %d", list.getFirstSelected());
//        loop = false;
//        break;
//      // add new item button
//      case InputResult::ButtonExt0:
//      {
//        char value[32] = "";
//        if (ib.textInput("New Item", "Please enter item value", value, 31) == InputResult::Enter) {
//          list.takeStrings();
//          list.append(value);
//        }
//        break;
//      }
//      // remove item button
//      case InputResult::ButtonExt1:
//        if (list.getFirstSelected() > -1 && ib.message("Please confirm", "Remove Item?", "No", "Yes") == InputResult::Enter)
//          list.remove(list.getFirstSelected());
//        break;
//      // button with subitems (Edit / Restore / Advanced)
//      case InputResult::ButtonExt2:
//        switch (ib.selectedSubItem()) {
//          // Edit sub-button
//          case 0:
//            ib.message(nullptr, "Edit - not implented!");
//            break;
//          // Restore
//          case 1:
//            ib.message(nullptr, "Restore - not implented!");
//            break;
//          // Advanced
//          case 2:
//            ib.message(nullptr, "Advanced - not implented!");
//            break;
//        }
//        break;
//      // cancel
//      default:
//        loop = false;
//        break;
//    }
//  }

}

static void iboxVideoTask(void* pvParameters)
{
  while(1)
  {
    //printf("sc\n");
    if(screenDirty && digitalRead(ELINK_BUSY))
    {
      //vTaskDelay(100); // 85 is long enough to be perfect
      vTaskDelay(85); // 57 is too short. 65 long enough to feel good but not perfect.
      screenDirty = false;
      //printf("df\n");
      lcd_chkstatus();
      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // full screen partial update
      //printf("@\n");      
    }
    vTaskDelay(10);
  }
}

extern void epd_reset(int times)
{
  PIC_display_Clear();
  for (int n = 0; n < times; n++)
  {
    // clear to black
    for (int y = 0; y < 480; y++)
    {
      for (int x = 0; x < 648; x++)
      {
        display.drawPixel(x, y, GxEPD_BLACK);
      }
    }
    lcd_chkstatus();
    //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    display.displayStayOn(false);
    //delay(100);
  
    // clear to white
    for (int y = 0; y < 480; y++)
    {
      for (int x = 0; x < 648; x++)
      {
        display.drawPixel(x, y, GxEPD_WHITE);
      }
    }
    lcd_chkstatus();
    //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    display.displayStayOn(false);
    //delay(100);
  }  
}



void setup()
{

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
//  pinMode(21, OUTPUT);
//  digitalWrite(21, HIGH);

  //tone(25, 917, 50); 
  tone(25, 917, 100); 
  Serial.begin(115200); 
  //Serial.begin(1000000); 

  delay(10); 
  printf("\n\n\nStartup\n\n");

  /* GPIO 21 LED test */
  //pinMode(21, OUTPUT);
  //digitalWrite(21, HIGH);
  /* END GPIO 21 LED test */
  
/*
  strip.Begin();
  for (int n = 0; n < 5; n++)
  {
    RgbColor color(64, 0, 0);   //red
    strip.SetPixelColor(0, color);
    strip.Show();
    delay(100);

    RgbColor color2 (0, 64, 0);   //green
    strip.SetPixelColor(0, color2);
    strip.Show();
    delay(100);
  }
  RgbColor color(0, 0, 0);   //off
  strip.SetPixelColor(0, color);
  strip.Show();
*/

// test lora configuration / deep sleep mode:
//  Serial1.begin(9600, SERIAL_8N1, 39, 0);
//  Serial1.write(0xC1);
//  Serial1.write(0x00);
//  Serial1.write(0x03);
//  Serial1.flush();
//  delay(100);
//  printf("Sent query to LORA adapter,waiting for response...\n");
//  while(!Serial1.available());
//  while(Serial1.available())
//  {
//    printf("0x%02X\n", Serial1.read()); 
//  }
//  printf("No more bytes from LORA adapter\n");
//  delay(999999);





  //disable wifi and BT
  WiFi.disconnect(true); 
  WiFi.mode(WIFI_OFF); 
  esp_wifi_stop(); 
  btStop(); 
  esp_bt_controller_disable();
  esp_bluedroid_disable();
  
  //delay(100);
  
  //delay(9999999);
  disableCore0WDT();
  //delay(100); // experienced crashes without this delay!
  disableCore1WDT();







  pinMode(ELINK_BUSY, INPUT);  //must set this before using e-ink display!

  // we need PSRAM for this system, but we will handle it manually, so please DO NOT enable PSRAM on your development env
  #ifdef BOARD_HAS_PSRAM
  /*
  ibox.message("Warning!", "Please disable PSRAM to improve performance!");
  */
  #endif

  // note: we use just 2MB (not so of Evertop) of PSRAM so the infamous PSRAM bug should not happen. But to avoid gcc compiler hack (-mfix-esp32-psram-cache-issue)
  // we enable PSRAM at runtime, otherwise the hack slows down CPU too much (PSRAM_HACK is no more required).
  if (esp_spiram_init() != ESP_OK)
  {
    /*
    ibox.message("Error!", "This system requires a board with PSRAM!", nullptr, nullptr);
    */
  }

  #ifndef BOARD_HAS_PSRAM
  esp_spiram_init_cache();
  #endif

  // configure and start epd display
  vspi = new SPIClass(VSPI);
  vspi->begin(EPD_SCLK, -1, EPD_MOSI, EPD_CS); //SCLK, MISO, MOSI, SS
  
  display.epd2.selectSPI(*vspi, SPISettings(26600000, MSBFIRST, SPI_MODE0));  // max 26.6Mhz unless can enter half_duplex mode and get 40Mhz
  displayInit();

  //epd_reset(1);
  //PS2Controller::begin(PS2Preset::KeyboardPort0_MousePort1, KbdMode::NoVirtualKeys);
//  display.setCursor(260, 236); //center next line on screen
  display.setFont(&VGA8x16);
//  display.print("SYSTEM STARTING");
  display.setCursor(0, 0);
  display.print("Evertop Freedom PC\n");
  display.print("Espressif ESP32 240MHz\n");
  display.print("8388KB PSRAM OK\n\n");
  display.print("WAIT...");

  display.drawXBitmap(416, 0, solarIcon, 232, 150, GxEPD_BLACK);
  
  lcd_chkstatus();
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);

  
  PS2Controller::begin(PS2Preset::KeyboardPort0_MousePort1, KbdMode::NoVirtualKeys);

  display.setCursor(0,464); //bottom left corner
  display.print("Press "); 
  display.setFont(&VGA8x16Bold); 
  display.print("DEL ");
  display.setFont(&VGA8x16);
  display.print("to enter ");
  display.setFont(&VGA8x16Bold);
  display.print("SETUP");
  display.setFont(&VGA8x16);

  lcd_chkstatus();
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);

//  tone(25, 917, 50); 

  /* Test keyboard input */
  //PS2Controller.begin(PS2Preset::KeyboardPort0);
  //PS2Controller::begin(PS2Preset::KeyboardPort0, KbdMode::CreateVirtualKeysQueue);
  //PS2Controller::begin(PS2Preset::KeyboardPort0_MousePort1, KbdMode::GenerateVirtualKeys);
  
  //PS2Controller::begin(PS2Preset::KeyboardPort0, KbdMode::NoVirtualKeys);
  
  
  
  //auto keyboard = PS2Controller.keyboard();
  //keyboard->suspendVirtualKeyGeneration(false);
  
  uint32_t startwait = millis();
  bool enterSetup = false;
  static int clen = 1;
  while(millis() - startwait < 2000)
  {
//    if(PS2Controller::keyboard()->virtualKeyAvailable())
//    {
//      // ascii mode (show ASCII and VirtualKeys)
//      bool down;
//      auto vk = PS2Controller::keyboard()->getNextVirtualKey(&down);
//      printf("VirtualKey = %s", PS2Controller::keyboard()->virtualKeyToString(vk));
//      int c = PS2Controller::keyboard()->virtualKeyToASCII(vk);
//      if (c > -1) 
//      {
//        printf("\tASCII = 0x%02X\t", c);
//        if (c >= ' ')
//        {
//          printf("%c", c);
//        }
//      }
//      if (!down)
//      {
//        printf("\tUP");
//      }
//      printf("\r\n");
//      //lastvk = down ? vk : fabgl::VK_NONE;
//      if (strcmp(PS2Controller::keyboard()->virtualKeyToString(vk), "VK_DELETE") == 0)
//      {
//        enterSetup = true;
//        break;
//      }
//    }

    if (PS2Controller::keyboard()->scancodeAvailable()) 
    {
      int scode = PS2Controller::keyboard()->getNextScancode();
      printf("%02X ", scode);
      if (scode == 0xF0 || scode == 0xE0) ++clen;
      --clen;
      if (clen == 0) 
      {
        clen = 1;
        printf("\r\n");
      }
      if (scode == 0x71)
      {
        enterSetup = true;
        break;
      }
    }


    delay(50); 
  }
  //PS2Controller::keyboard()->enableVirtualKeys(false, false); // don't use virtual keys
  //PS2Controller::begin(PS2Preset::KeyboardPort0, KbdMode::NoVirtualKeys);
  //PS2Controller::end();
  //PS2Controller::begin(PS2Preset::KeyboardPort0_MousePort1, KbdMode::GenerateVirtualKeys);
  //PS2Controller::keyboard()->enableVirtualKeys(true, true);
  printf("enterSetup = %d\n", enterSetup);

  
  
  /* End test keyboard input */
  //refreshScreen();

  
 /*
  // 4-button controller keypad test with WS2811 LED test
  //strip.Begin();
  
  
  display.setCursor(0, 0); //center next line on screen
  int lastRead33 = 1;
  int lastRead32 = 1;
  int lastRead26 = 1;
  int lastRead27 = 1;

  pinMode(33, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);


  printf("Press buttons 1-4 to test\n");
  display.print("Button test: press up, down, left, right buttons to test... \n");
  lcd_chkstatus();
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
  int count = 0;
  while(1)
  {
    int r33 = digitalRead(33);
    int r32 = digitalRead(32);
    int r26 = digitalRead(26);
    int r27 = digitalRead(27);
    
    if (r33 != lastRead33)
    {
      //RgbColor color(64, 0, 0);   //red
      //strip.SetPixelColor(0, color);
      //strip.Show(); 
      printf("33->%d\n", r33);
      display.print("33");
      display.print(r33);
      display.print(" ");
      lcd_chkstatus();
      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
      count++;
      

    }
  
    if (r32 != lastRead32)
    {
//      RgbColor color(0, 64, 0);   //green
//      strip.SetPixelColor(0, color);
//      strip.Show();
      printf("32->%d\n", r32);
      display.print("32");
      display.print(r32);
      display.print(" ");
      lcd_chkstatus();
      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
      count++;

 
    }
  
    if (r27 != lastRead27)
    {
//      RgbColor color(0, 0, 64);   //blue
//      strip.SetPixelColor(0, color);
//      strip.Show(); 
      printf("27->%d\n", r27);
      display.print("27");
      display.print(r27);
      display.print(" ");
      lcd_chkstatus();
      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
      count++;

            
    }
  
    if (r26 != lastRead26)
    {
//      RgbColor color(32, 0, 32);   //purple
//      strip.SetPixelColor(0, color);
//      strip.Show(); 
      printf("26->%d\n", r26);
      display.print("26");
      display.print(r26);
      display.print(" ");
      lcd_chkstatus();
      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
      count++;

      
    }
    
    lastRead33= r33;
    lastRead32= r32;
    lastRead26= r26;
    lastRead27= r27;

    
    if (count >= 20)
    {
      display.print("\n");
      lcd_chkstatus();
      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
      count = 0;
    }
    delay(10);
  }
  // end 4 button with ws2811 LED test
 */

  printf("iBoxMemory = %p\n", iBoxMemory);
  iBoxMemory = (uint8_t*)(SOC_EXTRAM_DATA_LOW + 2 * 1024 * 1024) + RAM_SIZE + 65536;
  printf("iBoxMemory = %p\n", iBoxMemory);

  if(enterSetup)
  {
    xTaskCreate(
        iboxVideoTask, /* Function to implement the task */
        "iboxVideoTask", /* Name of the task */
        //1024 * 2,  /* Stack size in words */
        1024 * 2,  /* Stack size in words */
        //this,  /* Task input parameter */
        NULL, /* Task input parameter */
        1,  /* Priority of the task */
        &iboxVideoTaskHandle  /* Task handle. */
     //   ,1  /* Core where the task should run,whichever one isn't running Machine::runTask() */ 
     ); 
  
    ibox.begin(648, 480, 2, false);
    ibox.setBackgroundColor(RGB888(255, 255, 255));
  
    ibox.onPaint = [&](Canvas * canvas) 
    {
      screenDirty = true;
    };
  }
  

  preferences.begin("PCEmulator", false);

  //pinMode(ELINK_BUSY, INPUT);

  //wake from light sleep on keypress (keyboard clock actually)
  gpio_wakeup_enable(GPIO_NUM_33, GPIO_INTR_LOW_LEVEL);

  //wake up on key signal (rather than clock) as well:
  gpio_wakeup_enable(GPIO_NUM_32, GPIO_INTR_LOW_LEVEL);


  // and as well for mouse:
  gpio_wakeup_enable(GPIO_NUM_26, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_27, GPIO_INTR_LOW_LEVEL);
  
  esp_sleep_enable_gpio_wakeup();

  bool sdOkay = true;
  if (!FileBrowser::mountSDCard(false, SD_MOUNT_PATH, 8, 16 * 1024, SDCARD_MISO, SDCARD_MOSI, SDCARD_SCLK, SDCARD_SS))   // @TODO: reduce to 4?
  {
    printf("Error!  This system requires an SD CARD!\n");
    sdOkay = false;
    /*
    ibox.message("Error!", "This system requires an SD-CARD!", nullptr, nullptr);
    */
  }
  else
  {
    printf("FileBrowser::mountSDCard succeeded\n");
  }
  printf("SDMMC_FREQ_DEFAULT = %d\n", SDMMC_FREQ_DEFAULT);

  esp_register_shutdown_handler(shutdownHandler);

  char compileTime[20];
  sprintf(compileTime, "%s%s", __DATE__, __TIME__);
  printf("compileTime: >%s<\n", compileTime);
  //printf("date: >%s< time: >%s<\n", __DATE__, __TIME__);

  // check if this is the first run on a new build.  If so, delete all hibernation (.hib) files, otherwise unhibernating on a new firmware version, especially a new BIOS verion, easily ruins disk images!
  char savedBuildTime[20];
  size_t ret = preferences.getString("buildTime", savedBuildTime, 20);
  //printf("ret = %d\n", ret);
  int diff = strcmp(savedBuildTime, compileTime);
  
  printf("savedBuildTime = >%s<\n", savedBuildTime);


  if(sdOkay && (ret == 0 || diff != 0)) // if SD card mounted, and (two strings don't match or if no previous value has been saved)
  {
    // delete all the *.hib hibernation files
    printf("NEW BUILD DETECTED!  DELETING ALL HIBERNATION FILES!\n");
    for (int n = 0; n < 100; n++)
    {
      int deleted = unlink(String("/SD/" + String(n) + ".hib").c_str());
      
      if (deleted == 0)
      {
        printf("Deleted /SD/%d.hib to prevent disk image corruption after firmware update\n", n);
      }
    }

    // save compiledTime to savedBuildTime preferences record for future comparisons.
    ret = preferences.putString("buildTime", compileTime);
    //printf("putString returned %d\n", ret);
  }
//  // we need PSRAM for this system, but we will handle it manually, so please DO NOT enable PSRAM on your development env
//  #ifdef BOARD_HAS_PSRAM
//  /*
//  ibox.message("Warning!", "Please disable PSRAM to improve performance!");
//  */
//  #endif
//
//  // note: we use just 2MB of PSRAM so the infamous PSRAM bug should not happen. But to avoid gcc compiler hack (-mfix-esp32-psram-cache-issue)
//  // we enable PSRAM at runtime, otherwise the hack slows down CPU too much (PSRAM_HACK is no more required).
//  if (esp_spiram_init() != ESP_OK)
//  {
//    /*
//    ibox.message("Error!", "This system requires a board with PSRAM!", nullptr, nullptr);
//    */
//  }
//
//  #ifndef BOARD_HAS_PSRAM
//  esp_spiram_init_cache();
//  #endif
  
  // uncomment to format SD!
  //FileBrowser::format(fabgl::DriveType::SDCard, 0);
  
  //esp_register_shutdown_handler(shutdownHandler);

//  // configure and start epd display
//  vspi = new SPIClass(VSPI);
//  vspi->begin(EPD_SCLK, -1, EPD_MOSI, EPD_CS); //SCLK, MISO, MOSI, SS
//
//
//  
//  display.epd2.selectSPI(*vspi, SPISettings(26000000, MSBFIRST, SPI_MODE0));  // max 26Mhz unless can enter half_duplex mode and get 40Mhz
//  //display.epd2.selectSPI(SPI, SPISettings(40000000, MSBFIRST, SPI_MODE0)); 
//  displayInit();
//  //display.fillScreen(GxEPD_WHITE);
//  display.setCursor(0,0);
//
////  display.print("Starting");
////  refreshScreen();
////  delay(2000);
////  
//
//
//
//
//
//  printf("iBoxMemory = %p\n", iBoxMemory);
//  iBoxMemory = (uint8_t*)(SOC_EXTRAM_DATA_LOW + 2 * 1024 * 1024) + RAM_SIZE + 65536;
//  printf("iBoxMemory = %p\n", iBoxMemory);
//
//  //xTaskCreatePinnedToCore(
//  xTaskCreate(
//      iboxVideoTask, /* Function to implement the task */
//      "iboxVideoTask", /* Name of the task */
//      //1024 * 2,  /* Stack size in words */
//      1024 * 2,  /* Stack size in words */
//      //this,  /* Task input parameter */
//      NULL, /* Task input parameter */
//      1,  /* Priority of the task */
//      &iboxVideoTaskHandle  /* Task handle. */
//   //   ,1  /* Core where the task should run,whichever one isn't running Machine::runTask() */ 
//   ); 
//
//  
//  ibox.begin(648, 480, 2);
//  ibox.setBackgroundColor(RGB888(255, 255, 255));
//
//  ibox.onPaint = [&](Canvas * canvas) 
//  {
//    //drawInfo(canvas); 
//    screenDirty = true;
//    //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // partial update
//    //refreshScreen();
//    //printf(".\n");
//  };


  
  if (!sdOkay)
  {
    for (int y = 0; y < 480; y++)
    {
      for (int x = 0; x < 648; x++)
      {
        display.drawPixel(x, y, GxEPD_WHITE);
      }
    }
    display.setCursor(232, 236); //center next line on screen
    display.print("SD card not found!");
    display.setCursor(260, 272); //center two lines down on screen
    display.print("POWERING OFF");
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    //delay(1000);
    printf("Powering off power circuit because missing sd card!\n");
    delay(1);
    lcd_chkstatus();
    //display.powerOff();
    delay(10);
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    delay(500);
    printf("POWER OFF FAILED, ENTERING DEEP SLEEP MODE INSTEAD!\n");
    
    display.setCursor(8, 288); //center two lines down on screen
    display.print("POWER OFF FAILED, ENTERING DEEP SLEEP MODE INSTEAD. PLEASE POWER OFF MANUALLY!");
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    delay(1);
    lcd_chkstatus();
    display.powerOff();
    delay(10);
    
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    delay(100);
    esp_deep_sleep_start();
  }

//  if (enterSetup)
//  {
//    updateDateTime();
//  }
  
  //refreshScreen();

  //disable wifi and BT
//  esp_bluedroid_disable();
//  esp_bt_controller_disable();
//  esp_wifi_stop();  // commented out because it breaks  Machine::redrawScreen() line: printCharAttr(ascii, attribute, n / 2); for some weird reason

  // machine configurations
  MachineConf mconf;

  if (enterSetup)
  {
    // show a list of machine configurations
    //ibox.setAutoOK(6);
    ibox.setAutoOK(0);
  }
  
  int idx = preferences.getInt("dconf", 0);
  printf("previous idx = %d\n", idx);
  bool need_unhibernate = false;
  bool testmode = false;


  //PS2Controller::begin(PS2Preset::KeyboardPort0_MousePort1, KbdMode::CreateVirtualKeysQueue);
  //loadMachineConfiguration(&mconf);
  if (enterSetup)
  {
    for (bool showDialog = true; showDialog; ) 
    {
  
      loadMachineConfiguration(&mconf);
  
      StringList dconfs;
      for (auto conf = mconf.getFirstItem(); conf; conf = conf->next)
      {
        dconfs.append(conf->desc);
      }
      dconfs.select(idx, true);
  
      ibox.setupButton(0, "Files");
      ibox.setupButton(1, "Machine", "Edit;New;Remove", 52);
      ibox.setupButton(2, "Networking", "Wifi;Ethernet;None", 52);
      ibox.setupButton(3, "Power Saving", "Half;Max;None",52);
      auto r = ibox.select("Machine Configurations", "Please select a machine configuration", &dconfs, nullptr, "Run");
  
      idx = dconfs.getFirstSelected();
  
      switch (r) 
      {
        case InputResult::ButtonExt0:
          // Browse Files
          ibox.folderBrowser("Browse Files", SD_MOUNT_PATH);
          break;
        case InputResult::ButtonExt1:
          // Machine
          switch (ibox.selectedSubItem()) 
          {
            // Edit
            case 0:
              editConfigDialog(&ibox, &mconf, idx);
              break;
            // New
            case 1:
              newConfigDialog(&ibox, &mconf, idx);
              break;
            // Remove
            case 2:
              delConfigDialog(&ibox, &mconf, idx);
              break;
          };
          break;
        case InputResult::ButtonExt2:
          //ibox.message("Notice", "You clicked the Networking button!");
          // Networking options
          switch (ibox.selectedSubItem()) 
          {
            // Wifi
            case 0:
              {
                //ibox.message("Notice", "You selected Wifi!");
                preferences.putString("Network", "Wifi");
                ibox.setAutoOK(0);
                InputResult res = ibox.message("Wifi networking", "Connect to Wifi network now?", "No", "Yes", true);
                if (res == InputResult::Enter)
                {
                  updateDateTime();
                }
              }
              break;
            // Ethernet
            case 1:
              ibox.message("Notice", "Remember to turn off COM2 and turn on ETH.");
              ibox.message("WARNING!!", "Ethernet uses a massive 100-150mA! Only use if you really need to, or if you can afford it!");
              preferences.putString("Network", "Ethernet");
              break;
            // No networking
            case 2:
              ibox.message("Notice", "No networking will be available.");
              preferences.putString("Network", "None");
              break;
          };
          break;
        case InputResult::ButtonExt3:
          // Power saving options:
          switch (ibox.selectedSubItem()) 
          {
            // Half
            case 0:
              //ibox.message("Notice", "You selected Half!");
              preferences.putInt("PowerSaving", 2);
              ibox.setAutoOK(0);
              ibox.message("Power Saving", "Default power saving mode set to HALF.");
              break;
            // Max
            case 1:
              //ibox.message("Notice", "You selected Max!");
              preferences.putInt("PowerSaving", 1);
              ibox.setAutoOK(0);
              ibox.message("Power Saving", "Default power saving mode set to MAX.");
              break;
            // None
            case 2:
              //ibox.message("Notice", "You selected None!");
              preferences.putInt("PowerSaving", 0);
              ibox.setAutoOK(0);
              ibox.message("Power Saving", "Default power saving mode set to NONE.");
              ibox.message("Warning", "This will reduce battery life to at most 250 hours of active use!");
              break;
          };
          break;
          
        case InputResult::Enter:
          // Run
          showDialog = false;
          break;
        default:
          break;
      }
  
      // next selection will not have timeout
      ibox.setAutoOK(0);
    }
  }
  else
  {
    loadMachineConfiguration(&mconf);
  }


  
  idx = imax(idx, 0);
  preferences.putInt("dconf", idx);

  printf(String("checking for /" + String(idx) + ".hib\n").c_str());

  auto file = FileBrowser(SD_MOUNT_PATH).openFile(String("/" + String(idx) + ".hib").c_str(), "r");

  if (file)
  {
    //display.fillScreen(GxEPD_WHITE);
    for (int y = 0; y < 480; y++)
    {
      for (int x = 0; x < 648; x++)
      {
        display.drawPixel(x, y, GxEPD_WHITE);
      }
    } 
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    display.setCursor(200, 224);
    display.setFont(&VGA8x16);
    display.print("Press ESC to skip unhibernate");
    //display.display(true); 
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); 
    need_unhibernate = true;
    int keycode = 0xffff;
    uint32_t startwait = millis();
    
    while(millis() - startwait < 3000)
    {
//      if(PS2Controller::keyboard()->virtualKeyAvailable())
//      {
//        VirtualKey pressedkey = PS2Controller::keyboard()->getNextVirtualKey();
//        keycode = PS2Controller::keyboard()->virtualKeyToASCII(pressedkey);
//        printf("keycode = %d\n", keycode);
//        if (keycode == 27)
//        {
//          need_unhibernate = false;
//          printf("Unhibernate cancelled.  Removing /%d.hib\n", idx);
//          FileBrowser(SD_MOUNT_PATH).remove(String("/" + String(idx) + ".hib").c_str());
//          display.setCursor(200, 244);
//          display.print("Unhibernate canceled       ");   
//          display.display(true);           
//        }
//      }
      if (PS2Controller::keyboard()->scancodeAvailable()) 
      {
        int scode = PS2Controller::keyboard()->getNextScancode();
        printf("%02X ", scode);
        if (scode == 0xF0 || scode == 0xE0) ++clen;
        --clen;
        if (clen == 0) 
        {
          clen = 1;
          printf("\r\n");
        }
        if (scode == 0x76)
        {
          need_unhibernate = false;
          printf("Unhibernate cancelled.  Removing /%d.hib\n", idx);
          FileBrowser(SD_MOUNT_PATH).remove(String("/" + String(idx) + ".hib").c_str());
          display.setCursor(216, 256);
          display.print("Unhibernate canceled       ");   
          //display.display(true); 
          lcd_chkstatus();
          display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
          delay(1000);
          break;
        }
      }
    }
  }

  fclose(file);  

  if (need_unhibernate)
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(200, 224);
    display.setFont(&VGA8x16);
    display.print("Unhibernating...             ");
    //display.display(true);  
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
  }
  // setup selected configuration
  printf("idx = %d\n", idx);
  auto conf = mconf.getItem(idx);
  char const * diskFilename[DISKCOUNT];
  downloadOK = true;
  for (int i = 0; i < DISKCOUNT && downloadOK; ++i)
  {
    printf("i = %d\n", i);
    diskFilename[i] = getDisk(conf->disk[i]);
    if (diskFilename[i])
    {
      printf("diskFilename[%d] = %s\n", i, diskFilename[i]);
    }
  }

  if (!downloadOK || (!diskFilename[0] && !diskFilename[2])) 
  {
    // unable to get boot disks
    /*
    ibox.message("Error!", "Unable to get system disks!");
    */
    printf("Error!  Unable to get system disk(s)!\n");
    //ibox.message("Error!", "Unable to get system disks!");
    display.setCursor(212, 236);
    display.print("Unable to get system disks(s)");
    display.setCursor(244, 252);
    display.print("Check machine settings");
    display.setCursor(268, 268);
    display.print("POWERING OFF...");
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    //lcd_chkstatus();
    //display.powerOff();
    delay(10);
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    delay(500);

    
    display.setCursor(8, 300); //center two lines down on screen
    display.print("POWER OFF FAILED, ENTERING DEEP SLEEP MODE INSTEAD. PLEASE POWER OFF MANUALLY!");
    lcd_chkstatus();
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    delay(1);
    lcd_chkstatus();
    display.powerOff();
    delay(10);
    
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    delay(100);
    esp_deep_sleep_start();
  
    //delay(2000);
    //esp_restart();
    
  }
  if (enterSetup)
  {
    ibox.end();
    screenDirty = false;
    delay(200);
    vTaskDelete(iboxVideoTaskHandle);
    delay(200);
  }
  for (int y = 0; y < 480; y++)
  {
    for (int x = 0; x < 648; x++)
    {
      display.drawPixel(x, y, GxEPD_WHITE);
    }
  } 
  lcd_chkstatus();
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
  lcd_chkstatus();
  delay(200);
//  display.fillScreen(GxEPD_WHITE);
//  refreshScreen();
//  display.displayStayOn(false);
  display.setFont(&VGA8x16);
//  if (!need_unhibernate)
//  {
//    display.setCursor(0,0);
//    display.println("Starting Emulator...");
//    refreshScreen();
//    display.displayStayOn(false);
//  }
  // without WiFi it is possible to increase SD card speed
//  FileBrowser::setSDCardMaxFreqKHz(SDMMC_FREQ_DEFAULT);
//  FileBrowser::remountSDCard();
  machine = new Machine;
  machine->setBaseDirectory(SD_MOUNT_PATH);
//  printf("Disk images:\n");
  for (int i = 0; i < DISKCOUNT; ++i)
  {
    machine->setDriveImage(i, diskFilename[i], conf->cylinders[i], conf->heads[i], conf->sectors[i]);
    if (diskFilename[i])
    {
      printf("%d. %s\n", i, diskFilename[i]);
    }                             
                        
  } 
                     
  machine->setBootDrive(conf->bootDrive);
  // see what networking mode to use.  If Ethernet mode, do not set up COM2

  char strNetworkMode[9] = "";
  preferences.getString("Network", strNetworkMode, sizeof(strNetworkMode));
  printf("preferences.getString(\"Network\") returned \"%s\"\n", strNetworkMode);
  if (strcmp(strNetworkMode, "Ethernet") == 0)
  {
    networkMode = NETWORKMODE_ETH;
  }
  if (strcmp(strNetworkMode, "Wifi") == 0)
  {
    networkMode = NETWORKMODE_WIFI;
  }
  printf("networkMode = %d\n", networkMode);
  if(networkMode != NETWORKMODE_ETH)
  {
    /*
     * I should make COM1 be an option at system startup.  Default to debug mode, select for COM1 mode
     */
    //  auto serial1 = new SerialPort;
    //  serial1->setSignals(UART_URX, UART_UTX);
    //  machine->setCOM1(serial1);
    auto serial2 = new SerialPort;
    serial2->setSignals(UART2_URX, UART2_UTX, UART2_RTS, UART2_CTS);
    machine->setCOM2(serial2);  
  
    //  auto serial3 = new FakeSerialPort;
    //  serial3->setSignals();
    //  machine->setCOM3(serial3);
  }

//  if(networkMode == NETWORKMODE_ETH)
//  {
//    Serial2.begin(9600, SERIAL_8N1, UART2_URX, UART2_UTX);
//    printf("executed Serial2.begin(9600, SERIAL_8N1, %d, %d)\n", UART2_URX, UART2_UTX);
//    pinMode(34, INPUT_PULLUP); // receive interrupt signals from ethernet port
//    // now we should be able to communicate with ethernet port via Serial2
//
//    // do some R&D testing:
//    // send test command and data byte, return byte should be bitwise NOT of data byte
//    //57 AB 06 55; 
//    Serial2.write(0x57);
//    Serial2.write(0xAB);
//    Serial2.write(0x06); // CMD_CHECK_EXIST command
//    Serial2.write(0x55); // 55 is data byte
//    printf("Sent test byte sequence 57  AB  06  55 to ethernet port\n");
//    delay(10);
//    while(!Serial2.available())
//    {
//      delay(10);
//    }
//    uint8_t tmpval = Serial2.read();
//    printf("CMD_CHECK_EXIST received 0x%02X from ethernet port!\n", tmpval);  // if received 0xAA, proves commo is working
//
////  SEND CMD_INIT_CH395 command
//    printf("Sending CMD_INIT_CH395 command\n");
//    // 57 AB 27
//    Serial2.write(0x57); // command prefix
//    Serial2.write(0xAB); // command prefix
//    Serial2.write(0x27); // CMD_INIT_CH395
//    delay(700); // datasheet says 350ms, try twice as much to be safe
//    // 57 AB 2C
//    Serial2.write(0x57); // command prefix
//    Serial2.write(0xAB); // command prefix
//    Serial2.write(0x2C); // CMD_GET_CMD_STATUS    
//    while(!Serial2.available())
//    {
//      delay(10);
//    }
//    tmpval = Serial2.read();
//    printf("CMD_GET_CMD_STATUS received 0x%02X from ethernet port!\n", tmpval);  
//
//    // GET MAC ADDRESS
//    printf("getting MAC address\n");
//    // 57 AB 40
//    //uint8_t macbuf[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//    Serial2.write(0x57); // command prefix
//    Serial2.write(0xAB); // command prefix
//    Serial2.write(0x40); // CMD_GET_MAC_ADDR
//    while(!Serial2.available())
//    {
//      delay(10);
//    }
//    int count = 0;
//    while(Serial2.available())
//    {
//      tmpval = Serial2.read();
//      printf("received: 0x%02X\n", tmpval);
//      ethMacAddr[count] = tmpval;
//      delay(5);
//      count++;
//    }
//    
//    printf("Setting ethernet port to MAC RAW mode\n");
//    // set ethernet port to MAC RAW working mode
//    // 57 AB 34 00 01
//    Serial2.write(0x57); // command prefix
//    Serial2.write(0xAB); // command prefix
//    Serial2.write(0x34); // CMD_SET_PROTO_TYPE_SN
//    Serial2.write(0x00); // Socket 0x00
//    Serial2.write(0x01); // mode 0x01 (MAC RAW)
//
//    // send CMD_OPEN_SOCKET_SN command to open socket 0x01
//    // 57 AB 35 00
//    Serial2.write(0x57); // command prefix
//    Serial2.write(0xAB); // command prefix
//    Serial2.write(0x35); // CMD_OPEN_SOCKET_SN
//    Serial2.write(0x00); // Socket 0x00
//
//    delay(100);
//
//    // send CMD_GET_CMD_STATUS to get result of operation
//    // 57 AB 2C
//    Serial2.write(0x57); // command prefix
//    Serial2.write(0xAB); // command prefix
//    Serial2.write(0x2C); // CMD_GET_CMD_STATUS
//    delay(10);
//    while(!Serial2.available())
//    {
//      delay(10);
//    }
//    tmpval = Serial2.read();
//    printf("CMD_GET_CMD_STATUS received 0x%02X from ethernet port!\n", tmpval); 
//
//    
//    // send data for testing with wireshark:
//    // send CMD_WRITE_SEND_BUF_SN
//    // 57 AB 39 00 64 00 FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 
//    //while(1)
//    {
//      delay(1000);
//      Serial2.write(0x57); // command prefix
//      Serial2.write(0xAB); // command prefix
//      Serial2.write(0x39); // CMD_WRITE_SEND_BUF_SN
//      Serial2.write(0x00); // Socket 0x00
//      Serial2.write(0xEA); // data length low byte
//      Serial2.write(0x05); // data length high byte
//      // dst address is 54 b2 03 f1 08 e2
//      Serial2.write(0x54);
//      Serial2.write(0xB2);
//      Serial2.write(0x03);
//      Serial2.write(0xF1);
//      Serial2.write(0x08);
//      Serial2.write(0xE2);
//      // src address is 84 c2 e4 ea a5 f5
//      Serial2.write(0x84);
//      Serial2.write(0xC2);
//      Serial2.write(0xE4);
//      Serial2.write(0xEA);
//      Serial2.write(0xA5);
//      Serial2.write(0xF5);
//      for (int n = 0; n < 0x05DE; n++)
//      {
//        Serial2.write((uint8_t)n);
//      }
//      Serial2.flush();
//      delay(1000);
//      
//      // 0x30 = CMD_GET_INT_STATUS_SN
//      Serial2.write(0x57); // command prefix
//      Serial2.write(0xAB); // command prefix
//      Serial2.write(0x30); // CMD_GET_INT_STATUS_SN
//      Serial2.write(0x00); // socket number 0x00
//      delay(10);
//      while(!Serial2.available())
//      {
//        delay(10);
//      }
//      tmpval = Serial2.read();
//      printf("After sending TEST packet, CMD_GET_INT_STATUS_SN received 0x%02X from ethernet port!\n", tmpval); 
//
//      
//    }
//
//    
//    
//    
//    
//    
//  }
//  
  /*
  printf("MALLOC_CAP_32BIT : %d bytes (largest %d bytes)\r\n", heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  printf("MALLOC_CAP_8BIT  : %d bytes (largest %d bytes)\r\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  printf("MALLOC_CAP_DMA   : %d bytes (largest %d bytes)\r\n\n", heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

  heap_caps_dump_all();
  */
  int powerSavingMode = preferences.getInt("PowerSaving", 2); //default to half power saving if no default saved yet.
  machine->powersaveEnabled = (uint8_t)(powerSavingMode & 0xff);
  printf("powerSavingMode set to %d\n", machine->powersaveEnabled);
  machine->setSysReqCallback(sysReqCallback);
  machine->setSysReqCallback2(sysReqCallback2);
  machine->setSysReqCallback3(sysReqCallback3);
  machine->setSysReqCallback4(sysReqCallback4);
  machine->machine_idx = idx;
  if (need_unhibernate)
  {
    //printf("setting machine->unhibernate = true\n");
    machine->need_unhibernate = true;
  }
  if (testmode)
  {
    //printf("setting machine->testmode = true\n");
    machine->testmode = true;
  }
  machine->run();
}


void loop()
{
  printf("vTaskDelete(NULL)\n");
  vTaskDelete(NULL);
  //vTaskDelay(1000);
}
