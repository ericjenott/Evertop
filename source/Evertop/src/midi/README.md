# ESP32Synth

* 20+ voice polyphonic Wavetable synthesizer for ESP32 with 5110 display as status display
* Uses exponential ADSR for each voice
* Has 4 different waveforms: Square, Saw, Sine and Triangle
* Width of square wave can be adjusted as well
* Receives MIDI events over Serial
* Outputs audio at 40.000 samples per second at the built in 8-bit DAC
* Controlled completely over MIDI control commands, so no hardware is needed for potentiometers and such
* Has many cool display pages like a live ADSR graph, status display of all voices, graphical wavetable selection and even a properly working audio oscilloscope
* Menu pages are switched by using the pich bend, and the modulation wheel is usually used to change values. ADSR values are changed using
* Designed for a Acorn Masterkey 61 (with 4 control knobs), but can be easily modified to work with other MIDI keyboards (or even MIDI over the Serial USB from a PC)
* Practically no latency!
