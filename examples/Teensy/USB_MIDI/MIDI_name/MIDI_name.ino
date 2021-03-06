/* USB MIDI Custom Name Example
 *  
 * This example demonstrates how to change the USB MIDI 
 * device name on Teensy LC and 3.x.  When creating more
 * that one MIDI device, custom names are much easier to
 * use when selecting each device in MIDI software on
 * your PC or Mac.  The custom name is in the "name.c" tab.
 * 
 * You must select MIDI from the "Tools > USB Type" menu
 * 
 * This example code is in the public domain.
 */

void setup() {
}

void loop() {
  // Add your MIDI application here...

  // MIDI Controllers should discard incoming MIDI messages.
  // http://forum.pjrc.com/threads/24179-Teensy-3-Ableton-Analog-CC-causes-midi-crash
  while (usbMIDI.read()) {
    // ignore incoming messages
  }
}

