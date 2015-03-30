// pololu maestro servo control

//  To use addressed commands, transmit 0xAA as the first (command) byte, followed by a Device Number data byte. 
//  Any controller on the line whose device number matches the specified device number accepts the command that follows.
//  all other Pololu devices ignore the command.

//  To command all devices, use the "compact command" which simply does not have the above stuff.  Note that
//  this format doesn't work for commands that require a response, as all the devices would respond at once
//  and interfere with each other.

// A quirky feature of the interface is that command bytes always have their most significant bit (not byte) set,
// while data bytes almost always have their most significant bit cleared.

// For example, when using the compact command format, the command might be 0x85 (move forward)
// and the data bytes might be 0 and 50 (at 50% speed).  Like this: 0x85 0 50
// However, when using the addressed command, the 0x85 is
// not used, instead 0x05 is used for move forward.  That's because it is considered a data byte of the 0xAA command.
// So to get device 14 to go at 50% speed, the bytes would be : 0xAA 14 0x05 0 50
// The above is a poor example, because it suggests that the entire most significant BYTE be set to 0 for data
// but really, it is just the most significant BIT


