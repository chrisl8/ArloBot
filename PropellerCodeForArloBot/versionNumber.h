// =============================================== //
// * Version Number *
// This is a simple, 8 bit integer
// to define the version number for the sole
// purpose of ensuring that the ROS Node Python
// code and the code on the Propeller board match.
// Increment this number each time you edit code
// for the Propeller board to ensure end users are
// warned to update it upon starting ROS.

// Do NOT increase the number above 254,
// just loop it back to 1 if we get more than
// 255 changes, and we will rely on time and chance
// to avoid collisions.
//
// NEVER set this to 0 or 256. Those are reserved
// for error checking.
// Use 1 to 254 and restart at 1 if you need to.
// =============================================== //

#define PROPELLER_CODE_VERSION_NUMBER 2
