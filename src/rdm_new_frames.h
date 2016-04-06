/*
 * rdm_new_frames.h
 *
 *  Created on: 06/04/2016
 *      Author: Mariano
 */

#ifndef RDM_NEW_FRAMES_H_
#define RDM_NEW_FRAMES_H_



// This is the definition for a unique DEVICE ID.
// DEVICEID[0..1] ESTA Manufacturer ID
// DEVICEID[2..5] unique number
typedef uint8_t DEVICEID[6];


#define DMXSERIAL_MAX_RDM_STRING_LENGTH 6

// ----- structures -----

// The RDMDATA structure (length = 24+data) is used by all GET/SET RDM commands.
// The maximum permitted data length according to the spec is 231 bytes.
typedef struct {
	uint8_t		StartCode;    // Start Code 0xCC for RDM
	uint8_t		SubStartCode; // Start Code 0x01 for RDM
	uint8_t		Length;       // packet length
	uint8_t		DestID[6];
	uint8_t		ourceID[6];

	uint8_t		_TransNo;     // transaction number, not checked
	uint8_t		ResponseType;    // ResponseType
	uint8_t		_unknown;     // I don't know, ignore this
	uint16_t	SubDev;      // sub device number (root = 0)
	uint8_t		CmdClass;     // command class
	uint16_t	Parameter;	   // parameter ID
	uint8_t		DataLength;   // parameter data length in bytes
	uint8_t		Data[231];   // data byte field
} RDMDATA; // struct RDMDATA

// ----- Library Class -----

// These types are used to pass all the data into the initRDM function.
// The library needs this data to reposonse at the corresponding commands for itself.

typedef struct  {
  uint16_t footprint;
  // maybe more here... when supporting more personalitites.
}RDMPERSONALITY; // struct RDMPERSONALITY


typedef struct  {
  char          *manufacturerLabel; //
  const uint16_t          deviceModelId;       //
  char          *deviceModel;       //
  uint16_t footprint;
  // uint16_t personalityCount;
  // RDMPERSONALITY *personalities;
  const uint16_t        additionalCommandsLength;
  const uint16_t       *additionalCommands;
} RDMINIT; // struct RDMINIT


// compare 2 DeviceIDs
#define DeviceIDCmp(id1, id2) memcmp(id1, id2, sizeof(DEVICEID))

// copy an DeviceID id2 to id1
#define DeviceIDCpy(id1, id2) memcpy(id1, id2, sizeof(DEVICEID))

// ----- RDM specific members -----

   // Return true when identify mode was set on by controller.
   bool isIdentifyMode(void);

   // Returns the Device ID. Copies the UID to the buffer passed through the uid argument.
   void getDeviceID(DEVICEID id);

   // Return the current DMX start address that is the first dmx address used by the device.
   uint16_t getStartAddress(void);

   // Return the current DMX footprint, that is the number of dmx addresses used by the device.
   uint16_t getFootprint(void);

   // Register a device-specific implemented function for RDM callbacks
   //void    attachRDMCallback (RDMCallbackFunction newFunction);

   // check for unprocessed RDM Command.
   void    tick(void);

   // Terminate operation.
   void    term(void);

   // A short custom label given to the device.
   char deviceLabel[DMXSERIAL_MAX_RDM_STRING_LENGTH];

#endif /* RDM_NEW_FRAMES_H_ */
