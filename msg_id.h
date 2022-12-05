#pragma once

namespace xsens {

// https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf
// Section 8.5
enum class MsgId {
  kReqDID = 0x00,  // Host request device ID of the device
  kDeviceID = 0x01,  // Device acknowledges request by sending its ID
  kReserved0 = 0x02,  // Reserved
  kReserved1 = 0x03,  // Reserved
  kReqPeriod = 0x04,  // Request current sampling period
  kSetPeriod = 0x04,  // Host sets sampling period (10-500Hz)
  kReqPeriodAck = 0x05,  // Device returns sampling period
  kSetPeriodAck = 0x05,  // Device acknowledges SetPeriod message
  kReserved2 = 0x0A,  // Reserved
  kReserved3 = 0x0B,  // Reserved
  kReqConfiguration = 0x0C,  // Request the configuration of device. Forlogging/quick setup purposes
  kConfiguration = 0x0D,  // Contains the configuration of device
  kRestoreFactoryDef = 0x0E,  // Restores all settings in MT to factory defaults
  kGoToMeasurement = 0x10,
  kGoToMeasurementAck = 0x11,
  kReqFWRev = 0x12,  // Host requests firmware revision of device
  kFirmwareRev = 0x13,  // Device acknowledges request by sending itsfirmware revision
  kReqBaudrate = 0x18,  // Requests current baud rate of the serialcommunication
  kSetBaudrate = 0x18,  // Host sets baud rate of serial communication
  kReqBaudrateAck = 0x19,  // Device returns baud rate of serialcommunication
  kSetBaudrateAck = 0x19,  // Device acknowledges SetBaudrate message
  kReqProductCode = 0x1C,  // Host request product code of the device
  kProductCode = 0x1D,  // Device acknowledges request by sending itsproduct code
  kReqHardwareVersion = 0x1E,  // Host requests hardware revision of device
  kHardwareVersion = 0x1F,  // Device acknowledges request by sending itshardware revision
  kReserved4 = 0x20,  // Reserved
  kReserved5 = 0x21,  // Reserved
  kSetNoRotation = 0x22,  // Initiates ‘no rotation’ update procedure
  kSetNoRotationAck = 0x23,  // Device acknowledges SetNoRotation message
  kRunSelftest = 0x24,  // Runs the built-in self test
  kSelftestAck = 0x25,  // Returns the self test results
  kReqSyncSettings = 0x2C,  // Request the synchronization settings of thedevice
  kSetSyncSettings = 0x2C,  // Set the synchronization settings of the device
  kReqSyncSettingsAck = 0x2D,  // Device returns synchronization settings
  kSetSyncSettingsAck = 0x2D,  // Device acknowledges SetSyncSettings
  kGoToConfig = 0x30,
  kGoToConfigAck = 0x31,
  kMtData = 0x32,  // Message with un-calibrated raw data, calibrateddata, orientation data or GPS
                   // PVT data (obsolete)
  kReqData = 0x34,  // Host requests device to send MTData2 message
  kMtData2 = 0x36,  // Message with one or more output data packets
  kWakeUp = 0x3E,
  kWakeUpAck = 0x3F,
  kReset = 0x40,
  kResetAck = 0x41,
  kError = 0x42,  // Error message
  kReqOptionFlags = 0x48,  // Requests state of OptionFlags
  kSetOptionFlags = 0x48,  // Sets state of OptionFlags
  kReqOptionFlagsAck = 0x49,  // Device returns OptionFlags
  kSetOptionFlagsAck = 0x49,  // Device acknowledges SetOptionFlags message
  kReqUTCTime = 0x60,  // Request UTC Time
  kSetUTCTime = 0x60,  // Sets time in UTC format
  kUTCTime = 0x61,  // Device return UTC Time
  kReqAvailableFilterProfiles = 0x62,  // Request available filter profiles
  kAvailableFilterProfiles = 0x63,  // Device return available filter profiles
  kReqFilterProfile = 0x64,  // Request current used filter profile
  kSetFilterProfile = 0x64,  // Host set current filter profile
  kReqFilterProfileAck = 0x65,  // Device return current filter profile
  kSetFilterProfileAck = 0x65,  // Device acknowledges SetFilterProfile
  kReserved6 = 0x66,  // Reserved
  kReserved7 = 0x66,  // Reserved
  kReserved8 = 0x67,  // Reserved
  kReserved9 = 0x67,  // Reserved
  kReqGnssLeverArm = 0x68,  // Request the lever arm settings that are storedin the device
  kSetGnssLeverArm = 0x68,  // Configure the GNSS lever arm in the device
  kReqGnssLeverArmAck = 0x69,  // Returns the lever arm settings that are storedin the device
  kSetGnssLeverArmAck = 0x69,  // Device acknowledges SetGnssLeverArm
  kReserved10 = 0x6A,  // Reserved
  kReserved11 = 0x6A,  // Reserved
  kReserved12 = 0x6B,  // Reserved
  kReserved13 = 0x6B,  // Reserved
  kReqLatLonAlt = 0x6E,  // Requests the latitude, longitude and altitudethat is stored in the
                         // device
  kSetLatLonAlt = 0x6E,  // Sets latitude, longitude and altitude in thedevice
  kReqLatLonAltAck = 0x6F,  // Returns the latitude, longitude and altitudethat is stored in the
                            // device
  kSetLatLonAltAck = 0x6F,  // Device acknowledges SetLatLonAlt
  kIccCommand = 0x74,
  kIccCommandAck = 0x75,
  kReqGnssPlatform = 0x76,  // Requests the current GNSS platform setting
  kReqGnssPlatformAck = 0x76,  // Returns the current GNSS platform setting
  kSetGnssPlatform = 0x76,  // Sets the GNSS platform setting
  kSetGnssPlatformAck = 0x76,  // Acknowledges setting of GNSS platform setting
  kReserved14 = 0x82,  // Reserved
  kReserved15 = 0x82,  // Reserved
  kReserved16 = 0x83,  // Reserved
  kReserved17 = 0x83,  // Reserved
  kReqLocationID = 0x84,  // Request location ID
  kSetLocationID = 0x84,  // Host sets location ID
  kReqLocationIDAck = 0x85,  // Device returns location ID
  kSetLocationIDAck = 0x85,  // Device acknowledges SetLocationID message
  kReqExtOutputMode = 0x86,  // Requests the current extended output mode
  kSetExtOutputMode = 0x86,  // Sets the extended output mode
  kExtOutputMode = 0x87,  // Device returns the current extended outputmode
  kSetExtOutputModeAck = 0x87,  // Device acknowledges SetExtOutputMode
  kReqPortConfig = 0x8C,  // Request the port configuration depending on size.
  kSetPortConfig = 0x8C,  // Set the port configuration depending on size.
  kReqStringOutputTypes = 0x8E,  // Request the configuration of the NMEA dataoutput
  kSetStringOutputTypes = 0x8E,  // Configures the NMEA data output
  kReqStringOutputTypesAck = 0x8F,  // Device returns the NMEA outputconfiguration
  kSetStringOutputTypesAck = 0x8F,  // Device acknowledges SetStringOutputTypesmessage
  kResetOrientation = 0xA4,  // Resets the orientation
  kResetOrientationAck = 0xA5,  // Device acknowledges ResetOrientationmessage
  kReserved18 = 0xA6,  // Reserved
  kReserved19 = 0xA7,  // Reserved
  kAdjustUTCTime = 0xA8,  // Sets correction ticks to UTC time
  kReqGnssReceiverSettings = 0xAC,  // Request the GNSS settings
  kReqGnssReceiverSettingsAck = 0xAD,  // Acknowledge the request of the GNSS settings
  kSetGnssReceiverSettings = 0xAC,  // Set the GNSS settings
  kSetGnssReceiverSettingsAck = 0xAD,  // Acknowledge the set of the GNSS settings
  kReqOutputConfiguration = 0xC0,  // Request the current output configuration
  kSetOutputConfiguration = 0xC0,  // Sets the output configuration
  kReqOutputConfigurationAck = 0xC1,  // Device returns the output configuration
  kSetOutputConfigurationAck = 0xC1,  // Device acknowledges SetOutputconfigurationmessage
  kReqOutputMode = 0xD0,  // Request current output mode (deprecated,see rev W of this document)
  kSetOutputMode = 0xD0,  // Host sets output mode (deprecated, see revW of this document)
  kReqOutputModeAck = 0xD1,  // Device returns output mode (deprecated,see rev W of this document)
  kSetOutputModeAck = 0xD1,  // Device acknowledges SetOutputModemessage (deprecated, see rev W of
                             // this document)
  kReqOutputSettings = 0xD2,  // Request current output settings (deprecated,see rev W of this
                              // document)
  kSetOutputSettings = 0xD2,  // Host sets output settings (deprecated, seerev W of this document)
  kReqOutputSettingsAck = 0xD3,  // Device returns output settings (deprecated,see rev W of this
                                 // document)
  kSetOutputSettingsAck = 0xD3,  // Device acknowledges SetOutputSettingsmessage (deprecated, see
                                 // rev W of thisdocument)
  kReserved20 = 0xD4,  // Reserved
  kReserved21 = 0xD5,  // Reserved
  kReserved22 = 0xD6,  // Reserved
  kReserved23 = 0xD7,  // Reserved
  kReserved24 = 0xD8,  // Reserved
  kReserved25 = 0xD9,  // Reserved
  kReqErrorMode = 0xDA,  // Request error mode (deprecated, see rev2020.A of this document)
  kSetErrorMode = 0xDA,  // Host sets error mode (deprecated, see rev2020.A of this document)
  kReqErrorModeAck = 0xDB,  // Device returns error mode (deprecated, see rev2020.A of this
                            // document)
  kSetErrorModeAck = 0xDB,  // Device acknowledges SetErrorMode message(deprecated, see rev 2020.A
                            // of this document)
  kReqTransmitDelay = 0xDC,  // Request the transmit delay in RS485 MT’s
  kSetTransmitDelay = 0xDC,  // Host sets transmit delay in RS485 MT’s
  kReqTransmitDelayAck = 0xDD,  // Device returns the transmit delay in RS485 MT’s
  kSetTransmitDelayAck = 0xDD,  // Device acknowledges SetTransmitDelaymessage
  kReserved26 = 0xE0,  // Reserved
  kReserved27 = 0xE1,  // Reserved
  kReqAlignmentRotation = 0xEC,  // Requests the sensor alignment or localalignment
  kSetAlignmentRotation = 0xEC,  // Sets the sensor alignment or local alignment
  kReqRotationQuaternionAck = 0xED,  // Device acknowledges ReqRotationQuaternion
  kSetRotationQuaternionAck = 0xED,  // Device acknowledges SetRotationQuaternion
};

};  // namespace xsens
