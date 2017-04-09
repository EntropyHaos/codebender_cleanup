/*
* MFRC522.cpp - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
* _Please_ see the comments in MFRC522.h - they give useful hints and background.
* Released into the public domain.
*/

#include &lt;Arduino.h&gt;
#include &lt;MFRC522.h&gt;

/////////////////////////////////////////////////////////////////////////////////////
// Functions for setting up the Arduino
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Constructor.
 * Prepares the output pins.
 */
MFRC522::MFRC522(	byte chipSelectPin,		///&lt; Arduino pin connected to MFRC522's SPI slave select input (Pin 24, NSS, active low)
					byte resetPowerDownPin	///&lt; Arduino pin connected to MFRC522's reset and power down input (Pin 6, NRSTPD, active low)
				) {
	// Set the chipSelectPin as digital output, do not select the slave yet
	_chipSelectPin = chipSelectPin;
	pinMode(_chipSelectPin, OUTPUT);
	digitalWrite(_chipSelectPin, LOW);
	
	// Set the resetPowerDownPin as digital output, do not reset or power down.
	_resetPowerDownPin = resetPowerDownPin;
	pinMode(_resetPowerDownPin, OUTPUT);
	digitalWrite(_resetPowerDownPin, HIGH);
	
	// Set SPI bus to work with MFRC522 chip.
	setSPIConfig();
} // End constructor

/**
 * Set SPI bus to work with MFRC522 chip.
 * Please call this function if you have changed the SPI config since the MFRC522 constructor was run.
 */
void MFRC522::setSPIConfig() {
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
} // End setSPIConfig()

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(	byte reg,		///&lt; The register to write to. One of the PCD_Register enums.
									byte value		///&lt; The value to write.
								) {
	digitalWrite(_chipSelectPin, LOW);		// Select slave
	SPI.transfer(reg &amp; 0x7E);					// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	SPI.transfer(value);
	digitalWrite(_chipSelectPin, HIGH);		// Release slave again
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(	byte reg,		///&lt; The register to write to. One of the PCD_Register enums.
									byte count,		///&lt; The number of bytes to write to the register
									byte *values	///&lt; The values to write. Byte array.
								) {
	digitalWrite(_chipSelectPin, LOW);		// Select slave
	SPI.transfer(reg &amp; 0x7E);				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (byte index = 0; index &lt; count; index++) {
		SPI.transfer(values[index]);
	}
	digitalWrite(_chipSelectPin, HIGH);		// Release slave again
} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
byte MFRC522::PCD_ReadRegister(	byte reg	///&lt; The register to read from. One of the PCD_Register enums.
								) {
	byte value;
	digitalWrite(_chipSelectPin, LOW);			// Select slave
	SPI.transfer(0x80 | (reg &amp; 0x7E));			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	value = SPI.transfer(0);					// Read the value back. Send 0 to stop reading.
	digitalWrite(_chipSelectPin, HIGH);			// Release slave again
	return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_ReadRegister(	byte reg,		///&lt; The register to read from. One of the PCD_Register enums.
								byte count,		///&lt; The number of bytes to read
								byte *values,	///&lt; Byte array to store the values in.
								byte rxAlign	///&lt; Only bit positions rxAlign..7 in values[0] are updated.
								) {
	if (count == 0) {
		return;
	}
	//Serial.print(&quot;Reading &quot;); 	Serial.print(count); Serial.println(&quot; bytes from register.&quot;);
	byte address = 0x80 | (reg &amp; 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	byte index = 0;							// Index in values array.
	digitalWrite(_chipSelectPin, LOW);		// Select slave
	count--;								// One read is performed outside of the loop
	SPI.transfer(address);					// Tell MFRC522 which address we want to read
	while (index &lt; count) {
		if (index == 0 &amp;&amp; rxAlign) { // Only update bit positions rxAlign..7 in values[0]
			// Create bit mask for bit positions rxAlign..7
			byte mask = 0;
			for (byte i = rxAlign; i &lt;= 7; i++) {
				mask |= (1 &lt;&lt; i);
			}
			// Read value and tell that we want to read the same address again.
			byte value = SPI.transfer(address);	
			// Apply mask to both current value of values[0] and the new data in value.
			values[0] = (values[index] &amp; ~mask) | (value &amp; mask);
		}
		else { // Normal case
			values[index] = SPI.transfer(address);	// Read value and tell that we want to read the same address again.
		}
		index++;
	}
	values[index] = SPI.transfer(0);			// Read the final byte. Send 0 to stop reading.
	digitalWrite(_chipSelectPin, HIGH);			// Release slave again
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void MFRC522::PCD_SetRegisterBitMask(	byte reg,	///&lt; The register to update. One of the PCD_Register enums.
										byte mask	///&lt; The bits to set.
									) { 
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void MFRC522::PCD_ClearRegisterBitMask(	byte reg,	///&lt; The register to update. One of the PCD_Register enums.
										byte mask	///&lt; The bits to clear.
									  ) {
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp &amp; (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PCD_CalculateCRC(	byte *data,		///&lt; In: Pointer to the data to transfer to the FIFO for CRC calculation.
								byte length,	///&lt; In: The number of bytes to transfer.
								byte *result	///&lt; Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);					// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, length, data);		// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73µs.
	word i = 5000;
	byte n;
	while (1) {
		n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq   reserved CRCIRq reserved reserved
		if (n &amp; 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop calculating CRC for new content in the FIFO.
	
	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadRegister(CRCResultRegL);
	result[1] = PCD_ReadRegister(CRCResultRegH);
	return STATUS_OK;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void MFRC522::PCD_Init() {
	if (digitalRead(_resetPowerDownPin) == LOW) { //The MFRC522 chip is in power down mode.
		digitalWrite(_resetPowerDownPin, HIGH);	// Exit power down mode. This triggers a hard reset.
		// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74µs. Let us be generous: 50ms.
		delay(50);
	}
	else { // Perform a soft reset
		PCD_Reset();
	}
	
	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(TPrescalerReg, 0xA9);	// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 =&gt; f_timer=40kHz, ie a timer period of 25µs.
    PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void MFRC522::PCD_Reset() {
	PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74µs. Let us be generous: 50ms.
	delay(50);
	// Wait for the PowerDown bit in CommandReg to be cleared
	while (PCD_ReadRegister(CommandReg) &amp; (1&lt;&lt;4)) {
		// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
	}
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins disabled.
 */
void MFRC522::PCD_AntennaOn() {
	byte value = PCD_ReadRegister(TxControlReg);
	if ((value &amp; 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PCD_TransceiveData(	byte *sendData,		///&lt; Pointer to the data to transfer to the FIFO.
									byte sendLen,		///&lt; Number of bytes to transfer to the FIFO.
									byte *backData,		///&lt; NULL or pointer to buffer if data should be read back after executing the command.
									byte *backLen,		///&lt; In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									byte *validBits,	///&lt; In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
									byte rxAlign,		///&lt; In: Defines the bit position in backData[0] for the first bit received. Default 0.
									bool checkCRC		///&lt; In: True =&gt; The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a commend, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PCD_CommunicateWithPICC(	byte command,		///&lt; The command to execute. One of the PCD_Command enums.
										byte waitIRq,		///&lt; The bits in the ComIrqReg register that signals successful completion of the command.
										byte *sendData,		///&lt; Pointer to the data to transfer to the FIFO.
										byte sendLen,		///&lt; Number of bytes to transfer to the FIFO.
										byte *backData,		///&lt; NULL or pointer to buffer if data should be read back after executing the command.
										byte *backLen,		///&lt; In: Max number of bytes to write to *backData. Out: The number of bytes returned.
										byte *validBits,	///&lt; In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
										byte rxAlign,		///&lt; In: Defines the bit position in backData[0] for the first bit received. Default 0.
										bool checkCRC		///&lt; In: True =&gt; The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	byte n, _validBits;
	unsigned int i;

	// Prepare values for BitFramingReg
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming	= (rxAlign &lt;&lt; 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);			// Execute the command
	if (command == PCD_Transceive) 	{
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86µs.
	i = 2000;
	while (1) {
		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq   HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n &amp; waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n &amp; 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	
	// Stop now if any errors except collisions were detected.
	byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl   CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue &amp; 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}	

	// If the caller wants data back, get it from the MFRC522.
	if (backData &amp;&amp; backLen) {
		n = PCD_ReadRegister(FIFOLevelReg);						// Number of bytes in the FIFO
		if (n &gt; *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;												// Number of bytes returned
		PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);		// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) &amp; 0x07;	// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue &amp; 0x08) { // CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData &amp;&amp; backLen &amp;&amp; checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 &amp;&amp; _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen &lt; 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		byte controlBuffer[2]; 
		n = PCD_CalculateCRC(&amp;backData[0], *backLen - 2, &amp;controlBuffer[0]);
		if (n != STATUS_OK) {
			return n;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PICC_RequestA(byte *bufferATQA,	///&lt; The buffer to store the ATQA (Answer to request) in
							byte *bufferSize	///&lt; Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
							) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PICC_WakeupA(	byte *bufferATQA,	///&lt; The buffer to store the ATQA (Answer to request) in
							byte *bufferSize	///&lt; Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
							) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
byte MFRC522::PICC_REQA_or_WUPA(	byte command, 		///&lt; The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
									byte *bufferATQA,	///&lt; The buffer to store the ATQA (Answer to request) in
									byte *bufferSize	///&lt; Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
							   ) {
	byte validBits;
	byte status;
	
	if (bufferATQA == NULL || *bufferSize &lt; 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);			// ValuesAfterColl=1 =&gt; Bits received after collision are cleared.
	validBits = 7;										// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&amp;command, 1, bufferATQA, bufferSize, &amp;validBits);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PICC_Select(	Uid *uid,			///&lt; Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
							byte validBits		///&lt; The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid-&gt;size.
						 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel	= 1; 
	byte result;
	byte count;
	byte index;
	byte uidIndex;					// The first index in uid-&gt;uidByte[] that is used in the current Cascade Level.
	char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	byte *responseBuffer;
	byte responseLength;
	
	// Description of buffer structure:
	// 		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	// 		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	// 		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	// 		Byte 3: UID-data
	// 		Byte 4: UID-data
	// 		Byte 5: UID-data
	// 		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits &gt; 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);			// ValuesAfterColl=1 =&gt; Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while ( ! uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits &amp;&amp; uid-&gt;size &gt; 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits &amp;&amp; uid-&gt;size &gt; 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits &lt; 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid-&gt;uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy &gt; maxBytes) { 
				bytesToCopy = maxBytes;
			}
			for (count = 0; count &lt; bytesToCopy; count++) {
				buffer[index++] = uid-&gt;uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while ( ! selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits &gt;= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(&quot;SELECT: currentLevelKnownBits=&quot;); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calulate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &amp;buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 =&gt; All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &amp;buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(&quot;ANTICOLLISION: currentLevelKnownBits=&quot;); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index &lt;&lt; 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &amp;buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a seperate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign &lt;&lt; 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &amp;responseLength, &amp;txLastBits, rxAlign);			
			if (result == STATUS_COLLISION) { // More than one PICC in the field =&gt; collision.
				result = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (result &amp; 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = result &amp; 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos &lt;= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 &lt;&lt; count); 
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits &gt;= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while ( ! selectDone)

		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid-&gt;uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count &lt; bytesToCopy; count++) {
			uid-&gt;uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) {		// SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &amp;buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] &amp; 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid-&gt;sak = responseBuffer[0];
		}
	} // End of while ( ! uidComplete)
	
	// Set correct uid-&gt;size
	uid-&gt;size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
byte MFRC522::PICC_HaltA() {
	byte result;
	byte buffer[4]; 

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &amp;buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is an success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 * 
 * All keys are set to FFFFFFFFFFFFh at chip delivery. A key consists of 6 bytes.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
byte MFRC522::PCD_Authenticate(byte command,		///&lt; PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
								byte blockAddr, 	///&lt; The block number. See numbering in the comments in the .h file.
								MIFARE_Key *key,	///&lt; Pointer to the Crypteo1 key to use (6 bytes)
								Uid *uid			///&lt; Pointer to Uid struct. The first 4 bytes of the UID is used.
								) {
	byte waitIRq = 0x10;		// IdleIRq
	
	// Build command buffer
	byte sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (byte i = 0; i &lt; MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key-&gt;keyByte[i];
	}
	for (byte i = 0; i &lt; 4; i++) {				// The first 4 bytes of the UID
		sendData[8+i] = uid-&gt;uidByte[i];
	}
	
	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &amp;sendData[0], sizeof(sendData));
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void MFRC522::PCD_StopCrypto1() {
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved   MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 * 
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_Read(	byte blockAddr, 	///&lt; MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
							byte *buffer,		///&lt; The buffer to store the data in
							byte *bufferSize	///&lt; Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
						) {
	byte result;
	
	// Sanity check
	if (buffer == NULL || *bufferSize &lt; 18) {
		return STATUS_NO_ROOM;
	}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &amp;buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()

/**
 * Writes 16 bytes to the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight the opretaion is called &quot;COMPATIBILITY WRITE&quot;.
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 * * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_Write(	byte blockAddr, ///&lt; MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
							byte *buffer,	///&lt; The 16 bytes to write to the PICC
							byte bufferSize	///&lt; Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
						) {
	byte result;

	// Sanity check
	if (buffer == NULL || bufferSize &lt; 16) {
		return STATUS_INVALID;
	}

	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	byte cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(	buffer, bufferSize); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	return STATUS_OK;
} // End MIFARE_Write()

/**
 * Writes a 4 byte page to the active MIFARE Ultralight PICC.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_Ultralight_Write(	byte page, 		///&lt; The page (2-15) to write to.
										byte *buffer,	///&lt; The 4 bytes to write to the PICC
										byte bufferSize	///&lt; Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
									) {
	byte result;

	// Sanity check
	if (buffer == NULL || bufferSize &lt; 4) {
		return STATUS_INVALID;
	}

	// Build commmand buffer
	byte cmdBuffer[6];
	cmdBuffer[0] = PICC_CMD_UL_WRITE;
	cmdBuffer[1] = page;
	memcpy(&amp;cmdBuffer[2], buffer, 4);
	
	// Perform the write
	result = PCD_MIFARE_Transceive(cmdBuffer, 6); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
} // End MIFARE_Ultralight_Write()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in &quot;value block&quot; mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_Decrement(	byte blockAddr, ///&lt; The block (0-0xff) number.
								long delta		///&lt; This number is subtracted from the value of block blockAddr.
							) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in &quot;value block&quot; mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_Increment(	byte blockAddr, ///&lt; The block (0-0xff) number.
								long delta		///&lt; This number is added to the value of block blockAddr.
							) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

/**
 * MIFARE Restore copies the value of the addressed block into a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in &quot;value block&quot; mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_Restore(	byte blockAddr ///&lt; The block (0-0xff) number.
							) {
	// The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
	// Doing only a single step does not work, so I chose to transfer 0L in step two.
	return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
} // End MIFARE_Restore()

/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_TwoStepHelper(	byte command,	///&lt; The command to use
									byte blockAddr,	///&lt; The block (0-0xff) number.
									long data		///&lt; The data to transfer in step 2
									) {
	byte result;
	byte cmdBuffer[2]; // We only need room for 2 bytes.

	// Step 1: Tell the PICC the command and block address
	cmdBuffer[0] = command;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(	(byte *)&amp;data, 4, true); // Adds CRC_A and accept timeout as success.
	if (result != STATUS_OK) {
		return result;
	}

	return STATUS_OK;
} // End MIFARE_TwoStepHelper()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in &quot;value block&quot; mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::MIFARE_Transfer(	byte blockAddr ///&lt; The block (0-0xff) number.
							) {
	byte result;
	byte cmdBuffer[2]; // We only need room for 2 bytes.

	// Tell the PICC we want to transfer the result into block blockAddr.
	cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(	cmdBuffer, 2); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
} // End MIFARE_Transfer()


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PCD_MIFARE_Transceive(	byte *sendData,		///&lt; Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
										byte sendLen,		///&lt; Number of bytes in sendData.
										bool acceptTimeout	///&lt; True =&gt; A timeout is also success
									) {
	byte result;
	byte cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

	// Sanity check
	if (sendData == NULL || sendLen &gt; 16) {
		return STATUS_INVALID;
	}
	
	// Copy sendData[] to cmdBuffer[] and add CRC_A
	memcpy(cmdBuffer, sendData, sendLen);
	result = PCD_CalculateCRC(cmdBuffer, sendLen, &amp;cmdBuffer[sendLen]);
	if (result != STATUS_OK) { 
		return result;
	}
	sendLen += 2;
	
	// Transceive the data, store the reply in cmdBuffer[]
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	byte cmdBufferSize = sizeof(cmdBuffer);
	byte validBits = 0;
	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &amp;cmdBufferSize, &amp;validBits);
	if (acceptTimeout &amp;&amp; result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result != STATUS_OK) {
		return result;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()

/**
 * Returns a string pointer to a status code name.
 * 
 */
const char *MFRC522::GetStatusCodeName(byte code	///&lt; One of the StatusCode enums.
										) {
	switch (code) {
		case STATUS_OK:				return &quot;Success.&quot;; break;
		case STATUS_ERROR:			return &quot;Error in communication.&quot;; break;
		case STATUS_COLLISION:		return &quot;Collission detected.&quot;; break;
		case STATUS_TIMEOUT:		return &quot;Timeout in communication.&quot;; break;
		case STATUS_NO_ROOM:		return &quot;A buffer is not big enough.&quot;; break;
		case STATUS_INTERNAL_ERROR:	return &quot;Internal error in the code. Should not happen.&quot;; break;
		case STATUS_INVALID:		return &quot;Invalid argument.&quot;; break;
		case STATUS_CRC_WRONG:		return &quot;The CRC_A does not match.&quot;; break;
		case STATUS_MIFARE_NACK:	return &quot;A MIFARE PICC responded with NAK.&quot;; break;
		default:
			return &quot;Unknown error&quot;;
			break;
	}
} // End GetStatusCodeName()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
byte MFRC522::PICC_GetType(byte sak		///&lt; The SAK byte returned from PICC_Select().
							) {
	if (sak &amp; 0x04) { // UID not complete
		return PICC_TYPE_NOT_COMPLETE;
	}
	
	switch (sak) {
		case 0x09:	return PICC_TYPE_MIFARE_MINI;	break;
		case 0x08:	return PICC_TYPE_MIFARE_1K;		break;
		case 0x18:	return PICC_TYPE_MIFARE_4K;		break;
		case 0x00:	return PICC_TYPE_MIFARE_UL;		break;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;	break;
		case 0x01:	return PICC_TYPE_TNP3XXX;		break;
		default:	break;
	}
	
	if (sak &amp; 0x20) {
		return PICC_TYPE_ISO_14443_4;
	}
	
	if (sak &amp; 0x40) {
		return PICC_TYPE_ISO_18092;
	}
	
	return PICC_TYPE_UNKNOWN;
} // End PICC_GetType()

/**
 * Returns a string pointer to the PICC type name.
 * 
 */
const char *MFRC522::PICC_GetTypeName(byte piccType	///&lt; One of the PICC_Type enums.
										) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:		return &quot;PICC compliant with ISO/IEC 14443-4&quot;;		break;
		case PICC_TYPE_ISO_18092:		return &quot;PICC compliant with ISO/IEC 18092 (NFC)&quot;;	break;
		case PICC_TYPE_MIFARE_MINI:		return &quot;MIFARE Mini, 320 bytes&quot;;					break;
		case PICC_TYPE_MIFARE_1K:		return &quot;MIFARE 1KB&quot;;								break;
		case PICC_TYPE_MIFARE_4K:		return &quot;MIFARE 4KB&quot;;								break;
		case PICC_TYPE_MIFARE_UL:		return &quot;MIFARE Ultralight or Ultralight C&quot;;			break;
		case PICC_TYPE_MIFARE_PLUS:		return &quot;MIFARE Plus&quot;;								break;
		case PICC_TYPE_TNP3XXX:			return &quot;MIFARE TNP3XXX&quot;;							break;
		case PICC_TYPE_NOT_COMPLETE:	return &quot;SAK indicates UID is not complete.&quot;;		break;
		case PICC_TYPE_UNKNOWN:
		default:						return &quot;Unknown type&quot;;								break;
	}
} // End PICC_GetTypeName()

/**
 * Dumps debug info about the selected PICC to Serial.
 * On success the PICC is halted after dumping the data.
 * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried. 
 */
void MFRC522::PICC_DumpToSerial(Uid *uid	///&lt; Pointer to Uid struct returned from a successful PICC_Select().
								) {
	MIFARE_Key key;
	
	// UID
	Serial.print(&quot;Card UID:&quot;);
	for (byte i = 0; i &lt; uid-&gt;size; i++) {
		Serial.print(uid-&gt;uidByte[i] &lt; 0x10 ? &quot; 0&quot; : &quot; &quot;);
		Serial.print(uid-&gt;uidByte[i], HEX);
	} 
	Serial.println();

	// PICC type
	byte piccType = PICC_GetType(uid-&gt;sak);
	Serial.print(&quot;PICC type: &quot;);
	Serial.println(PICC_GetTypeName(piccType));
	
	// Dump contents
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
		case PICC_TYPE_MIFARE_1K:
		case PICC_TYPE_MIFARE_4K:
			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for (byte i = 0; i &lt; 6; i++) {
				key.keyByte[i] = 0xFF;
			}
			PICC_DumpMifareClassicToSerial(uid, piccType, &amp;key);
			break;
			
		case PICC_TYPE_MIFARE_UL:
			PICC_DumpMifareUltralightToSerial();
			break;
			
		case PICC_TYPE_ISO_14443_4:	
		case PICC_TYPE_ISO_18092:
		case PICC_TYPE_MIFARE_PLUS:
		case PICC_TYPE_TNP3XXX:
			Serial.println(&quot;Dumping memory contents not implemented for that PICC type.&quot;);
			break;
			
		case PICC_TYPE_UNKNOWN:
		case PICC_TYPE_NOT_COMPLETE:
		default:
			break; // No memory dump here
	}

	Serial.println();
	PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
} // End PICC_DumpToSerial()

/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
void MFRC522::PICC_DumpMifareClassicToSerial(	Uid *uid,		///&lt; Pointer to Uid struct returned from a successful PICC_Select().
												byte piccType,	///&lt; One of the PICC_Type enums.
												MIFARE_Key *key	///&lt; Key A used for all sectors.
											) {
	byte no_of_sectors = 0;
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
			// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			no_of_sectors = 5;
			break;
			
		case PICC_TYPE_MIFARE_1K:
			// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			no_of_sectors = 16;
			break;
			
		case PICC_TYPE_MIFARE_4K:
			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			no_of_sectors = 40;
			break;
			
		default: // Should not happen. Ignore.
			break; 
	}
	
	// Dump sectors, highest address first.
	if (no_of_sectors) {
		Serial.println(&quot;Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits&quot;);
		for (char i = no_of_sectors - 1; i &gt;= 0; i--) {
			PICC_DumpMifareClassicSectorToSerial(uid, key, i);
		}
	}
	PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
	PCD_StopCrypto1();
} // End PICC_DumpMifareClassicToSerial()

/**
 * Dumps memory contents of a sector of a MIFARE Classic PICC.
 * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
 * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 */
void MFRC522::PICC_DumpMifareClassicSectorToSerial(Uid *uid,			///&lt; Pointer to Uid struct returned from a successful PICC_Select().
													MIFARE_Key *key,	///&lt; Key A for the sector.
													byte sector			///&lt; The sector to dump, 0..39.
													) {
	byte status;
	byte firstBlock;		// Address of lowest address to dump actually last block dumped)
	byte no_of_blocks;		// Number of blocks in sector
	bool isSectorTrailer;	// Set to true while handling the &quot;last&quot; (ie highest address) in the sector.

	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	byte c1, c2, c3;		// Nibbles
	byte c1_, c2_, c3_;		// Inverted nibbles
	bool invertedError;		// True if one of the inverted nibbles did not match
	byte g[4];				// Access bits for each of the four groups.
	byte group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group

	// Determine position and size of sector.
	if (sector &lt; 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector &lt; 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return;
	}
		
	// Dump blocks, highest address first.
	byte byteCount;
	byte buffer[18];
	byte blockAddr;
	isSectorTrailer = true;
	for (char blockOffset = no_of_blocks - 1; blockOffset &gt;= 0; blockOffset--) {
		blockAddr = firstBlock + blockOffset;
		// Sector number - only on first line
		if (isSectorTrailer) {
			Serial.print(sector &lt; 10 ? &quot;   &quot; : &quot;  &quot;); // Pad with spaces
			Serial.print(sector);
			Serial.print(&quot;   &quot;);
		}
		else {
			Serial.print(&quot;       &quot;);
		}
		// Block number
		Serial.print(blockAddr &lt; 10 ? &quot;   &quot; : (blockAddr &lt; 100 ? &quot;  &quot;	 : &quot; &quot;)); // Pad with spaces
		Serial.print(blockAddr);
		Serial.print(&quot;  &quot;);
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != STATUS_OK) {
				Serial.print(&quot;PCD_Authenticate() failed: &quot;);
				Serial.println(GetStatusCodeName(status));
				return;
			}
		}
		// Read block
		byteCount = sizeof(buffer);
		status = MIFARE_Read(blockAddr, buffer, &amp;byteCount);
		if (status != STATUS_OK) {
			Serial.print(&quot;MIFARE_Read() failed: &quot;);
			Serial.println(GetStatusCodeName(status));
			continue;
		}
		// Dump data
		for (byte index = 0; index &lt; 16; index++) {
			Serial.print(buffer[index] &lt; 0x10 ? &quot; 0&quot; : &quot; &quot;);
			Serial.print(buffer[index], HEX);
			if ((index % 4) == 3) {
				Serial.print(&quot; &quot;);
			}
		}
		// Parse sector trailer data
		if (isSectorTrailer) {
			c1  = buffer[7] &gt;&gt; 4;
			c2  = buffer[8] &amp; 0xF;
			c3  = buffer[8] &gt;&gt; 4;
			c1_ = buffer[6] &amp; 0xF;
			c2_ = buffer[6] &gt;&gt; 4;
			c3_ = buffer[7] &amp; 0xF;
			invertedError = (c1 != (~c1_ &amp; 0xF)) || (c2 != (~c2_ &amp; 0xF)) || (c3 != (~c3_ &amp; 0xF));
			g[0] = ((c1 &amp; 1) &lt;&lt; 2) | ((c2 &amp; 1) &lt;&lt; 1) | ((c3 &amp; 1) &lt;&lt; 0);
			g[1] = ((c1 &amp; 2) &lt;&lt; 1) | ((c2 &amp; 2) &lt;&lt; 0) | ((c3 &amp; 2) &gt;&gt; 1);
			g[2] = ((c1 &amp; 4) &lt;&lt; 0) | ((c2 &amp; 4) &gt;&gt; 1) | ((c3 &amp; 4) &gt;&gt; 2);
			g[3] = ((c1 &amp; 8) &gt;&gt; 1) | ((c2 &amp; 8) &gt;&gt; 2) | ((c3 &amp; 8) &gt;&gt; 3);
			isSectorTrailer = false;
		}

		// Which access group is this block in?
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = true;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}
		
		if (firstInGroup) {
			// Print access bits
			Serial.print(&quot; [ &quot;);
			Serial.print((g[group] &gt;&gt; 2) &amp; 1, DEC); Serial.print(&quot; &quot;);
			Serial.print((g[group] &gt;&gt; 1) &amp; 1, DEC); Serial.print(&quot; &quot;);
			Serial.print((g[group] &gt;&gt; 0) &amp; 1, DEC);
			Serial.print(&quot; ] &quot;);
			if (invertedError) {
				Serial.print(&quot; Inverted access bits did not match! &quot;);
			}
		}
		
		if (group != 3 &amp;&amp; (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
			long value = (long(buffer[3])&lt;&lt;24) | (long(buffer[2])&lt;&lt;16) | (long(buffer[1])&lt;&lt;8) | long(buffer[0]);
			Serial.print(&quot; Value=0x&quot;); Serial.print(value, HEX);
			Serial.print(&quot; Adr=0x&quot;); Serial.print(buffer[12], HEX);
		}
		Serial.println();
	}
	
	return;
} // End PICC_DumpMifareClassicSectorToSerial()

/**
 * Dumps memory contents of a MIFARE Ultralight PICC.
 */
void MFRC522::PICC_DumpMifareUltralightToSerial() {
	byte status;
	byte byteCount;
	byte buffer[18];
	byte i;

	Serial.println(&quot;Page  0  1  2  3&quot;);
	// Try the mpages of the original Ultralight. Ultralight C has more pages.
	for (byte page = 0; page &lt; 16; page +=4) { // Read returns data for 4 pages at a time.
		// Read pages
		byteCount = sizeof(buffer);
		status = MIFARE_Read(page, buffer, &amp;byteCount);
		if (status != STATUS_OK) {
			Serial.print(&quot;MIFARE_Read() failed: &quot;);
			Serial.println(GetStatusCodeName(status));
			break;
		}
		// Dump data
		for (byte offset = 0; offset &lt; 4; offset++) {
			i = page + offset;
			Serial.print(i &lt; 10 ? &quot;  &quot; : &quot; &quot;); // Pad with spaces
			Serial.print(i);
			Serial.print(&quot;  &quot;);
			for (byte index = 0; index &lt; 4; index++) {
				i = 4 * offset + index;
				Serial.print(buffer[i] &lt; 0x10 ? &quot; 0&quot; : &quot; &quot;);
				Serial.print(buffer[i], HEX);
			}
			Serial.println();
		}
	}
} // End PICC_DumpMifareUltralightToSerial()

/**
 * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tupples C1 is MSB (=4) and C3 is LSB (=1).
 */
void MFRC522::MIFARE_SetAccessBits(	byte *accessBitBuffer,	///&lt; Pointer to byte 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
									byte g0,				///&lt; Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
									byte g1,				///&lt; Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
									byte g2,				///&lt; Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
									byte g3					///&lt; Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
								) {
	byte c1 = ((g3 &amp; 4) &lt;&lt; 1) | ((g2 &amp; 4) &lt;&lt; 0) | ((g1 &amp; 4) &gt;&gt; 1) | ((g0 &amp; 4) &gt;&gt; 2);
	byte c2 = ((g3 &amp; 2) &lt;&lt; 2) | ((g2 &amp; 2) &lt;&lt; 1) | ((g1 &amp; 2) &lt;&lt; 0) | ((g0 &amp; 2) &gt;&gt; 1);
	byte c3 = ((g3 &amp; 1) &lt;&lt; 3) | ((g2 &amp; 1) &lt;&lt; 2) | ((g1 &amp; 1) &lt;&lt; 1) | ((g0 &amp; 1) &lt;&lt; 0);
	
	accessBitBuffer[0] = (~c2 &amp; 0xF) &lt;&lt; 4 | (~c1 &amp; 0xF);
	accessBitBuffer[1] =          c1 &lt;&lt; 4 | (~c3 &amp; 0xF);
	accessBitBuffer[2] =          c3 &lt;&lt; 4 | c2;
} // End MIFARE_SetAccessBits()

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only &quot;new&quot; cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool MFRC522::PICC_IsNewCardPresent() {
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);
	byte result = PICC_RequestA(bufferATQA, &amp;bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
bool MFRC522::PICC_ReadCardSerial() {
	byte result = PICC_Select(&amp;uid);
	return (result == STATUS_OK);//return a '1' if PICC_Select returns a '1', else a '0'
} // End PICC_ReadCardSerial()
 
