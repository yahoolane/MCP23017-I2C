// Register addresses
#define IODIRA 0x00 // I/O direction register for Port A
#define IODIRB 0x01 // I/O direction register for Port B
#define GPPUA  0x0C // GPIO pull-up register for Port A
#define GPPUB  0x0D // GPIO pull-up register for Port B
#define GPIOA  0x12 // General purpose I/O register for Port A
#define GPIOB  0x13 // General purpose I/O register for Port B
#define GPINTENA 0x02 // Interrupt-on-change control register for Port A
#define GPINTENB 0x03 // Interrupt-on-change control register for Port B
#define DEFVALA 0x06 // Default value register for Port A
#define DEFVALB 0x07 // Default value register for Port B
#define INTCONA 0x08 // Interrupt control register for Port A
#define INTCONB 0x09 // Interrupt control register for Port B
#define INTFA  0x0E // Interrupt flag register for Port A
#define INTFB  0x0F // Interrupt flag register for Port B

// Initialize the MCP23017 at a specific I2C address
void mcp23017_begin(uint8_t address) {
  Wire.begin(); // Initialize the I2C bus
  // Set all pins as inputs by default
  mcp23017_writeRegister(address, IODIRA, 0xFF); // Port A
  mcp23017_writeRegister(address, IODIRB, 0xFF); // Port B
}

// Set the direction of a pin (INPUT, OUTPUT, or INPUT_PULLUP)
void mcp23017_pinMode(uint8_t address, uint8_t pin, uint8_t mode) {
  uint8_t port = pin / 8; // Determine which port (A or B)
  uint8_t pinMask = 1 << (pin % 8); // Create a mask for the specific pin

  uint8_t iodirReg = (port == 0) ? IODIRA : IODIRB; // Select the appropriate IODIR register
  uint8_t gppuReg = (port == 0) ? GPPUA : GPPUB; // Select the appropriate GPPU register

  uint8_t currentDir = mcp23017_readRegister(address, iodirReg); // Read the current direction setting
  uint8_t currentPullUp = mcp23017_readRegister(address, gppuReg); // Read the current pull-up setting

  if (mode == INPUT) {
    currentDir |= pinMask; // Set the pin as input
    currentPullUp &= ~pinMask; // Disable pull-up
  } else if (mode == INPUT_PULLUP) {
    currentDir |= pinMask; // Set the pin as input
    currentPullUp |= pinMask; // Enable pull-up
  } else {
    currentDir &= ~pinMask; // Set the pin as output
    currentPullUp &= ~pinMask; // Disable pull-up
  }

  mcp23017_writeRegister(address, iodirReg, currentDir); // Write the new direction setting
  mcp23017_writeRegister(address, gppuReg, currentPullUp); // Write the new pull-up setting
}

// Write a value to a pin (HIGH or LOW)
void mcp23017_digitalWrite(uint8_t address, uint8_t pin, uint8_t value) {
  uint8_t port = pin / 8; // Determine which port (A or B)
  uint8_t pinMask = 1 << (pin % 8); // Create a mask for the specific pin

  uint8_t gpioReg = (port == 0) ? GPIOA : GPIOB; // Select the appropriate GPIO register
  uint8_t currentGpio = mcp23017_readRegister(address, gpioReg); // Read the current GPIO state

  if (value == HIGH) {
    currentGpio |= pinMask; // Set the pin high
  } else {
    currentGpio &= ~pinMask; // Set the pin low
  }

  mcp23017_writeRegister(address, gpioReg, currentGpio); // Write the new GPIO state
}

// Read the value of a pin (HIGH or LOW)
uint8_t mcp23017_digitalRead(uint8_t address, uint8_t pin) {
  uint8_t port = pin / 8; // Determine which port (A or B)
  uint8_t pinMask = 1 << (pin % 8); // Create a mask for the specific pin

  uint8_t gpioReg = (port == 0) ? GPIOA : GPIOB; // Select the appropriate GPIO register
  uint8_t gpioState = mcp23017_readRegister(address, gpioReg); // Read the current GPIO state

  return (gpioState & pinMask) ? HIGH : LOW; // Return the pin state
}

// Write a value to a register
void mcp23017_writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read a value from a register
uint8_t mcp23017_readRegister(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1); // Ensure both arguments are uint8_t
  return Wire.read();
}

// Enable interrupts on a specific pin
void mcp23017_interruptEnable(uint8_t address, uint8_t pin) {
  uint8_t port = pin / 8; // Determine which port (A or B)
  uint8_t pinMask = 1 << (pin % 8); // Create a mask for the specific pin

  uint8_t gpintenReg = (port == 0) ? GPINTENA : GPINTENB; // Select the appropriate GPINTEN register
  uint8_t currentIntEn = mcp23017_readRegister(address, gpintenReg); // Read the current interrupt enable setting

  currentIntEn |= pinMask; // Enable interrupt on the pin

  mcp23017_writeRegister(address, gpintenReg, currentIntEn); // Write the new interrupt enable setting
}

// Set the interrupt mode (change, falling edge, rising edge)
void mcp23017_interruptMode(uint8_t address, uint8_t pin, uint8_t mode) {
  uint8_t port = pin / 8; // Determine which port (A or B)
  uint8_t pinMask = 1 << (pin % 8); // Create a mask for the specific pin

  uint8_t intconReg = (port == 0) ? INTCONA : INTCONB; // Select the appropriate INTCON register
  uint8_t defvalReg = (port == 0) ? DEFVALA : DEFVALB; // Select the appropriate DEFVAL register

  uint8_t currentIntCon = mcp23017_readRegister(address, intconReg); // Read the current interrupt control setting
  uint8_t currentDefVal = mcp23017_readRegister(address, defvalReg); // Read the current default value setting

  if (mode == CHANGE) {
    currentIntCon &= ~pinMask; // Set to change
  } else if (mode == FALLING) {
    currentIntCon |= pinMask; // Set to compare against default value
    currentDefVal |= pinMask; // Default value is high
  } else if (mode == RISING) {
    currentIntCon |= pinMask; // Set to compare against default value
    currentDefVal &= ~pinMask; // Default value is low
  }

  mcp23017_writeRegister(address, intconReg, currentIntCon); // Write the new interrupt control setting
  mcp23017_writeRegister(address, defvalReg, currentDefVal); // Write the new default value setting
}

// Clear the interrupt flag on a specific pin
void mcp23017_interruptClear(uint8_t address, uint8_t pin) {
  uint8_t port = pin / 8; // Determine which port (A or B)
  uint8_t pinMask = 1 << (pin % 8); // Create a mask for the specific pin

  uint8_t intfReg = (port == 0) ? INTFA : INTFB; // Select the appropriate INTF register

  uint8_t currentIntf = mcp23017_readRegister(address, intfReg); // Read the current interrupt flag setting
  currentIntf &= ~pinMask; // Clear the interrupt flag

  mcp23017_writeRegister(address, intfReg, currentIntf); // Write the new interrupt flag setting
}

// Attach an interrupt service routine (ISR) to a specific pin
void mcp23017_attachInterrupt(uint8_t address, uint8_t pin, void (*isr)(void), uint8_t mode) {
  mcp23017_interruptEnable(address, pin);
  mcp23017_interruptMode(address, pin, mode);
  attachInterrupt(digitalPinToInterrupt(pin), isr, mode);
}

// Detach an interrupt service routine from a specific pin
void mcp23017_detachInterrupt(uint8_t address, uint8_t pin) {
  detachInterrupt(digitalPinToInterrupt(pin));
  mcp23017_interruptClear(address, pin);
  mcp23017_interruptEnable(address, pin); // Disable interrupt on the pin
}

// Bulk read all ports at once
uint16_t mcp23017_bulkRead(uint8_t address) {
  uint8_t gpioA = mcp23017_readRegister(address, GPIOA);
  uint8_t gpioB = mcp23017_readRegister(address, GPIOB);
  return (gpioB << 8) | gpioA;
}

// Bulk write all ports at once
void mcp23017_bulkWrite(uint8_t address, uint16_t value) {
  mcp23017_writeRegister(address, GPIOA, value & 0xFF);
  mcp23017_writeRegister(address, GPIOB, (value >> 8) & 0xFF);
}

// Inverse the bits of the value
uint16_t mcp23017_inverseBits(uint16_t value) {
  return ~value;
}

// Get the state of a specific pin from the bulk read result
uint8_t mcp23017_getPinStateFromBulkRead(uint16_t bulkResult, uint8_t pin) {
  uint8_t port = pin / 8; // Determine which port (A or B)
  uint8_t pinMask = 1 << (pin % 8); // Create a mask for the specific pin

  uint16_t portValue = (port == 0) ? (bulkResult & 0xFF) : ((bulkResult >> 8) & 0xFF);

  return (portValue & pinMask) ? HIGH : LOW;
}


/*
// Example usage
void setup() {
  Wire.begin(); // Initialize the I2C bus
  Serial.begin(115200); // Initialize serial communication

  // Define I2C addresses for the MCP23017 devices
  uint8_t addresses[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25};

  // Initialize each MCP23017
  for (uint8_t i = 0; i < 6; i++) {
    mcp23017_begin(addresses[i]);
  }

  // Set pin 0 of the first MCP23017 as an output and set it high
  mcp23017_pinMode(addresses[0], 0, OUTPUT);
  mcp23017_digitalWrite(addresses[0], 0, HIGH);

  // Set pin 1 of the second MCP23017 as an input with pull-up and enable interrupt
  mcp23017_pinMode(addresses[1], 1, INPUT_PULLUP);
  mcp23017_attachInterrupt(addresses[1], 1, handleInterrupt, FALLING);
}

void loop() {
  // Toggle pin 0 of the first MCP23017 every second
  mcp23017_digitalWrite(addresses[0], 0, !mcp23017_digitalRead(addresses[0], 0));
  delay(1000);

  // Read pin 1 of the second MCP23017 and print its state
  Serial.println(mcp23017_digitalRead(addresses[1], 1) ? "High" : "Low");
  delay(1000);

  // Example of using bulk read, get pin state, inverse bits, and bulk write
  uint16_t value = mcp23017_bulkRead(addresses[0]);
  Serial.print("Bulk Read Value: ");
  Serial.println(value, BIN);

  // Get the state of pin 0 from the bulk read result
  uint8_t pin0State = mcp23017_getPinStateFromBulkRead(value, 0);
  Serial.print("Pin 0 State: ");
  Serial.println(pin0State ? "High" : "Low");

  // Inverse the bits and write back
  value = mcp23017_inverseBits(value);
  mcp23017_bulkWrite(addresses[0], value);
  Serial.print("Inversed and Written Value: ");
  Serial.println(value, BIN);
}

void handleInterrupt() {
  Serial.println("Interrupt detected!");


*/
