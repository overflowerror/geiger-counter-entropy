#define GEIGER_PIN (4)

#define ENTROPY_BITS_PER_CLICK (3)
#define ENTROPY_BITS_PER_CLICK_MASK ((1 << ENTROPY_BITS_PER_CLICK) - 1)

#define ENTROPY_BUFFER_SIZE_BYTES (8)
#define ENTROPY_BUFFER_SIZE_BITS (ENTROPY_BUFFER_SIZE_BYTES * 8)

#define ENTROPY_BITS_FOR_FLUSH (8)

#define FLUSH_AS_ASCII (true)

static inline void clearISR() {
  unsigned long gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
}

static unsigned long last = 0;
static uint8_t entropyBuffer[ENTROPY_BUFFER_SIZE_BYTES];
static int bufferState = 0;
static volatile int entropyBits = 0;
static volatile int newBits = 0;

static void ICACHE_RAM_ATTR geigerISR() {
  unsigned long current = millis();
  int entropy = (current - last) & ENTROPY_BITS_PER_CLICK_MASK;
  last = current;

  int currentBufferByte = bufferState / 8;
  int positionInBufferByte = bufferState % 8;
  int numberOfBitsInByte = (8 - positionInBufferByte);
  if (numberOfBitsInByte > ENTROPY_BITS_PER_CLICK) {
    numberOfBitsInByte = ENTROPY_BITS_PER_CLICK;
  }
  int bitmaskForBitsInBytes = (1 << numberOfBitsInByte) - 1;
  
  entropyBuffer[currentBufferByte] &= ~(ENTROPY_BITS_PER_CLICK_MASK << positionInBufferByte);
  entropyBuffer[currentBufferByte] |= (entropy & bitmaskForBitsInBytes) << positionInBufferByte;

  positionInBufferByte += ENTROPY_BITS_PER_CLICK;

  // more than 8 bits per click is not reasonable
  // (would be > 1/4 seconds of precision in timing)
  if (positionInBufferByte > 8) {
    currentBufferByte = (currentBufferByte + 1) % ENTROPY_BUFFER_SIZE_BYTES;
    numberOfBitsInByte = (positionInBufferByte % 8) + 1;
    positionInBufferByte = 0;
    bitmaskForBitsInBytes = (1 << numberOfBitsInByte) - 1;

    entropy >>= (ENTROPY_BITS_PER_CLICK - numberOfBitsInByte);
    
    entropyBuffer[currentBufferByte] &= ~(bitmaskForBitsInBytes);
    entropyBuffer[currentBufferByte] |= entropy;
  }

  bufferState = (bufferState + ENTROPY_BITS_PER_CLICK) % ENTROPY_BUFFER_SIZE_BITS;

  newBits += ENTROPY_BITS_PER_CLICK;
  entropyBits += ENTROPY_BITS_PER_CLICK;
  if (entropyBits > ENTROPY_BUFFER_SIZE_BITS) {
    entropyBits = ENTROPY_BUFFER_SIZE_BITS;
  }
  
  clearISR();
}

static uint8_t getRandomByte() {
  while (entropyBits < 8) {
    delay(1); // delay resets watchdog
  }

  // disable interrupts modify global state
  noInterrupts();
  int localBufferState = bufferState;
  int localEntropyBits = entropyBits;
  entropyBits -= 8;
  newBits = 0;
  interrupts();
  
  uint8_t value = 0;
  int startBit = (localBufferState - localEntropyBits + ENTROPY_BUFFER_SIZE_BITS) % ENTROPY_BUFFER_SIZE_BITS;
  int currentByte = startBit / 8;
  startBit %= 8;
  int numberOfBits = 8 - startBit;
  int bitMask = (1 << numberOfBits) - 1;

  value |= (entropyBuffer[currentByte] & (bitMask << startBit)) >> startBit;

  numberOfBits = 8 - numberOfBits;
  bitMask = (1 << numberOfBits) - 1;
  currentByte = (currentByte + 1) % ENTROPY_BUFFER_SIZE_BYTES;

  value |= (entropyBuffer[currentByte] & (bitMask)) << (8 - numberOfBits);

  noInterrupts();
  // too much new entropy data
  // there is the possibility of duplicate sequences
  // -> reset current entropy bits to zero
  if (newBits > ENTROPY_BUFFER_SIZE_BITS - 8) {
    entropyBits = 0;
  }
  interrupts();

  return value;
}

void setup() {
  Serial.begin(9600);
  
  pinMode(GEIGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), geigerISR, RISING);
}

void loop() {
  Serial.print("waiting for entropy: ");
  Serial.println(getRandomByte());
}
