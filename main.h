void killMotors(void);

void writeQCSR(void);

void addStickPos(int16_t *,int16_t *);

void readXbeeKey(int16_t *,int16_t *);

int16_t interpolateStickPosition(int32_t,int32_t,int32_t,int32_t,int32_t,int32_t,int32_t);

void writeChar2EEPROM(char c,uint16_t address);
//void sendStringViaUART(const char *,uint8_t);