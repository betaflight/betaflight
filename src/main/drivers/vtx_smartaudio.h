
// For generic API use, but here for now

typedef struct vtxPowerTable_s {
    char    *name;
    int16_t power;
    int16_t value;
} vtxPowerTable_t;

void smartAudioInit(void);
void smartAudioProcess(void);
void smartAudioSetPowerByIndex(uint8_t);
void smartAudioSetFreq(uint16_t);
void smartAudioSetBandChan(int, int);
