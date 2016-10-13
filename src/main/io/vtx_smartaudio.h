
// For generic API use, but here for now

bool smartAudioInit();
void smartAudioProcess(uint32_t);

#ifdef OSD

// API for BFOSD3.0

char smartAudioStatusString[31];

uint8_t smartAudioBand;
uint8_t smartAudioChan;
uint8_t smartAudioPower;
uint8_t smartAudioMode;

void smartAudioConfigureBandByGvar(void *);
void smartAudioConfigureChanByGvar(void *);
void smartAudioConfigurePowerByGvar(void *);
void smartAudioSetModeByGvar(void *);
#endif
