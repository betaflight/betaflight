
// For generic API use, but here for now

bool smartAudioInit();
void smartAudioProcess(uint32_t);

#ifdef OSD

// API for BFOSD3.0

uint16_t smartAudioSmartbaud;

uint16_t saerr_badpre;
uint16_t saerr_badlen;
uint16_t saerr_crcerr;
uint16_t saerr_oooresp;

char smartAudioStatusString[31];

uint8_t smartAudioOpModel;
uint8_t smartAudioStatus;
uint8_t smartAudioBand;
uint8_t smartAudioChan;
uint16_t smartAudioFreq;
uint8_t smartAudioPower;
uint8_t smartAudioTxMode;
uint8_t smartAudioPitFMode;
uint16_t smartAudioORFreq;

long smartAudioConfigureOpModelByGvar(displayPort_t *, void *self);
long smartAudioConfigurePitFModeByGvar(displayPort_t *, void *self);
long smartAudioConfigureBandByGvar(displayPort_t *, void *self);
long smartAudioConfigureChanByGvar(displayPort_t *, void *self);
long smartAudioConfigurePowerByGvar(displayPort_t *, void *self);
long smartAudioSetTxModeByGvar(displayPort_t *, void *self);

extern OSD_Entry cmsx_menuVtxSmartAudio[];
#endif
