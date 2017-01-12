
struct vtxVTable_s;
typedef struct vtxDevice_s {
    const struct vtxVTable_s *vTable;
    uint8_t numBand;
    uint8_t numChan;
    uint8_t numPower;

    uint16_t *freqTable;

    // Config variable?
    uint8_t opModel;     // Power up in: 0 = transmitting, 1 = pit mode

    // CMS only?
    uint8_t curBand;
    uint8_t curChan;
    uint8_t curPower;
    uint8_t curPitState; // 0 = PIT, 1 = non-PIT

    // CMS only?
    char *bandNames;
    char *chanNames;
    char *powerNames;
} vtxDevice_t;

typedef struct vtxVTable_s {
    void (*setBandChan)(uint8_t band, uint8_t chan);
    void (*setFreq)(uint16_t freq);
    void (*setRFPower)(uint8_t level); // 0 = OFF, 1~ = device dep.
    void (*setPitmode)(uint8_t onoff);
} vtxVTable_t;

// PIT mode is defined as LOWEST POSSIBLE RF POWER.
// - It can be a dedicated mode, or lowest RF power possible.
// - It is *NOT* RF on/off control.

bool vtx58_Freq2Bandchan(uint16_t freq, uint8_t *pBand, uint8_t *pChan);
