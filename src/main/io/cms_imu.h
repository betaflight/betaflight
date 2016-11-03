extern OSD_Entry cmsx_menuImu[];

// All of below should be gone.

extern OSD_Entry cmsx_menuPid[];
extern OSD_Entry cmsx_menuRc[];
extern OSD_Entry cmsx_menuRateExpo[];

void cmsx_PidRead(void);
void cmsx_PidWriteback(void);
void cmsx_RateExpoRead(void);
void cmsx_RateExpoWriteback(void);

