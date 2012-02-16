volatile uint16_t rcValue[18] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#if defined(SERIAL_SUM_PPM)
  static uint8_t rcChannel[8] = {SERIAL_SUM_PPM};
#elif defined(SBUS)
  // for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
  static uint8_t rcChannel[18] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
  static uint16_t sbusIndex=0;
#else
  static uint8_t rcChannel[8]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
#endif
#if defined(SPEKTRUM)
  #define SPEK_MAX_CHANNEL 7
  #define SPEK_FRAME_SIZE 16
  #if (SPEKTRUM == 1024)
    #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
  #endif
  #if (SPEKTRUM == 2048)
    #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
  #endif
  volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
#endif

// Configure each rc pin for PCINT
void configureReceiver() {
  #if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS) && !defined(BTSERIAL)
    #if defined(PROMINI)
      // PCINT activated only for specific pin inside [D0-D7]  , [D2 D4 D5 D6 D7] for this multicopter
      PORTD   = (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTD (no high impedence PINs)
      PCMSK2 |= (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7); 
      PCICR   = (1<<2) ; // PCINT activated only for the port dealing with [D0-D7] PINs on port B
      #if defined(RCAUXPIN)
        PCICR  |= (1<<0) ; // PCINT activated also for PINS [D8-D13] on port B
        #if defined(RCAUXPIN8)
          PCMSK0 = (1<<0);
        #endif
        #if defined(RCAUXPIN12)
          PCMSK0 = (1<<4);
        #endif
      #endif
    #endif
    #if defined(MEGA)
      // PCINT activated only for specific pin inside [A8-A15]
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
      PORTK   = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTK
      PCMSK2 |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
      PCICR   = 1<<2; // PCINT activated only for PORTK dealing with [A8-A15] PINs
    #endif
  #endif
  #if defined(SERIAL_SUM_PPM)
    PPM_PIN_INTERRUPT;
  #endif
  #if defined (SPEKTRUM)
    SerialOpen(1,115200);
  #endif
  #if defined(SBUS)
    SerialOpen(1,100000);
  #endif
}

#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS) && !defined(BTSERIAL)
  ISR(PCINT2_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  
    #if defined(PROMINI)
      pin = PIND;             // PIND indicates the state of each PIN for the arduino port dealing with [D0-D7] digital pins (8 bits variable)
    #endif
    #if defined(MEGA)
      pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
    #endif
    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    
    // mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
    // chan = pin sequence of the port. chan begins at D2 and ends at D7
    if (mask & 1<<2)           //indicates the bit 2 of the arduino port [D0-D7], that is to say digital pin 2, if 1 => this pin has just changed
      if (!(pin & 1<<2)) {     //indicates if the bit 2 of the arduino port [D0-D7] is not at a high state (so that we match here only descending PPM pulse)
        dTime = cTime-edgeTime[2]; if (900<dTime && dTime<2200) rcValue[2] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
      } else edgeTime[2] = cTime;    // if the bit 2 of the arduino port [D0-D7] is at a high state (ascending PPM pulse), we memorize the time
    if (mask & 1<<4)   //same principle for other channels   // avoiding a for() is more than twice faster, and it's important to minimize execution time in ISR
      if (!(pin & 1<<4)) {
        dTime = cTime-edgeTime[4]; if (900<dTime && dTime<2200) rcValue[4] = dTime;
      } else edgeTime[4] = cTime;
    if (mask & 1<<5)
      if (!(pin & 1<<5)) {
        dTime = cTime-edgeTime[5]; if (900<dTime && dTime<2200) rcValue[5] = dTime;
      } else edgeTime[5] = cTime;
    if (mask & 1<<6)
      if (!(pin & 1<<6)) {
        dTime = cTime-edgeTime[6]; if (900<dTime && dTime<2200) rcValue[6] = dTime;
      } else edgeTime[6] = cTime;
    if (mask & 1<<7)
      if (!(pin & 1<<7)) {
        dTime = cTime-edgeTime[7]; if (900<dTime && dTime<2200) rcValue[7] = dTime;
      } else edgeTime[7] = cTime;
    #if defined(MEGA)
      if (mask & 1<<0)    
        if (!(pin & 1<<0)) {
          dTime = cTime-edgeTime[0]; if (900<dTime && dTime<2200) rcValue[0] = dTime; 
        } else edgeTime[0] = cTime; 
      if (mask & 1<<1)      
        if (!(pin & 1<<1)) {
          dTime = cTime-edgeTime[1]; if (900<dTime && dTime<2200) rcValue[1] = dTime; 
        } else edgeTime[1] = cTime;
      if (mask & 1<<3)
        if (!(pin & 1<<3)) {
          dTime = cTime-edgeTime[3]; if (900<dTime && dTime<2200) rcValue[3] = dTime;
        } else edgeTime[3] = cTime;
    #endif
    #if defined(FAILSAFE)
      if (mask & 1<<THROTTLEPIN) {    // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; }
    #endif
  }
  
  #if defined(RCAUXPIN)
  /* this ISR is a simplication of the previous one for PROMINI on port D
     it's simplier because we know the interruption deals only with on PIN:
     bit 0 of PORT B, ie Arduino PIN 8
     or bit 4 of PORTB, ie Arduino PIN 12
   => no need to check which PIN has changed */
  ISR(PCINT0_vect) {
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime;
  
    pin = PINB;
    sei();
    cTime = micros();
    #if defined(RCAUXPIN8)
     if (!(pin & 1<<0)) {     //indicates if the bit 0 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
    #endif
    #if defined(RCAUXPIN12)
     if (!(pin & 1<<4)) {     //indicates if the bit 4 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
    #endif
      dTime = cTime-edgeTime; if (900<dTime && dTime<2200) rcValue[0] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
    } else edgeTime = cTime;    // if the bit 2 is at a high state (ascending PPM pulse), we memorize the time
  }
  #endif
#endif

#if defined(SERIAL_SUM_PPM)
void rxInt() {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t chan = 0;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>3000) chan = 0;
  else {
    if(900<diff && diff<2200 && chan<8 ) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
      rcValue[chan] = diff;
      #if defined(FAILSAFE)
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // clear FailSafe counter - added by MIS  //incompatible to quadroppm
      #endif
    }
    chan++;
  }
}
#endif

#if defined(SPEKTRUM)
  ISR(SPEK_SERIAL_VECT) {
    uint32_t spekTime;
    static uint32_t spekTimeLast, spekTimeInterval;
    static uint8_t  spekFramePosition;
    spekTime=micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;
    if (spekTimeInterval > 5000) spekFramePosition = 0;
    spekFrame[spekFramePosition] = SPEK_DATA_REG;
    if (spekFramePosition == SPEK_FRAME_SIZE - 1) {
      rcFrameComplete = 1;
      #if defined(FAILSAFE)
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // clear FailSafe counter
      #endif
    } else {
      spekFramePosition++;
    }
  }
#endif

#if defined(SBUS)
void  readSBus(){
  #define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
  static uint16_t sbus[25]={0};
  while(SerialAvailable(1)){
    int val = SerialRead(1);
    if(sbusIndex==0 && val != SBUS_SYNCBYTE)
      continue;
    sbus[sbusIndex++] = val;
    if(sbusIndex==25){
      sbusIndex=0;
      rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+976; // Perhaps you may change the term "/2+976" -> center will be 1486
      rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+976; 
      rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+976; 
      rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+976; 
      rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+976; 
      rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+976;
      rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+976; 
      rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+976; // & the other 8 + 2 channels if you need them
      //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
      rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+976; 
      rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+976; 
      rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+976; 
      rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+976; 
      rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+976; 
      rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+976; 
      rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+976; 
      rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+976; 
      // now the two Digital-Channels
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      #if defined(FAILSAFE)
      if (!((sbus[23] >> 3) & 0x0001))
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;}   // clear FailSafe counter
      #endif
    }
  }        
}
#endif

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG; cli(); // Let's disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  #if defined(SPEKTRUM)
    static uint32_t spekChannelData[SPEK_MAX_CHANNEL];
    if (rcFrameComplete) {
      for (uint8_t b = 3; b < SPEK_FRAME_SIZE; b += 2) {
        uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> SPEK_CHAN_SHIFT);
        if (spekChannel < SPEK_MAX_CHANNEL) spekChannelData[spekChannel] = (long(spekFrame[b - 1] & SPEK_CHAN_MASK) << 8) + spekFrame[b];
      }
      rcFrameComplete = 0;
    }
  #endif
  SREG = oldSREG; sei();// Let's enable the interrupts
  #if defined(SPEKTRUM)
    static uint8_t spekRcChannelMap[SPEK_MAX_CHANNEL] = {1,2,3,0,4,5,6};
    if (chan >= SPEK_MAX_CHANNEL) {
      data = 1500;
    } else {
      #if (SPEKTRUM == 1024)
        data = 988 + spekChannelData[spekRcChannelMap[chan]];          // 1024 mode
      #endif
      #if (SPEKTRUM == 2048)
        data = 988 + (spekChannelData[spekRcChannelMap[chan]] >> 1);   // 2048 mode
      #endif
    }
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}
    
void computeRC() {
  static int16_t rcData4Values[8][4], rcDataMean[8];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a,ind;

  #if defined(SBUS)
    readSBus();
  #endif
  rc4ValuesIndex++;
  for (chan = 0; chan < 8; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcDataMean[chan] = 0;
    for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
    rcDataMean[chan]= (rcDataMean[chan]+2)/4;
    if ( rcDataMean[chan] < rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
    if ( rcDataMean[chan] > rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
  }
}

