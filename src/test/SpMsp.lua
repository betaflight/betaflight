--
-- Test script for the MSP/SPORT bridge
--

-- Protocol version
SPORT_MSP_VERSION = 1

-- Sensor ID used by the local LUA script
LOCAL_SENSOR_ID  = 0x0D

-- Sensor ID used by the MSP server (BF, CF, MW, etc...)
REMOTE_SENSOR_ID = 0x1B

REQUEST_FRAME_ID = 0x30
REPLY_FRAME_ID   = 0x32

-- Sequence number for next MSP/SPORT packet
sportMspSeq = 0

-- Stats
requestsSent    = 0
repliesReceived = 0

lastReqTS = 0

local function pollReply()
   local sensorId, frameId, dataId, value = sportTelemetryPop()
   if sensorId == REMOTE_SENSOR_ID and frameId == REPLY_FRAME_ID then
      repliesReceived = repliesReceived + 1
   end
end

local function mspSendRequest(cmd)

   local dataId = 0
   dataId = sportMspSeq                                -- sequence number
   dataId = dataId + bit32.lshift(1,4)                 -- start flag
   dataId = dataId + bit32.lshift(SPORT_MSP_VERSION,5) -- MSP/SPORT version
   -- size is 0 for now, no need to add it to dataId
   -- dataId = dataId + bit32.lshift(0,8)
   sportMspSeq = bit32.band(sportMspSeq + 1, 0x0F)
   
   local value = 0
   value = bit32.band(cmd,0xFF)        -- MSP command
   value = value + bit32.lshift(cmd,8) -- CRC

   requestsSent = requestsSent + 1
   return sportTelemetryPush(LOCAL_SENSOR_ID, REQUEST_FRAME_ID, dataId, value)
end

local function run(event)

   local now = getTime()

   if event == EVT_MINUS_FIRST or event == EVT_ROT_LEFT or event == EVT_MINUS_REPT then
      requestsSent    = 0
      repliesReceived = 0
   end

   lcd.clear()

   lcd.drawText(1,11,"Requests:",0)
   lcd.drawNumber(60,11,requestsSent)
   
   lcd.drawText(1,21,"Replies:",0)
   lcd.drawNumber(60,21,repliesReceived)

   -- last request is at least 2s old
   if lastReqTS + 200 <= now then
      mspSendRequest(117) -- MSP_PIDNAMES
      lastReqTS = now
   end

   pollReply()
end

return {run=run}
