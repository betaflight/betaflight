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

mspRxBuf = {}
mspRxIdx = 1
mspRxCRC = 0
mspStarted = false

-- Stats
requestsSent    = 0
repliesReceived = 0

mspReceivedReply_cnt = 0
mspReceivedReply_cnt1 = 0
mspReceivedReply_cnt2 = 0
mspReceivedReply_cnt3 = 0

lastReqTS = 0

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

local function mspReceivedReply(payload)

   -- TODO: MSP checksum checking

   mspReceivedReply_cnt = mspReceivedReply_cnt + 1
   
   local idx      = 1
   local head     = payload[idx]
   local err_flag = (bit32.band(head,0x20) ~= 0)
   idx = idx + 1

   if err_flag then
      -- error flag set
      mspStarted = false

      mspReceivedReply_cnt1 = mspReceivedReply_cnt1 + 1

      -- return error
      -- CRC checking missing

      --return bit32.band(payload[idx],0xFF)
      return nil
   end
   
   local start = (bit32.band(head,0x10) ~= 0)
   local seq   = bit32.band(head,0x0F)

   if start then
      -- start flag set
      mspRxIdx = 1
      mspRxBuf = {}

      mspRxSize = payload[idx]
      mspRxCRC  = mspRxSize
      idx = idx + 1
      mspStarted = true
      
      mspReceivedReply_cnt2 = mspReceivedReply_cnt2 + 1

   elseif not mspStarted then
      mspReceivedReply_cnt3 = mspReceivedReply_cnt3 + 1
      return nil
   -- TODO: add sequence number checking
   -- elseif ...
   end

   while (idx <= 6) and (mspRxIdx <= mspRxSize) do
      mspRxBuf[mspRxIdx] = payload[idx]
      mspRxCRC = bit32.bxor(mspRxCRC,payload[idx])
      mspRxIdx = mspRxIdx + 1
      idx = idx + 1
   end

   if idx > 6 then
      lastRxSeq = seq
      return
   end

   -- check CRC
   if mspRxCRC ~= payload[idx] then
      mspStarted = false
   end

   repliesReceived = repliesReceived + 1
   mspStarted = false
   return mspRxBuf
end

local function pollReply()
   local sensorId, frameId, dataId, value = sportTelemetryPop()
   if sensorId == REMOTE_SENSOR_ID and frameId == REPLY_FRAME_ID then

      local payload = {}
      payload[1] = bit32.band(dataId,0xFF)
      dataId = bit32.rshift(dataId,8)
      payload[2] = bit32.band(dataId,0xFF)

      payload[3] = bit32.band(value,0xFF)
      value = bit32.rshift(value,8)
      payload[4] = bit32.band(value,0xFF)
      value = bit32.rshift(value,8)
      payload[5] = bit32.band(value,0xFF)
      value = bit32.rshift(value,8)
      payload[6] = bit32.band(value,0xFF)

      return mspReceivedReply(payload)
   end
end

local function run(event)

   local now = getTime()

   if event == EVT_MINUS_FIRST or event == EVT_ROT_LEFT or event == EVT_MINUS_REPT then
      requestsSent    = 0
      repliesReceived = 0
      mspReceivedReply_cnt = 0
      mspReceivedReply_cnt1 = 0
      mspReceivedReply_cnt2 = 0
      mspReceivedReply_cnt3 = 0
   end

   lcd.clear()

   lcd.drawText(1,11,"Requests:",0)
   lcd.drawNumber(60,11,requestsSent)
   
   lcd.drawText(1,21,"Replies:",0)
   lcd.drawNumber(60,21,repliesReceived)

   lcd.drawText(1,31,"cnt:",0)
   lcd.drawNumber(30,31,mspReceivedReply_cnt)

   lcd.drawText(1,41,"cnt1:",0)
   lcd.drawNumber(30,41,mspReceivedReply_cnt1)

   lcd.drawText(71,31,"cnt2:",0)
   lcd.drawNumber(100,31,mspReceivedReply_cnt2)

   lcd.drawText(71,41,"cnt3:",0)
   lcd.drawNumber(100,41,mspReceivedReply_cnt3)

   -- last request is at least 2s old
   if lastReqTS + 200 <= now then
      mspSendRequest(117) -- MSP_PIDNAMES
      lastReqTS = now
   end

   pollReply()
end

return {run=run}
