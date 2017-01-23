--
-- MSP/SPORT code
--

-- Protocol version
SPORT_MSP_VERSION = bit32.lshift(1,5)

SPORT_MSP_STARTFLAG = bit32.lshift(1,4)

-- Sensor ID used by the local LUA script
LOCAL_SENSOR_ID  = 0x0D

-- Sensor ID used by the MSP server (BF, CF, MW, etc...)
REMOTE_SENSOR_ID = 0x1B

REQUEST_FRAME_ID = 0x30
REPLY_FRAME_ID   = 0x32

-- Sequence number for next MSP/SPORT packet
local sportMspSeq = 0
local sportMspRemoteSeq = 0

local mspRxBuf = {}
local mspRxIdx = 1
local mspRxCRC = 0
local mspStarted = false
local mspLastReq = 0

-- Stats
mspRequestsSent    = 0
mspRepliesReceived = 0
mspPkRxed = 0
mspErrorPk = 0
mspStartPk = 0
mspOutOfOrder = 0
mspCRCErrors = 0

local function mspResetStats()
   mspRequestsSent    = 0
   mspRepliesReceived = 0
   mspPkRxed = 0
   mspErrorPk = 0
   mspStartPk = 0
   mspOutOfOrderPk = 0
   mspCRCErrors = 0
end

local mspTxBuf = {}
local mspTxIdx = 1
local mspTxCRC = 0

local mspTxPk = 0

local function mspSendSport(payload)

   local dataId = 0
   dataId = payload[1] + bit32.lshift(payload[2],8)

   local value = 0
   value = payload[3] + bit32.lshift(payload[4],8)
      + bit32.lshift(payload[5],16) + bit32.lshift(payload[6],24)

   local ret = sportTelemetryPush(LOCAL_SENSOR_ID, REQUEST_FRAME_ID, dataId, value)
   if ret then
      mspTxPk = mspTxPk + 1
   end
end

local function mspProcessTxQ()

   if (#(mspTxBuf) == 0) then
      return false
   end

   if not sportTelemetryPush() then
      return true
   end

   local payload = {}
   payload[1] = sportMspSeq + SPORT_MSP_VERSION
   sportMspSeq = bit32.band(sportMspSeq + 1, 0x0F)

   if mspTxIdx == 1 then
      -- start flag
      payload[1] = payload[1] + SPORT_MSP_STARTFLAG
   end

   local i = 2
   while (i <= 6) do
      payload[i] = mspTxBuf[mspTxIdx]
      mspTxIdx = mspTxIdx + 1
      mspTxCRC = bit32.bxor(mspTxCRC,payload[i])
      i = i + 1
      if mspTxIdx > #(mspTxBuf) then
         break
      end
   end

   if i <= 6 then
      payload[i] = mspTxCRC
      i = i + 1

      -- zero fill
      while i <= 6 do
         payload[i] = 0
         i = i + 1
      end

      mspSendSport(payload)
      
      mspTxBuf = {}
      mspTxIdx = 1
      mspTxCRC = 0
      
      return false
   end
      
   mspSendSport(payload)
   return true
end

local function mspSendRequest(cmd,payload)

   -- busy
   if #(mspTxBuf) ~= 0 then
      return nil
   end

   mspTxBuf[1] = #(payload)
   mspTxBuf[2] = bit32.band(cmd,0xFF)  -- MSP command

   for i=1,#(payload) do
      mspTxBuf[i+2] = payload[i]
   end
   
   mspLastReq = cmd
   mspRequestsSent = mspRequestsSent + 1
   return mspProcessTxQ()
end

local function mspReceivedReply(payload)

   mspPkRxed = mspPkRxed + 1
   
   local idx      = 1
   local head     = payload[idx]
   local err_flag = (bit32.band(head,0x20) ~= 0)
   idx = idx + 1

   if err_flag then
      -- error flag set
      mspStarted = false

      mspErrorPk = mspErrorPk + 1

      -- return error
      -- CRC checking missing

      --return payload[idx]
      return nil
   end
   
   local start = (bit32.band(head,0x10) ~= 0)
   local seq   = bit32.band(head,0x0F)

   if start then
      -- start flag set
      mspRxIdx = 1
      mspRxBuf = {}

      mspRxSize = payload[idx]
      mspRxCRC  = bit32.bxor(mspRxSize,mspLastReq)
      idx = idx + 1
      mspStarted = true
      
      mspStartPk = mspStartPk + 1

   elseif not mspStarted then
      mspOutOfOrder = mspOutOfOrder + 1
      return nil

   elseif bit32.band(sportMspRemoteSeq + 1, 0x0F) ~= seq then
      mspOutOfOrder = mspOutOfOrder + 1
      mspStarted = false
      return nil
   end

   while (idx <= 6) and (mspRxIdx <= mspRxSize) do
      mspRxBuf[mspRxIdx] = payload[idx]
      mspRxCRC = bit32.bxor(mspRxCRC,payload[idx])
      mspRxIdx = mspRxIdx + 1
      idx = idx + 1
   end

   if idx > 6 then
      sportMspRemoteSeq = seq
      return true
   end

   -- check CRC
   if mspRxCRC ~= payload[idx] then
      mspStarted = false
      mspCRCErrors = mspCRCErrors + 1
      return nil
   end

   mspRepliesReceived = mspRepliesReceived + 1
   mspStarted = false
   return mspRxBuf
end

local function mspPollReply()
   while true do
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

         local ret = mspReceivedReply(payload)
         if type(ret) == "table" then
            return mspLastReq,ret
         end
      else
         break
      end
   end

   return nil
end

--
-- End of MSP/SPORT code
--

-- getter
local MSP_RC_TUNING     = 111
local MSP_PID           = 112

-- setter
local MSP_SET_PID       = 202
local MSP_SET_RC_TUNING = 204

-- BF specials
local MSP_PID_ADVANCED     = 94
local MSP_SET_PID_ADVANCED = 95

local MSP_EEPROM_WRITE = 250

local REQ_TIMEOUT = 80 -- 800ms request timeout

--local PAGE_REFRESH = 1
local PAGE_DISPLAY = 2
local EDITING      = 3
local PAGE_SAVING  = 4
local MENU_DISP    = 5

local gState = PAGE_DISPLAY

local SetupPages = {
   {
      title = "PIDs",
      text = {
         { t = "Roll",  x = 21,  y = 14 },
         { t = "Pitch", x = 75,  y = 14 },
         { t = "Yaw",   x = 129, y = 14 },
      },
      fields = {
         -- ROLL
         { t = "P", x = 25,  y = 24, i=1 },
         { t = "I", x = 25,  y = 34, i=2 },
         { t = "D", x = 25,  y = 44, i=3 },
         -- PITCH
         { t = "P", x = 80,  y = 24, i=4 },
         { t = "I", x = 80,  y = 34, i=5 },
         { t = "D", x = 80,  y = 44, i=6 },
         -- YAW
         { t = "P", x = 135, y = 24, i=7 },
         { t = "I", x = 135, y = 34, i=8 },
         --{ t = "D", x = 135, y = 44, i=9 },
      },
      read  = MSP_PID,
      write = MSP_SET_PID,
   },
   {
      title = "Rates",
      text = {
         { t = "Super rates", x = 5,  y = 14 },
         { t = "RC",          x = 80, y = 14 }
      },
      fields = {
         -- Super Rate
         { t = "Roll",  x = 10,  y = 24, sp = 40, i=3 },
         { t = "Pitch", x = 10,  y = 34, sp = 40, i=4 },

         -- Roll + Pitch
         { t = "Rate",  x = 75,  y = 29, sp = 30, i=1 },
         { t = "Expo",  x = 135, y = 29, sp = 30, i=2 },

         -- Yaw
         { t = "Yaw",   x = 10,  y = 48, sp = 40, i=5 },
         { t = "Rate",  x = 75,  y = 48, sp = 30, i=12 },
         { t = "Expo",  x = 135, y = 48, sp = 30, i=11 },
      },
      read  = MSP_RC_TUNING,
      write = MSP_SET_RC_TUNING,
   }
}

local currentPage = 1
local currentLine = 1
local saveTS = 0
local saveRetries = 0

local function saveSettings(new)
   local page = SetupPages[currentPage]
   if page.values then
      mspSendRequest(page.write,page.values)
      saveTS = getTime()
      if gState == PAGE_SAVING then
         saveRetries = saveRetries + 1
      else
         gState = PAGE_SAVING
      end
   end
end

local function invalidatePages()
   for i=1,#(SetupPages) do
      local page = SetupPages[i]
      page.values = nil
   end
   gState = PAGE_DISPLAY
   saveTS = 0
end

local menuList = {

   { t = "save page",
     f = saveSettings },

   { t = "reload",
     f = invalidatePages }
}

local telemetryScreenActive = false
local menuActive = false

local function processMspReply(cmd,rx_buf)

   if cmd == nil or rx_buf == nil then
      return
   end

   local page = SetupPages[currentPage]

   -- ignore replies to write requests for now
   if cmd == page.write then
      mspSendRequest(MSP_EEPROM_WRITE,{})
      return
   end

   if cmd == MSP_EEPROM_WRITE then
      gState = PAGE_DISPLAY
      page.values = nil
      saveTS = 0
      return
   end
   
   if cmd ~= page.read then
      return
   end

   if #(rx_buf) > 0 then
      page.values = {}
      for i=1,#(rx_buf) do
         page.values[i] = rx_buf[i]
      end
   end
end
   
local function MaxLines()
   return #(SetupPages[currentPage].fields)
end

local function incPage(inc)
   currentPage = currentPage + inc
   if currentPage > #(SetupPages) then
      currentPage = 1
   elseif currentPage < 1 then
      currentPage = #(SetupPages)
   end
   currentLine = 1
end

local function incLine(inc)
   currentLine = currentLine + inc
   if currentLine > MaxLines() then
      currentLine = 1
   elseif currentLine < 1 then
      currentLine = MaxLines()
   end
end

local function incMenu(inc)
   menuActive = menuActive + inc
   if menuActive > #(menuList) then
      menuActive = 1
   elseif menuActive < 1 then
      menuActive = #(menuList)
   end
end

local function requestPage(page)
   if page.read and ((page.reqTS == nil) or (page.reqTS + REQ_TIMEOUT <= getTime())) then
      page.reqTS = getTime()
      mspSendRequest(page.read,{})
   end
end

local function drawScreen(page,page_locked)

   local screen_title = page.title

   lcd.drawScreenTitle('Betaflight setup: '..screen_title,currentPage,#(SetupPages))

   for i=1,#(page.text) do
      local f = page.text[i]
      lcd.drawText(f.x, f.y, f.t, text_options)
   end
   
   for i=1,#(page.fields) do
      local f = page.fields[i]

      local text_options = 0
      if i == currentLine then
         text_options = INVERS
         if gState == EDITING then
            text_options = text_options + BLINK
         end
      end

      lcd.drawText(f.x, f.y, f.t .. ":", 0)

      -- draw some value
      local spacing = 20
      if f.sp ~= nil then
         spacing = f.sp
      end

      local idx = f.i or i
      if page.values and page.values[idx] then
         lcd.drawText(f.x + spacing, f.y, page.values[idx], text_options)
      else
         lcd.drawText(f.x + spacing, f.y, "---", text_options)
      end
   end
end

local function clipValue(val)
   if val < 0 then
      val = 0
   elseif val > 255 then
      val = 255
   end

   return val
end

local function getCurrentField()
   local page = SetupPages[currentPage]
   return page.fields[currentLine]
end

local function incValue(inc)
   local page = SetupPages[currentPage]
   local field = page.fields[currentLine]
   local idx = field.i or currentLine
   page.values[idx] = clipValue(page.values[idx] + inc)
end

local function drawMenu()
   local x = 40
   local y = 12
   local w = 120
   local h = #(menuList) * 8 + 6
   lcd.drawFilledRectangle(x,y,w,h,ERASE)
   lcd.drawRectangle(x,y,w-1,h-1,SOLID)
   lcd.drawText(x+4,y+3,"Menu:")

   for i,e in ipairs(menuList) do
      if menuActive == i then
         lcd.drawText(x+36,y+(i-1)*8+3,e.t,INVERS)
      else
         lcd.drawText(x+36,y+(i-1)*8+3,e.t)
      end
   end
end

local EVT_MENU_LONG = bit32.bor(bit32.band(EVT_MENU_BREAK,0x1f),0x80)

local lastRunTS = 0

local function run(event)

   local now = getTime()

   -- if lastRunTS old than 500ms
   if lastRunTS + 50 < now then
      invalidatePages()
   end
   lastRunTS = now

   -- TODO: implement retry + retry counter
   if (gState == PAGE_SAVING) and (saveTS + 150 < now) then
      if saveRetries < 2 then
         saveSettings()
      else
         -- two retries and still no success
         gState = PAGE_DISPLAY
         saveTS = 0
      end
   end
   
   if #(mspTxBuf) > 0 then
      mspProcessTxQ()
   end

   -- navigation
   if event == EVT_MENU_LONG then
      menuActive = 1
      gState = MENU_DISP

   -- menu is currently displayed
   elseif gState == MENU_DISP then
      if event == EVT_EXIT_BREAK then
         gState = PAGE_DISPLAY
      elseif event == EVT_PLUS_BREAK or event == EVT_ROT_LEFT then
         incMenu(-1)
      elseif event == EVT_MINUS_BREAK or event == EVT_ROT_RIGHT then
         incMenu(1)
      elseif event == EVT_ENTER_BREAK then
         gState = PAGE_DISPLAY
         menuList[menuActive].f()
      end
   -- normal page viewing
   elseif gState <= PAGE_DISPLAY then
      if event == EVT_MENU_BREAK then
         incPage(1)
      elseif event == EVT_PLUS_BREAK or event == EVT_ROT_LEFT then
         incLine(-1)
      elseif event == EVT_MINUS_BREAK or event == EVT_ROT_RIGHT then
         incLine(1)
      elseif event == EVT_ENTER_BREAK then
         local page = SetupPages[currentPage]
         local field = page.fields[currentLine]
         local idx = field.i or currentLine
         if page.values and page.values[idx] then
            gState = EDITING
         end
      end
   -- editing value
   elseif gState == EDITING then
      if (event == EVT_EXIT_BREAK) or (event == EVT_ENTER_BREAK) then
         gState = PAGE_DISPLAY
      elseif event == EVT_PLUS_FIRST or event == EVT_PLUS_REPT or event == EVT_ROT_RIGHT then
         incValue(1)
      elseif event == EVT_MINUS_FIRST or event == EVT_MINUS_REPT or event == EVT_ROT_LEFT then
         incValue(-1)
      end
   end

   local page = SetupPages[currentPage]
   local page_locked = false

   if not page.values then
      -- request values
      requestPage(page)
      page_locked = true
   end

   -- draw screen
   lcd.clear()
   drawScreen(page,page_locked)
   
   -- do we have valid telemetry data?
   if getValue("RSSI") == 0 then
      -- No!
      lcd.drawText(70,55,"No telemetry",BLINK)
      invalidatePages()
   end

   if gState == MENU_DISP then
      drawMenu()
   elseif gState == PAGE_SAVING then
      lcd.drawFilledRectangle(40,12,120,30,ERASE)
      lcd.drawRectangle(40,12,120,30,SOLID)
      lcd.drawText(44,15,"Saving...",DBLSIZE + BLINK)
   end

   processMspReply(mspPollReply())
   return 0
end

return {run=run}
