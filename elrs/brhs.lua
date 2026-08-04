-- ==========================================================================
-- FILE-SCOPED VARIABLES & CONFIGURATION (Shared across all functions)
-- ==========================================================================
-- Telemetry & Decoded data variables
local txBat, rxBat, rxBatMax, lq, rssi, alt, altRef, heading, headingRef
local homeBearing, homeDistance, satField, fm, pitch, roll
local gnssReliable, nSat
local latitude , longitude 

-- Battery alert state from FC (0 none / 1 low / 2 critical), carried on Bat%
local batteryAlertState

-- Persistent state tracking variables for audio alerts
local aAlertWelcomePlayed = false
local aAlertLastStart = nil
local aAlertLastNav = nil
local aAlertLastAlt = nil
local aAlertLastLand = nil
local aAlertLastMission = nil
local aAlertLastGnss = nil
local aAlertNextBatAlert = 0
local aAlertNextLinkAlert = 0  -- Timer tracking state for link quality alerts

-- ALERT CONFIGURATION SETTINGS

-- Battery alerting: the FC decides the tier (chemistry, curve, and latch all
-- live on the FC) and pushes it on the repurposed Bat% telemetry field.
-- The radio just plays the matching sound at the right cadence -- no thresholds.
local BAT_ALERT_NONE = 0
local BAT_ALERT_LOW  = 1
local BAT_ALERT_CRIT = 2

local aAlertBatInterval      = 2000 -- 20 seconds repeat interval (Low Battery)
local aAlertCritBatInterval  = 500  -- 5 seconds repeat interval (Critically low Battery)

-- Link Quality Thresholds and Timings
local aAlertLowLinkThreshold  = 70
local aAlertCritLinkThreshold = 40
local aAlertLinkInterval      = 1500  -- 15 seconds repeat interval (Low Link)
local aAlertCritLinkInterval  = 300   -- 3 seconds repeat interval (Critical Link)

-- VCP USB Params
local vcpUSBLastSend = 0
local NAV_WP_PAYLOAD_SIZE = 11
local CRSF_MSP_NAV_COMMAND = 0x7C
local mspRxBuffer = ""
local mspSequence = 0

-- Decoded Flight Mode Audio Mapping Tables
local aAlertStartModes = {
    ["O"] = "/SOUNDS/en/brhs/start/armed.wav",
    ["I"] = "/SOUNDS/en/brhs/start/idle.wav",
    ["S"] = "/SOUNDS/en/brhs/start/warm.wav",
    ["F"] = "/SOUNDS/en/brhs/start/fly.wav",
    ["C"] = "/SOUNDS/en/brhs/start/crash.wav",
}

local aAlertNavModes = {
    ["S"] = "/SOUNDS/en/brhs/nav/stab.wav",
    ["N"] = "/SOUNDS/en/brhs/nav/nav.wav",
    ["R"] = "/SOUNDS/en/brhs/nav/rth.wav",
	["C"] = "/SOUNDS/en/brhs/nav/rthCom.wav",
}

local aAlertAltModes = {
    ["B"] = "/SOUNDS/en/brhs/alt/altB.wav",
    ["T"] = "/SOUNDS/en/brhs/alt/altT.wav",
}

local aAlertLandModes = {
    ["L"] = "/SOUNDS/en/brhs/land/land.wav",
    ["F"] = "/SOUNDS/en/brhs/land/fly.wav",
}

local aAlertMissionModes = {
    ["M"] = "/SOUNDS/en/brhs/mission/miOn.wav",
    ["N"] = "/SOUNDS/en/brhs/mission/miOff.wav",
	["C"] = "/SOUNDS/en/brhs/mission/miCom.wav",
}

-- Constant lookup table for compass directions (allocated once in memory)
local DIRECTIONS = {
    "N", "NE", "E", "SE",
    "S", "SW", "W", "NW"
}

-- ==========================================================================
-- AUDIO & HELPER FUNCTIONS
-- ==========================================================================

local function getCompassDirection(angle)
    angle = angle % 360
    local index = math.floor((angle + 22.5) / 45) % 8
    return DIRECTIONS[index + 1]
end

-- Dedicated function to parse and trigger Flight Mode audio alerts
local function doFMAlert()
    -- Clean the FM string by removing hyphens/spaces and making it uppercase
    local cleanFm = string.gsub(string.upper(tostring(fm)), "%W", "")

    -- Only process individual flight mode states if telemetry returns a valid 4-character block
    if #cleanFm == 5 then
        local currentStart = string.sub(cleanFm, 1, 1)
        local currentNav   = string.sub(cleanFm, 2, 2)
        local currentAlt   = string.sub(cleanFm, 3, 3)
        local currentLand  = string.sub(cleanFm, 4, 4)
		local currentMission  = string.sub(cleanFm, 5, 5)

        -- Initialize tracking states on first valid telemetry contact to prevent startup spam
        if aAlertLastStart == nil then aAlertLastStart = currentStart end
        if aAlertLastNav == nil then aAlertLastNav = currentNav end
        if aAlertLastAlt == nil then aAlertLastAlt = currentAlt end
        if aAlertLastLand == nil then aAlertLastLand = currentLand end
		if aAlertLastMission == nil then aAlertLastMission = currentMission end

        -- [Start State] Sub-alert Check
        if currentStart ~= aAlertLastStart then
            if aAlertStartModes[currentStart] then
                playFile(aAlertStartModes[currentStart])
            end
            aAlertLastStart = currentStart
        end

        -- [NavState] Sub-alert Check
        if currentNav ~= aAlertLastNav then
            if aAlertNavModes[currentNav] then
                playFile(aAlertNavModes[currentNav])
            end
            aAlertLastNav = currentNav
        end

        -- [Alt hold Mode] Sub-alert Check
        if currentAlt ~= aAlertLastAlt then
            if aAlertAltModes[currentAlt] then
                playFile(aAlertAltModes[currentAlt])
            end
            aAlertLastAlt = currentAlt
        end

        -- [Land state] Sub-alert Check
        if currentLand ~= aAlertLastLand then
            if aAlertLandModes[currentLand] then
                playFile(aAlertLandModes[currentLand])
            end
            aAlertLastLand = currentLand
        end
		
		-- [Mission state] Sub-alert Check
        if currentMission ~= aAlertLastMission then
            if aAlertMissionModes[currentMission] then
                playFile(aAlertMissionModes[currentMission])
            end
            aAlertLastMission = currentMission
        end
		
    end
end

-- Dedicated function to manage dual-tier battery alerts.
-- The tier is decided on the FC and carried on the Bat% field (0/1/2);
-- this only plays the matching sound at the right repeat cadence.
local function doBatAlert()
    local currentTime = getTime() -- Internal clock (100 ticks = 1 second)

    -- Round + band the incoming state so a scaled/rounded telemetry value
    -- (e.g. 2.0, or a future higher state) still resolves to the right tier.
    local state = math.floor((batteryAlertState or 0) + 0.5)

    -- rxBat > 1.5 confirms the telemetry link is live before trusting the state.
    if rxBat > 1.5 then
        if state >= BAT_ALERT_CRIT then
            -- Critical: repeat every 5 seconds
            if currentTime > aAlertNextBatAlert then
                playFile("/SOUNDS/en/brhs/bat/rxBatC.wav")
                aAlertNextBatAlert = currentTime + aAlertCritBatInterval
            end
        elseif state >= BAT_ALERT_LOW then
            -- Low: repeat every 20 seconds
            if currentTime > aAlertNextBatAlert then
                playFile("/SOUNDS/en/brhs/bat/rxBatL.wav")
                aAlertNextBatAlert = currentTime + aAlertBatInterval
            end
        else
            -- No alert (state 0), keep timer synchronized
            if currentTime > aAlertNextBatAlert then
                aAlertNextBatAlert = currentTime
            end
        end
    else
        -- Telemetry link down, keep timer synchronized
        if currentTime > aAlertNextBatAlert then
            aAlertNextBatAlert = currentTime
        end
    end
end

-- Dedicated function to manage dual-tier link quality alerts
local function doLinkAlert()
    local currentTime = getTime() -- Internal clock (100 ticks = 1 second)

    -- Only process alerts if telemetry connection is active
    if rxBat > 1.5 then
        if lq < aAlertCritLinkThreshold then
            -- Tier 1: Critical Link Quality Alert (< 40) every 3 seconds
            if currentTime > aAlertNextLinkAlert then
                playFile("/SOUNDS/en/brhs/link/LinkQC.wav")
                aAlertNextLinkAlert = currentTime + aAlertCritLinkInterval
            end
        elseif lq < aAlertLowLinkThreshold then
            -- Tier 2: Low Link Quality Alert (< 70) every 15 seconds
            if currentTime > aAlertNextLinkAlert then
                playFile("/SOUNDS/en/brhs/link/LinkQL.wav")
                aAlertNextLinkAlert = currentTime + aAlertLinkInterval
            end
        else
            -- Link quality is healthy, keep timer synchronized
            if currentTime > aAlertNextLinkAlert then
                aAlertNextLinkAlert = currentTime
            end
        end
    else
        -- Telemetry link down, keep timer synchronized
        if currentTime > aAlertNextLinkAlert then
            aAlertNextLinkAlert = currentTime
        end
    end
end

-- Dedicated function to manage GPS status transition alerts
local function doGnssAlert()
    -- Initialize GPS tracking state on first run
    if aAlertLastGnss == nil then aAlertLastGnss = gnssReliable end

    -- GPS Status Change Alert Check
    if gnssReliable ~= aAlertLastGnss then
        if gnssReliable then
            playFile("/SOUNDS/en/brhs/gnss/gnssY.wav")
        else
            playFile("/SOUNDS/en/brhs/gnss/gnssN.wav")
        end
        aAlertLastGnss = gnssReliable
    end
end

local function playAudioAlerts()
    -- 1. Welcome Message (Plays exactly once when the script starts)
    if not aAlertWelcomePlayed then
        playFile("/SOUNDS/en/brhs/welcome/welcome.wav")
        aAlertWelcomePlayed = true
    end

    -- 2. Process Flight Mode Alerts
    doFMAlert()

    -- 3. Process GPS Status Change Alerts
    doGnssAlert()

    -- 4. Process Battery Status Alerts
    doBatAlert()

    -- 5. Process Link Quality Status Alerts
    doLinkAlert()
end

local function sendTelemetryToVCP()
     local now = getTime();
	  if (now - vcpUSBLastSend) >= 100 then    -- 100 ms
        vcpUSBLastSend = now
		local telemetry =
		tostring(txBat) .. "," ..
		tostring(rxBat) .. "," ..
		tostring(rxBatMax) .. "," ..
		tostring(lq) .. "," ..
		tostring(rssi) .. "," ..
		tostring(latitude) .. "," ..
		tostring(longitude) .. "," ..
		tostring(alt) .. "," ..
		tostring(altRef) .. "," ..
		tostring(heading) .. "," ..
		tostring(headingRef) .. "," ..
		tostring(homeBearing) .. "," ..
		tostring(homeDistance) .. "," ..
		tostring(satField) .. "," ..
		tostring(fm) .. "," ..
		tostring(pitch) .. "," ..
		tostring(roll)
        serialWrite(telemetry .. "\n")
    end
end

----------------------------------------------------------------------------
-- Pack uint16_t (Big Endian)
----------------------------------------------------------------------------
local function packUint16BE(tbl, pos, value)

    value = math.floor(value) % 0x10000

    tbl[pos]     = math.floor(value / 0x100) % 0x100
    tbl[pos + 1] = value % 0x100
end

----------------------------------------------------------------------------
-- Pack int32_t (Big Endian)
----------------------------------------------------------------------------
local function packInt32BE(tbl, pos, value)
    value = math.floor(value)
    if value < 0 then
        value = value + 0x100000000
    end
    tbl[pos]     = math.floor(value / 0x1000000) % 0x100
    tbl[pos + 1] = math.floor(value / 0x10000)   % 0x100
    tbl[pos + 2] = math.floor(value / 0x100)     % 0x100
    tbl[pos + 3] = value % 0x100
end

----------------------------------------------------------------------------
-- Pack int32_t (Big Endian) , No floats and rounding
----------------------------------------------------------------------------
local function packDecStrInt32BE(tbl, pos, str)
    str = string.match(str, "^%s*(-?%d+)%s*$")
    if str == nil then
        tbl[pos], tbl[pos+1], tbl[pos+2], tbl[pos+3] = 0,0,0,0
        return
    end
    local neg = string.sub(str,1,1) == "-"
    if neg then str = string.sub(str,2) end

    local b0,b1,b2,b3 = 0,0,0,0            -- b3 = MSB, b0 = LSB
    for i = 1, #str do
        local d = string.byte(str,i) - 48
        -- multiply the 4-byte number by 10 and add digit, byte by byte
        local c = b0*10 + d ; b0 = c % 256 ; c = math.floor(c/256)
        c = b1*10 + c       ; b1 = c % 256 ; c = math.floor(c/256)
        c = b2*10 + c       ; b2 = c % 256 ; c = math.floor(c/256)
        c = b3*10 + c       ; b3 = c % 256          -- top carry discarded (>2^32)
    end

    if neg then                            -- two's complement: invert + 1
        b0 = (255-b0) ; b1 = (255-b1) ; b2 = (255-b2) ; b3 = (255-b3)
        local c = b0 + 1 ; b0 = c % 256 ; c = math.floor(c/256)
        c = b1 + c ; b1 = c % 256 ; c = math.floor(c/256)
        c = b2 + c ; b2 = c % 256 ; c = math.floor(c/256)
        c = b3 + c ; b3 = c % 256
    end

    tbl[pos]   = b3     -- big-endian: MSB first
    tbl[pos+1] = b2
    tbl[pos+2] = b1
    tbl[pos+3] = b0
end

----------------------------------------------------------------------------
-- Build MSP NAV payload from parsed command
----------------------------------------------------------------------------
local function buildNavMSPPayload(action, index, latStr, lonStr)
    local payload = {
        0xC8,        -- Destination (FC)
        0xEA,        -- Origin (Radio)
        mspSequence  -- Sequence
    }
    payload[4] = action
    if action == 2 then
        packUint16BE(payload, 5, index)
        packDecStrInt32BE(payload, 7,  latStr)   -- lat
        packDecStrInt32BE(payload, 11, lonStr)   -- lon
    end
	mspSequence = (mspSequence + 1) % 256
    return payload
end


local function receiveDataFromVCPAndSendToFC()
    ----------------------------------------------------------------------
    -- Read all available data (non-blocking)
    ----------------------------------------------------------------------
    local data = serialRead()

    if data ~= "" then
        mspRxBuffer = mspRxBuffer .. data

        if #mspRxBuffer > 512 then
            mspRxBuffer = ""
            return
        end
    end
    ----------------------------------------------------------------------
    -- Process every complete command line
    ----------------------------------------------------------------------
	local processed = 0
    while processed < 16 do

        local eol = string.find(mspRxBuffer, "\n", 1, true)

        if eol == nil then
            break
        end

        local line = string.sub(mspRxBuffer, 1, eol - 1)

        mspRxBuffer = string.sub(mspRxBuffer, eol + 1)
        processed = processed + 1
        --------------------------------------------------------------
        -- Parse CSV
        --------------------------------------------------------------
        local fields = {}

        for value in string.gmatch(line, "([^,]+)") do
            fields[#fields + 1] = value
        end

        local action = tonumber(fields[1])
		
        if action == 1 then

            local payload = buildNavMSPPayload(action)

            if crossfireTelemetryPush(CRSF_MSP_NAV_COMMAND, payload) then
                playTone(2000, 100, 0, PLAY_NOW)
            else
                playTone(100, 100, 0, PLAY_NOW)
            end

        elseif action == 2 then

            if #fields ~= 4 then
                goto continue
            end

            local index   = tonumber(fields[2])
            local latStr  = string.match(fields[3], "^%s*(-?%d+)%s*$")
            local lonStr  = string.match(fields[4], "^%s*(-?%d+)%s*$")

            -- index must be a small integer; lat/lon must be integer strings
            if (index == nil) or (latStr == nil) or (lonStr == nil) then
                goto continue   -- malformed or non-integer coord: skip, don't send (0,0)
            end

            local payload = buildNavMSPPayload(action, index, latStr, lonStr)

            if crossfireTelemetryPush(CRSF_MSP_NAV_COMMAND, payload) then
                playTone(2000, 100, 0, PLAY_NOW)
            else
                playTone(100, 100, 0, PLAY_NOW)
            end
        end
        
        ::continue::
    end
end

-- ==========================================================================
-- MAIN EXECUTION LOOP
-- ==========================================================================
local function run(event)
    lcd.clear()

    --------------------------------------------------------------------------
    -- Fetch Telemetry (Updates the shared file-scoped variables)
    --------------------------------------------------------------------------
    txBat        = getValue("tx-voltage") or getValue("tx-volt") or getValue("tx-v") or 0
    rxBat        = getValue("RxBt") or 0
    rxBatMax     = getValue("Curr") or 0
    batteryAlertState = getValue("Bat%") or 0

    lq           = getValue("RQly") or 0
    rssi         = getValue("1RSS") or 0

    alt          = getValue("Alt") or 0
    altRef       = getValue("Alts") or 0

    heading      = getValue("Yaw") or 0
    headingRef   = getValue("Hdg") or 0

    homeBearing  = getValue("VSpd") or 0
    homeDistance = getValue("GSpd") or 0

    satField     = getValue("Sats") or 0
    fm           = getValue("FM") or "---"

    pitch        = getValue("Ptch") or getValue("Pitch") or 0
    roll         = getValue("Roll") or getValue("Rol") or 0
	
	local gpsCoords = getValue("GPS")
	latitude = 0
	longitude = 0
	if type(gpsCoords) == "table" then
		latitude = gpsCoords.lat or 0
		longitude = gpsCoords.lon or 0
	end

    --------------------------------------------------------------------------
    -- Decode Telemetry
    --------------------------------------------------------------------------
    if heading < 0 then heading = heading + 360 end
    if headingRef < 0 then headingRef = headingRef + 655.36 end
    if homeBearing < 0 then homeBearing = homeBearing + 360 end

    -- UPDATED: Match the 6th-bit shifting structure from firmware
    gnssReliable = satField >= 64
    nSat = satField % 64

    --------------------------------------------------------------------------
    -- Row 1 (Y: 2)
    --------------------------------------------------------------------------
    lcd.drawText(2, 2, "BV:", 0)
    lcd.drawText(18, 2, string.format("%.1f", rxBat), BOLD)
    lcd.drawText(40, 2, string.format("/%.1f", rxBatMax), 0)
   
    lcd.drawText(66, 2, "LQ:", 0)
    lcd.drawText(82, 2, string.format("%d", lq), BOLD)
    lcd.drawText(102, 3, string.format(",%d", rssi), SMLSIZE)

    --------------------------------------------------------------------------
    -- Row 2 (Y: 12)
    --------------------------------------------------------------------------
    lcd.drawText(2, 12, "AR:", 0)
    lcd.drawText(18, 12, string.format("%.1fm", (altRef - 9000) / 100), BOLD)

    lcd.drawText(66, 12, "AC:", 0)
    lcd.drawText(82, 12, string.format("%.1fm", alt / 10), BOLD)

    --------------------------------------------------------------------------
    -- Row 3 (Y: 22)
    --------------------------------------------------------------------------
    lcd.drawText(2, 22, "HR:", 0)
    lcd.drawText(18, 22, string.format("%.1f°%s", headingRef, getCompassDirection(headingRef)), BOLD)

    lcd.drawText(66, 22, "HC:", 0)
    lcd.drawText(82, 22, string.format("%.1f°%s", heading, getCompassDirection(heading)), BOLD)

    --------------------------------------------------------------------------
    -- Row 4 (Y: 32)
    --------------------------------------------------------------------------
    lcd.drawText(2, 32, "BR:", 0)
    lcd.drawText(18, 32, string.format("%.1f°%s", homeBearing, getCompassDirection(homeBearing)), BOLD)

    lcd.drawText(66, 32, "DT:", 0)
    lcd.drawText(82, 32, string.format("%.1fm", homeDistance), BOLD)

    --------------------------------------------------------------------------
    -- Row 5 (Y: 42)
    --------------------------------------------------------------------------
    lcd.drawText(2, 42, "PR:", 0)
    lcd.drawText(18, 42, string.format("%.1f,%.1f", pitch, roll), SMLSIZE)

    lcd.drawText(66, 42, "GN:", 0)
    lcd.drawText(82, 42, gnssReliable and "Y" or "N", BOLD)
    lcd.drawText(91, 43, string.format(",%d", nSat), SMLSIZE)

    --------------------------------------------------------------------------
    -- Flight Mode (Y: 54)
    --------------------------------------------------------------------------
    lcd.drawText(38, 54, string.format("[ %s ]", tostring(fm)), BOLD)
    
    --------------------------------------------------------------------------
    -- Trigger Audio Module
    --------------------------------------------------------------------------
    playAudioAlerts()

    --------------------------------------------------------------------------
    -- Send Data to USB VCP
    --------------------------------------------------------------------------
    sendTelemetryToVCP();
    receiveDataFromVCPAndSendToFC();
   
end

local function init()
   setSerialBaudrate(115200)
end

return { 
 init = init,
 run = run 
}