-- ==========================================================================
-- FILE-SCOPED VARIABLES & CONFIGURATION (Shared across all functions)
-- ==========================================================================
-- Telemetry & Decoded data variables
local txBat, rxBat, rxBatMax, lq, rssi, alt, altRef, heading, headingRef
local homeBearing, homeDistance, satField, fm, pitch, roll
local gnssReliable, nSat

-- Persistent state tracking variables for audio alerts
local aAlertWelcomePlayed = false
local aAlertLastStart = nil
local aAlertLastNav = nil
local aAlertLastAlt = nil
local aAlertLastLand = nil
local aAlertLastGnss = nil
local aAlertNextBatAlert = 0
local aAlertNextLinkAlert = 0  -- Timer tracking state for link quality alerts

-- ALERT CONFIGURATION SETTINGS

-- Battery thresholds, percentages, and timings
local aAlertLowBatPercent    = 0.88 -- 88% of rxBatMax for Low Battery alert
local aAlertCritBatPercent   = 0.84 -- 84% of rxBatMax for Critical Battery alert
local aAlertBatInterval      = 2000 -- 20 seconds repeat interval (Low Battery)
local aAlertCritBatInterval  = 500  -- 5 seconds repeat interval (Critically low Battery)

-- Link Quality Thresholds and Timings
local aAlertLowLinkThreshold  = 70
local aAlertCritLinkThreshold = 40
local aAlertLinkInterval      = 1500  -- 15 seconds repeat interval (Low Link)
local aAlertCritLinkInterval  = 300   -- 3 seconds repeat interval (Critical Link)

-- VCP USB Params
local vcpUSBLastSend = 0


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
}

local aAlertAltModes = {
    ["B"] = "/SOUNDS/en/brhs/alt/altB.wav",
    ["T"] = "/SOUNDS/en/brhs/alt/altT.wav",
}

local aAlertLandModes = {
    ["L"] = "/SOUNDS/en/brhs/land/land.wav",
    ["F"] = "/SOUNDS/en/brhs/land/fly.wav",
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
    if #cleanFm == 4 then
        local currentStart = string.sub(cleanFm, 1, 1)
        local currentNav   = string.sub(cleanFm, 2, 2)
        local currentAlt   = string.sub(cleanFm, 3, 3)
        local currentLand  = string.sub(cleanFm, 4, 4)

        -- Initialize tracking states on first valid telemetry contact to prevent startup spam
        if aAlertLastStart == nil then aAlertLastStart = currentStart end
        if aAlertLastNav == nil then aAlertLastNav = currentNav end
        if aAlertLastAlt == nil then aAlertLastAlt = currentAlt end
        if aAlertLastLand == nil then aAlertLastLand = currentLand end

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
    end
end

-- Dedicated function to manage dual-tier voltage alerts
local function doBatAlert()
    local currentTime = getTime() -- Internal clock (100 ticks = 1 second)

    -- Only process if telemetry is active and rxBatMax has captured a valid value
    if rxBat > 1.5 and rxBatMax > 0 then
        if rxBat < (rxBatMax * aAlertCritBatPercent) then
            -- Tier 1: Critical Battery Alert every 5 seconds
            if currentTime > aAlertNextBatAlert then
                playFile("/SOUNDS/en/brhs/bat/rxBatC.wav")
                aAlertNextBatAlert = currentTime + aAlertCritBatInterval
            end
        elseif rxBat < (rxBatMax * aAlertLowBatPercent) then
            -- Tier 2: Low Battery Alert every 20 seconds
            if currentTime > aAlertNextBatAlert then
                playFile("/SOUNDS/en/brhs/bat/rxBatL.wav")
                aAlertNextBatAlert = currentTime + aAlertBatInterval
            end
        else
            -- Battery is healthy, keep timer synchronized
            if currentTime > aAlertNextBatAlert then
                aAlertNextBatAlert = currentTime
            end
        end
    else
        -- Telemetry completely lost, keep timer synchronized
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

local function sendDataToVCP()
     local now = getTime();
	  if (now - vcpUSBLastSend) >= 1000 then    -- 1000 ms
        vcpUSBLastSend = now
        serialWrite("hello\n")
    end
	 
end

local function receiveDataFromVCP()
    local data = serialRead()
    if data and #data > 0 then
        playTone(1000, 100, 0, PLAY_NOW)
       serialWrite("Got\n")
    end
end

local function sendMSPToFC()
    local now = getTime();
	if (now - vcpUSBLastSend) >= 1000 then    -- 1000 ms
        vcpUSBLastSend = now
		local payload = {
			0xC8,       -- Destination: Flight Controller
			0xEA,       -- Origin: Radio
			0x01,       -- Sequence number (can start at 1)
			0x55        -- Test data
        }
		
		local ok = crossfireTelemetryPush(0x7C, payload)

		if ok then
			serialWrite("MSP Sent\r\n")
		else
			serialWrite("MSP Busy\r\n")
		end
       
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
   --sendDataToVCP()
   --receiveDataFromVCP();
   --sendMSPToFC();
end

local function init()
    --setSerialBaudrate(115200)
end

return { 
 init = init,
 run = run 
}