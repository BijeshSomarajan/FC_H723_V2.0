-- ==========================================================================
-- FILE-SCOPED VARIABLES & CONFIGURATION (Shared across all functions)
-- ==========================================================================

-- Telemetry & Decoded data variables
local txBat, rxBat, rxBatMax, lq, rssi, alt, heading, headingRef, throttleControl
local expectedGroundSpeed, groundSpeed, homeDistance, satField, fm, pitch, roll
local gnssReliable, nSat
local latitude, longitude

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
local aAlertLastWpDone = nil
local aAlertLastOpSeq = nil
local aAlertNextBatAlert = 0
local aAlertNextLinkAlert = 0  -- Timer tracking state for link quality alerts
local WP_REACHED_SOUND = "/SOUNDS/en/brhs/mission/wpReach.wav"

-- ==========================================================================
-- ALERT CONFIGURATION SETTINGS
-- ==========================================================================

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
local NAV_WP_PAYLOAD_SIZE = 56   -- 3 hdr + 1 action + 2 idx + 4 lat + 4 lon + 2 velocity + 8 ops * 5

-- Must match CRSF_NAV_MAX_OPERATIONS on the receiver. The operations array is
-- fixed size on the wire; unused slots are zero filled (operation = 0).
-- Operation codes are NOT validated here: the CSV key is sent as-is (one byte)
-- and the FC decides what each code means.
local NAV_MAX_OPERATIONS = 8

local CRSF_MSP_NAV_COMMAND = 0x7C
local mspRxBuffer = ""
local mspSequence = 0


-- ==========================================================================
-- Decoded Flight Mode Audio Mapping Tables
-- ==========================================================================

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
    ["F"] = "/SOUNDS/en/brhs/nav/failSafe.wav",
    ["V"] = "/SOUNDS/en/brhs/nav/cruise.wav",
}

local aAlertAltModes = {
    ["B"] = "/SOUNDS/en/brhs/alt/altB.wav",
    ["T"] = "/SOUNDS/en/brhs/alt/altT.wav",
}

local aAlertLandModes = {
    ["C"] = "/SOUNDS/en/brhs/land/altc.wav",
    ["T"] = "/SOUNDS/en/brhs/land/takeoff.wav",
    ["L"] = "/SOUNDS/en/brhs/land/land.wav",
    ["F"] = "/SOUNDS/en/brhs/land/fly.wav",
    ["G"] = "/SOUNDS/en/brhs/land/touchDwn.wav",
}

local aAlertMissionModes = {
    ["M"] = "/SOUNDS/en/brhs/mission/miOn.wav",
    ["N"] = "/SOUNDS/en/brhs/mission/miOff.wav",
    ["C"] = "/SOUNDS/en/brhs/mission/miCom.wav",
}

local aAlertOpDoneModes = {
    ["T"] = "/SOUNDS/en/brhs/mission/opTkoff.wav",
    ["L"] = "/SOUNDS/en/brhs/mission/opLand.wav",
    ["A"] = "/SOUNDS/en/brhs/mission/opAlt.wav",
    ["W"] = "/SOUNDS/en/brhs/mission/opWait.wav",
}


-- ==========================================================================
-- Constant lookup table for compass directions
-- ==========================================================================

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


local function doMissionProgressAlert(cleanFm)
    -- chars 6-7: waypoints reached, 8: op-done sequence, 9: last op type
    if #cleanFm < 9 then return end

    local wpDone = tonumber(string.sub(cleanFm, 6, 7))
    local opSeq  = tonumber(string.sub(cleanFm, 8, 8))
    local opType = string.sub(cleanFm, 9, 9)
    if wpDone == nil or opSeq == nil then return end

    -- First valid frame: adopt the current state silently
    if aAlertLastWpDone == nil then aAlertLastWpDone = wpDone end
    if aAlertLastOpSeq == nil then aAlertLastOpSeq = opSeq end

    -- A count below the last one means a new mission started
    if wpDone > aAlertLastWpDone then
        playFile(WP_REACHED_SOUND)
        playNumber(wpDone, 0)
    end
    aAlertLastWpDone = wpDone

    if opSeq ~= aAlertLastOpSeq then
        if opType ~= "0" and aAlertOpDoneModes[opType] then
            playFile(aAlertOpDoneModes[opType])
        end
        aAlertLastOpSeq = opSeq
    end
end

-- ==========================================================================
-- Flight Mode Audio Alert
-- ==========================================================================

local function doFMAlert()

    -- Clean the FM string by removing hyphens/spaces and making it uppercase
    local cleanFm = string.gsub(string.upper(tostring(fm)), "%W", "")

    -- Only process individual flight mode states if telemetry returns a
    -- valid 5-character block
    if #cleanFm >= 5 then

        local currentStart   = string.sub(cleanFm, 1, 1)
        local currentNav     = string.sub(cleanFm, 2, 2)
        local currentAlt     = string.sub(cleanFm, 3, 3)
        local currentLand    = string.sub(cleanFm, 4, 4)
        local currentMission = string.sub(cleanFm, 5, 5)

        -- Initialize tracking states on first valid telemetry contact to
        -- prevent startup spam
        if aAlertLastStart == nil then
            aAlertLastStart = currentStart
        end

        if aAlertLastNav == nil then
            aAlertLastNav = currentNav
        end

        if aAlertLastAlt == nil then
            aAlertLastAlt = currentAlt
        end

        if aAlertLastLand == nil then
            aAlertLastLand = currentLand
        end

        if aAlertLastMission == nil then
            aAlertLastMission = currentMission
        end

        -- [Start State] Sub-alert Check
        if currentStart ~= aAlertLastStart then
            if aAlertStartModes[currentStart] then
                playFile(aAlertStartModes[currentStart])
            end

            aAlertLastStart = currentStart
        end

        -- [Nav State] Sub-alert Check
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
		
		doMissionProgressAlert(cleanFm)
    end
end


-- ==========================================================================
-- Battery Audio Alert
-- ==========================================================================

local function doBatAlert()

    local currentTime = getTime() -- Internal clock (100 ticks = 1 second)

    -- Round + band the incoming state so a scaled/rounded telemetry value
    -- still resolves to the right tier.
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


-- ==========================================================================
-- Link Quality Audio Alert
-- ==========================================================================

local function doLinkAlert()

    local currentTime = getTime() -- Internal clock (100 ticks = 1 second)

    -- Only process alerts if telemetry connection is active
    if rxBat > 1.5 then

        if lq < aAlertCritLinkThreshold then

            -- Critical Link Quality Alert (< 40) every 3 seconds
            if currentTime > aAlertNextLinkAlert then
                playFile("/SOUNDS/en/brhs/link/LinkQC.wav")
                aAlertNextLinkAlert = currentTime + aAlertCritLinkInterval
            end

        elseif lq < aAlertLowLinkThreshold then

            -- Low Link Quality Alert (< 70) every 15 seconds
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


-- ==========================================================================
-- GPS Status Transition Audio Alert
-- ==========================================================================

local function doGnssAlert()

    -- Initialize GPS tracking state on first run
    if aAlertLastGnss == nil then
        aAlertLastGnss = gnssReliable
    end

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


-- ==========================================================================
-- Audio Alert Manager
-- ==========================================================================

local function playAudioAlerts()

    -- 1. Welcome Message
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


-- ==========================================================================
-- Send Telemetry to USB VCP
-- ==========================================================================

local function sendTelemetryToVCP()

    local now = getTime()

    -- NOTE: getTime() ticks are 10 ms, so >= 100 means once per second.
    -- Use >= 10 for 10 Hz.
    if (now - vcpUSBLastSend) >= 100 then

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
            tostring(homeDistance) .. "," ..
            tostring(heading) .. "," ..
            tostring(headingRef) .. "," ..
            tostring(expectedGroundSpeed) .. "," ..
            tostring(groundSpeed) .. "," ..
            tostring(satField) .. "," ..
            tostring(fm) .. "," ..
            tostring(pitch) .. "," ..
            tostring(roll)

        serialWrite(telemetry .. "\n")
    end
end


-- ==========================================================================
-- Pack uint16_t (Big Endian)
-- ==========================================================================

local function packUint16BE(tbl, pos, value)

    value = math.floor(value) % 0x10000

    tbl[pos]     = math.floor(value / 0x100) % 0x100
    tbl[pos + 1] = value % 0x100
end


-- ==========================================================================
-- Pack int32_t (Big Endian)
-- ==========================================================================

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


-- ==========================================================================
-- Pack decimal-string int32_t (Big Endian)
-- No floating point conversion and no rounding
-- ==========================================================================

local function packDecStrInt32BE(tbl, pos, str)

    str = string.match(str, "^%s*(-?%d+)%s*$")

    if str == nil then
        tbl[pos], tbl[pos + 1], tbl[pos + 2], tbl[pos + 3] = 0, 0, 0, 0
        return
    end

    local neg = string.sub(str, 1, 1) == "-"

    if neg then
        str = string.sub(str, 2)
    end

    local b0, b1, b2, b3 = 0, 0, 0, 0

    -- b3 = MSB, b0 = LSB
    for i = 1, #str do

        local d = string.byte(str, i) - 48

        -- Multiply the 4-byte number by 10 and add digit
        local c = b0 * 10 + d
        b0 = c % 256
        c = math.floor(c / 256)

        c = b1 * 10 + c
        b1 = c % 256
        c = math.floor(c / 256)

        c = b2 * 10 + c
        b2 = c % 256
        c = math.floor(c / 256)

        c = b3 * 10 + c
        b3 = c % 256
    end

    if neg then

        -- Two's complement: invert + 1
        b0 = 255 - b0
        b1 = 255 - b1
        b2 = 255 - b2
        b3 = 255 - b3

        local c = b0 + 1
        b0 = c % 256
        c = math.floor(c / 256)

        c = b1 + c
        b1 = c % 256
        c = math.floor(c / 256)

        c = b2 + c
        b2 = c % 256
        c = math.floor(c / 256)

        c = b3 + c
        b3 = c % 256
    end

    tbl[pos]     = b3
    tbl[pos + 1] = b2
    tbl[pos + 2] = b1
    tbl[pos + 3] = b0
end


-- ==========================================================================
-- Build MSP NAV payload from parsed command
--
-- WP CSV format:
--   action,index,latitude,longitude,velocity,Code=Value,...
--   e.g.  2,2,129410000,776110000,10.0,5=3.0
--
-- Velocity from the app is xx.x m/s.
-- FC representation is uint16_t in units of 0.1 m/s:
--
--   5.0 m/s  -> 50
--   5.5 m/s  -> 55
--   12.3 m/s -> 123
--
-- Operations are "Code=Value" where Code is a numeric operation code (sent
-- as-is, one byte) and Value is a decimal scaled to int32_t in 0.1 steps
-- (3.0 -> 30). The Lua side does not validate codes; the FC interprets them.
--
-- Payload:
--   Byte 1-2   Destination / Origin
--   Byte 3     Sequence
--   Byte 4     Action
--   Byte 5-6   WP index
--   Byte 7-10  Latitude
--   Byte 11-14 Longitude
--   Byte 15-16 Velocity (uint16 BE, units of 0.1 m/s)
--   Byte 17-56 8 x { int32 value BE, uint8 operation }
-- ==========================================================================

-- Convert a decimal string ("5.0", "-12.35", "+3") into a scaled integer
-- string ("50", "-123", "30") without any floating point maths. Extra
-- decimals are truncated. Returns nil if malformed or outside int32_t range.
local function decimalToScaledString(str, decimals)

    local sign, ip, fp = string.match(str, "^%s*([%+%-]?)(%d*)%.?(%d*)%s*$")

    if sign == nil or (ip == "" and fp == "") then
        return nil
    end

    fp = string.sub(fp .. string.rep("0", decimals), 1, decimals)

    local digits = (string.gsub(ip .. fp, "^0+", ""))

    if digits == "" then
        digits = "0"
    end

    if #digits > 10 or (#digits == 10 and digits > "2147483647") then
        return nil
    end

    return (sign == "-" and "-" or "") .. digits
end


-- Parse "Code=Value" fields (from startIdx onward). Liberal: the key is any
-- integer op code, the value any decimal. Malformed fields are skipped,
-- fields beyond NAV_MAX_OPERATIONS are ignored. The FC decides what each
-- code means.
local function parseOperations(fields, startIdx)

    local ops = {}

    for i = startIdx, #fields do

        if #ops >= NAV_MAX_OPERATIONS then
            break
        end

        local key, val = string.match(fields[i], "^%s*(%d+)%s*=%s*([%+%-]?[%d%.]+)%s*$")

        if key ~= nil then

            local scaled = decimalToScaledString(val, 1)  -- 0.1 units: 3.0 -> 30

            if scaled ~= nil then
                ops[#ops + 1] = { code = tonumber(key) % 256, valueStr = scaled }
            end
        end
    end

    return ops
end


local function buildNavMSPPayload(action, index, latStr, lonStr, velocity, ops)
    local payload = {
        0xC8,        -- Destination (FC)
        0xEA,        -- Origin (Radio)
        mspSequence  -- Sequence
    }
    payload[4] = action
    if action == 2 then
        packUint16BE(payload, 5, index)
        packDecStrInt32BE(payload, 7, latStr)
        packDecStrInt32BE(payload, 11, lonStr)
        packUint16BE(payload, 15, velocity)
        -- Fixed array of NAV_MAX_OPERATIONS slots, 5 bytes each:
        --   int32_t value (BE), uint8_t operation. Unused slots are zero.
        for i = 1, NAV_MAX_OPERATIONS do
            local pos = 17 + (i - 1) * 5
            local op = ops[i]

            if op then
                packDecStrInt32BE(payload, pos, op.valueStr)
                payload[pos + 4] = op.code
            else
                payload[pos], payload[pos + 1], payload[pos + 2],
                payload[pos + 3], payload[pos + 4] = 0, 0, 0, 0, 0
            end
        end
    end
    mspSequence = (mspSequence + 1) % 256
    return payload
end


-- ==========================================================================
-- Receive data from VCP and send to FC
-- ==========================================================================

local function receiveDataFromVCPAndSendToFC()

    --------------------------------------------------------------------------
    -- Read all available data (non-blocking)
    --------------------------------------------------------------------------

    local data = serialRead()

    if data ~= "" then

        mspRxBuffer = mspRxBuffer .. data

        if #mspRxBuffer > 512 then
            mspRxBuffer = ""
            return
        end
    end


    --------------------------------------------------------------------------
    -- Process every complete command line
    --------------------------------------------------------------------------

    local processed = 0

    while processed < 16 do

        local eol = string.find(mspRxBuffer, "\n", 1, true)

        if eol == nil then
            break
        end

        local line = string.sub(mspRxBuffer, 1, eol - 1)

        mspRxBuffer = string.sub(mspRxBuffer, eol + 1)

        processed = processed + 1


        ----------------------------------------------------------------------
        -- Parse CSV
        ----------------------------------------------------------------------

        local fields = {}

        for value in string.gmatch(line, "([^,]+)") do
            fields[#fields + 1] = value
        end

        local action = tonumber(fields[1])


        ----------------------------------------------------------------------
        -- Action 1/3/4: Mission command without WP data
        ----------------------------------------------------------------------

        if action == 1 or action == 3 or action == 4 then

            local payload = buildNavMSPPayload(action)

            if crossfireTelemetryPush(CRSF_MSP_NAV_COMMAND, payload) then
                playTone(2000, 100, 0, PLAY_NOW)
            else
                playTone(100, 100, 0, PLAY_NOW)
            end


        ----------------------------------------------------------------------
        -- Action 2: Send waypoint
        --
        -- Expected CSV:
        -- 2,index,latitude,longitude,velocity,Code=Value,Code=Value,...
        -- e.g. 2, 2, 129410000, 776110000, 10.0, 5=3.0
        ----------------------------------------------------------------------

        elseif action == 2 then

            local index  = tonumber(fields[2])
            local latStr = fields[3] and string.match(fields[3], "^%s*(-?%d+)%s*$")
            local lonStr = fields[4] and string.match(fields[4], "^%s*(-?%d+)%s*$")

            -- Index and position are the only hard requirements: without
            -- them there is no waypoint to send.
            if index == nil or latStr == nil or lonStr == nil then
                playTone(100, 300, 0, PLAY_NOW)
                goto continue
            end

            index = math.floor(index) % 0x10000

            -- Velocity (m/s -> 0.1 m/s). If field 5 is actually an op,
            -- treat velocity as 0 and start ops there.
            local velocity = 0
            local opsStart = 6
            local vField = fields[5] or ""

            if string.find(vField, "=", 1, true) then
                opsStart = 5
            else
                local v = tonumber(vField)
                if v ~= nil then
                    velocity = math.floor(v * 10 + 0.5)
                    if velocity < 0 then velocity = 0 end
                    if velocity > 65535 then velocity = 65535 end
                end
            end

            local ops = parseOperations(fields, opsStart)

            local payload = buildNavMSPPayload(action, index, latStr, lonStr, velocity, ops)

            if crossfireTelemetryPush(CRSF_MSP_NAV_COMMAND, payload) then
                playTone(2000, 100, 0, PLAY_NOW)
            else
                playTone(100, 100, 0, PLAY_NOW)
            end
        end


        ::continue::
    end
end

local function getFirstValue(...)
    for i = 1, select("#", ...) do
        local name = select(i, ...)
        local field = getFieldInfo(name)

        if field then
            return getValue(field.id)
        end
    end

    return 0
end

-- ==========================================================================
-- MAIN EXECUTION LOOP
-- ==========================================================================

local function run(event)

    lcd.clear()


    --------------------------------------------------------------------------
    -- Fetch Telemetry
    --------------------------------------------------------------------------

    txBat = getValue("tx-voltage")
        or getValue("tx-volt")
        or getValue("tx-v")
        or 0

    rxBat = getValue("RxBt") or 0
    rxBatMax = getValue("Curr") or 0

    batteryAlertState = getValue("Bat%") or 0

    throttleControl = getValue("Capa") or 0

    lq = getValue("RQly") or 0
    rssi = getValue("1RSS") or 0

    -- Baro frame
    alt = getValue("Alt") or 0
    expectedGroundSpeed = getValue("VSpd") or 0

    -- Attitude Frame
    pitch = getValue("Ptch") or getValue("Pitch") or 0
    roll = getValue("Roll") or getValue("Rol") or 0
    heading = getValue("Yaw") or 0
    heading = (heading * 57.2957795) -- Convert to degrees

    -- GPS frame
    groundSpeed = getValue("GSpd") or 0
    homeDistance = getFirstValue("galt", "Galt", "GAlt", "Alts") or 0 -- Some tx does not support this property, you need to map this
    headingRef = getValue("Hdg") or 0
    satField = getValue("Sats") or 0

    -- Flight mode frame
    fm = getValue("FM") or "---"

    --------------------------------------------------------------------------
    -- GPS Coordinates
    --------------------------------------------------------------------------

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

    if heading < 0 then
        heading = heading + 360
    end

    if headingRef < 0 then
        headingRef = headingRef + 655.36
    end


    -- Match the 6th-bit shifting structure from firmware
    gnssReliable = satField >= 64
    nSat = satField % 64


    --------------------------------------------------------------------------
    -- Row 1 ( Battery Volts and Link Quality )
    --------------------------------------------------------------------------

    lcd.drawText(2, 2, "BV:", 0)
    lcd.drawText(18, 2, string.format("%.1f", rxBat), BOLD)
    lcd.drawText(40, 2, string.format("/%.1f", rxBatMax), 0)

    lcd.drawText(66, 2, "LQ:", 0)
    lcd.drawText(82, 2, string.format("%d", lq), BOLD)
    lcd.drawText(102, 3, string.format(",%d", rssi), SMLSIZE)


    --------------------------------------------------------------------------
    -- Row 2 ( Vertical speed and Altitude )
    --------------------------------------------------------------------------

    lcd.drawText(2, 12, "AL:", 0)
    lcd.drawText(
        18,
        12,
        string.format("%.1fm", alt / 10),
        BOLD
    )

    lcd.drawText(66, 12, "TS:", 0)
    lcd.drawText(
        82,
        12,
        string.format(
            "%.1fms",
            expectedGroundSpeed
        ),
        BOLD
    )

    --------------------------------------------------------------------------
    -- Row 3 Distance and Ground Speed
    --------------------------------------------------------------------------

    lcd.drawText(2, 22, "DT:", 0)
    lcd.drawText(
        18,
        22,
        string.format(
            "%.1fm",
            homeDistance / 10
        ),
        BOLD
    )

    lcd.drawText(66, 22, "CS:", 0)
    lcd.drawText(
        82,
        22,
        string.format("%.1fms", groundSpeed / 10),
        BOLD
    )

    --------------------------------------------------------------------------
    -- Row 4 , Heading Ref and Current heading
    --------------------------------------------------------------------------

    lcd.drawText(2, 32, "HH:", 0)
    lcd.drawText(
        18,
        32,
        string.format(
            "%.1f°%s",
            headingRef,
            getCompassDirection(headingRef)
        ),
        BOLD
    )

    lcd.drawText(66, 32, "HC:", 0)
    lcd.drawText(
        82,
        32,
        string.format(
            "%.1f°%s",
            heading,
            getCompassDirection(heading)
        ),
        BOLD
    )

    --------------------------------------------------------------------------
    -- Row 5 Throttle and GNSS status
    --------------------------------------------------------------------------

    lcd.drawText(2, 42, "TH:", 0)
    lcd.drawText(18, 42, string.format("%d", throttleControl), BOLD)

    lcd.drawText(66, 42, "GN:", 0)
    lcd.drawText(82, 42, gnssReliable and "Y" or "N", BOLD)
    lcd.drawText(91, 43, string.format(",%d", nSat), SMLSIZE)

    --------------------------------------------------------------------------
    -- Flight Mode
    --------------------------------------------------------------------------
    lcd.drawText(
        38,
        54,
        string.format("[ %s ]", tostring(fm)),
        BOLD
    )


    --------------------------------------------------------------------------
    -- Trigger Audio Module
    --------------------------------------------------------------------------

    playAudioAlerts()


    --------------------------------------------------------------------------
    -- Send Data to USB VCP
    --------------------------------------------------------------------------

    sendTelemetryToVCP()

    receiveDataFromVCPAndSendToFC()
end


-- ==========================================================================
-- INITIALIZATION
-- ==========================================================================

local function init()
    setSerialBaudrate(115200)
end


-- ==========================================================================
-- SCRIPT ENTRY POINT
-- ==========================================================================

return {
    init = init,
    run = run
}