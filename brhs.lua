local function getCompassDirection(angle)
    -- Normalize to 0-360
    angle = angle % 360

    local directions = {
        "N", "NE", "E", "SE",
        "S", "SW", "W", "NW"
    }

    local index = math.floor((angle + 22.5) / 45) % 8
    return directions[index + 1]
end

local function run(event)
    lcd.clear()

    --------------------------------------------------------------------------
    -- Fetch Telemetry
    --------------------------------------------------------------------------
    local txBat        = getValue("tx-voltage") or getValue("tx-volt") or getValue("tx-v") or 0
    local rxBat        = getValue("RxBt") or 0

    local lq           = getValue("RQly") or 0
    local rssi         = getValue("1RSS") or 0

    local alt          = getValue("Alt") or 0
    local altRef       = getValue("Alts") or 0

    local heading      = getValue("Yaw") or 0
    local headingRef   = getValue("Hdg") or 0

    local homeBearing  = getValue("VSpd") or 0
    local homeDistance = getValue("GSpd") or 0

    local satField     = getValue("Sats") or 0
    local fm           = getValue("FM") or "---"

    local pitch        = getValue("Ptch") or getValue("Pitch") or 0
    local roll         = getValue("Roll") or getValue("Rol") or 0

    --------------------------------------------------------------------------
    -- Decode Telemetry
    --------------------------------------------------------------------------
    -- Aircraft heading (-180..180 -> 0..360)
    if heading < 0 then
        heading = heading + 360
    end

    -- GNSS heading overflow workaround
    if headingRef < 0 then
        headingRef = headingRef + 655.36
    end

    -- Home bearing (-180..180 -> 0..360)
    if homeBearing < 0 then
        homeBearing = homeBearing + 360
    end

    -- Decode GNSS reliability + satellite count
    local gnssReliable = satField >= 128
    local nSat = satField % 128

    --------------------------------------------------------------------------
    -- Row 1 (Y: 2)
    --------------------------------------------------------------------------
    lcd.drawText(2, 2, "BT:", 0)
    lcd.drawText(18, 2, string.format("%.1f,", txBat), 0)
    lcd.drawText(36, 2, string.format("%.1fv", rxBat), BOLD)

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

end

return { run = run }