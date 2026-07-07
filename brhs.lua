local function run(event)
  lcd.clear()

  -- Fetch Raw Telemetry Streams (with fallbacks if nil)
  local txBat = getValue("tx-voltage") or getValue("tx-volt") or getValue("tx-v") or 0
  local lq = getValue("RQly") or 0
  local rssi = getValue("1RSS") or 0  
  local alt = getValue("Alt") or 0
  local altRef = getValue("Alts") or 0
  local heading = getValue("Yaw") or 0
  local headingRef = getValue("Hdg") or 0
  local rxBat = getValue("RxBt") or 0
  local vSpd = getValue("VSpd") or 0
  local nSat = getValue("Sats") or 0
  local fm = getValue("FM")
  
  -- Fetch Pitch and Roll
  local pitch = getValue("Ptch") or getValue("Pitch") or 0
  local roll = getValue("Roll") or getValue("Rol") or 0

  -- Convert Yaw back to 0-360
  if heading < 0 then
    heading = heading + 360
  end
  
  -- Handle the signed int16 overflow bug for Reference/GNSS Heading
  if headingRef < 0 then
    headingRef = headingRef + 655.36
  end

  -- LEFT COLUMN (X: 2) - Split into Normal Label & Bold Value
  lcd.drawText(2, 2,  "BTX:", 0)
  lcd.drawText(26, 2, string.format("%.1fV", txBat), BOLD)

  lcd.drawText(2, 12, "LQ:", 0)
  lcd.drawText(20, 12, string.format("%d%%", lq), BOLD)

  lcd.drawText(2, 22, "ARef:", 0)
  lcd.drawText(32, 22, string.format("%.1f", (altRef-9000)/100), BOLD)

  lcd.drawText(2, 32, "HRef:", 0)
  lcd.drawText(32, 32, string.format("%.1f", headingRef), BOLD) 

  lcd.drawText(2, 42, "VS:", 0)
  lcd.drawText(20, 42, string.format("%.2f", vSpd), BOLD) 

  -- RIGHT COLUMN (X: 64) - Split into Normal Label & Bold Value
  lcd.drawText(64, 2,  "BRX:", 0)
  lcd.drawText(88, 2,  string.format("%.1fV", rxBat), BOLD)

  lcd.drawText(64, 12, "RS:", 0)
  lcd.drawText(82, 12, string.format("%ddB", rssi), BOLD)

  lcd.drawText(64, 22, "ACur:", 0)
  lcd.drawText(94, 22, string.format("%.1f", alt/10), BOLD)

  lcd.drawText(64, 32, "HCur:", 0)
  lcd.drawText(94, 32, string.format("%.1f", heading), BOLD) 

  lcd.drawText(64, 42, "SAT:", 0)
  lcd.drawText(88, 42, string.format("%d", nSat), BOLD) 
   
  -- BOTTOM ROW (Y: 54) - Shared row for Pitch, Flight Mode, and Roll
  -- Pitch (Left-aligned)
  lcd.drawText(2, 54, "P:", 0)
  lcd.drawText(14, 54, string.format("%0.1f", pitch), BOLD)

  -- Flight Mode (Centered)
  lcd.drawText(45, 54, string.format("%s", tostring(fm or "---")), BOLD) 

  -- Roll (Right-aligned)
  lcd.drawText(88, 54, "R:", 0)
  lcd.drawText(100, 54, string.format("%0.1f", roll), BOLD)

end

return { run=run }