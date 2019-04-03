# 2D laser engraver firmware
Firmware for my DIY laser engraver

Specs :
- 37x37mm working area
- Adafruit motor shield v2.3
- 2.3W 445nm laser
- ATX power supply
- no end stops

G-Codes implemented :
- G00/G01 (linear moves)
- G02/G03 (arc moves)
- G4 (pause)
- G92 (set position)
- M03 (laser on)
- M05 (laser off)
- M18/M84 (disable steppers)
- M106 (fan speed)
- M107 (fan off)
- M112 (emergency stop)
- M114 (get position)
