import time, framebuf

class TimeGraph():
    xMin = 0
    def __init__(self, xPos, yPos, width, height, xMax, yMin, yMax, xDivisions, yDivisions):
        self.xPos = xPos
        self.yPos = yPos
        self.width = width
        self.height = height
        self.xMax = xMax
        # self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.yDivisions = yDivisions # yMin, ..., yMax
        self.xDivisions = xDivisions # 0,5,10 ... xMax

        self.fbuf = framebuf.FrameBuffer(bytearray(width * height * 4), width, height, framebuf.RGB565)
        self.dataPoints = [self.yMin for i in range(self.xMax)]

    @staticmethod
    def pointsToLines(points):
        lines = []
        for i in range(len(points) - 1):
            current_point = points[i]
            next_point = points[i + 1]
            line_dict = {
              "draw": "line",
              "x0": round(current_point[0]),
              "y0": round(current_point[1]),
              "x1": round(next_point[0]),
              "y1": round(next_point[1]),
              "colour": (0, 0, 255)  # Set blue color for lines
            }
            lines.append(line_dict)
        return lines
    def drawData(self):
        ps = []
        for xTime,yTemp in enumerate(self.dataPoints):
            yTemp = (yTemp - self.yMin) / (self.yMax - self.yMin)
            y = self.height - (yTemp * self.height)
            xTime = (xTime - self.xMin) / (self.xMax - self.xMin)
            x = ((xTime) * self.width)
            ps.append((x,y))

        ls = self.pointsToLines(ps)

        self.fbuf.fill(0)

        for l in ls:
            if l["draw"] == "line":
                self.fbuf.line(l["x0"], l["y0"], l["x1"], l["y1"], st7789.color565(*l["colour"]))
        
        return self.fbuf

    def drawBox(self):
        queue =[
            {
                "draw": "line", # vertical line
                "x0": self.xPos-1,
                "y0": self.yPos,
                "x1": self.xPos-1,
                "y1": self.yPos + self.height,
                "colour": (255,255,255),
            },
            {
                "draw": "line", # horizontal line
                "x0": self.xPos,
                "y0": self.yPos + self.height,
                "x1": self.xPos + self.width,
                "y1": self.yPos + self.height,
                "colour": (255,255,255),
            },
            {
                "draw": "text", # yMax
                "anchor": "rightcenter",
                "x": self.xPos,
                "y": self.yPos,
                "text": str(self.yDivisions[-1]),
                "colour": (255,0,0),
            },
            {
                "draw": "text", # yMin
                "anchor": "rightcenter",
                "x": self.xPos,
                "y": self.yPos + self.height,
                "text": str(self.yDivisions[0]),
                "colour": (255,0,0),
            },
                        {
                "draw": "text", # xMax
                "anchor": "topcenter",
                "x": self.xPos,
                "y": self.yPos + self.height,
                "text": str(self.xDivisions[-1]),
                "colour": (0,0,255),
            },
            {
                "draw": "text", # xMin
                "anchor": "topcenter",
                "x": self.xPos + self.width,
                "y": self.yPos + self.height,
                "text": str(self.xDivisions[0]),
                "colour": (0,0,255),
            },
        ]
        queue += [ {
                "draw": "text", # yDivisions
                "anchor": "rightcenter",
                "x": self.xPos,
                "y": round(self.yPos + (((self.height*(len(self.yDivisions)-i-2))/(len(self.yDivisions)-1)))),
                "text": str(v),
                "colour": (255,0,0),
            } for i,v in enumerate(self.yDivisions[1:-1])
        ]
        queue += [ {
                "draw": "text", # xDivisions
                "anchor": "topcenter",
                "x": round(self.xPos +((self.width*(i+1))/(len(self.xDivisions)-1))),
                "y": self.yPos + self.height,
                "text": str(v),
                "colour": (0,0,255),
            } for i,v in enumerate(reversed(self.xDivisions[1:-1]))
        ]

        return queue
    
    def add(self, val):
        l = list(self.dataPoints)
        l.append(val)
        l.pop(0)
        self.dataPoints = l

# https://github.com/russhughes/st7789_mpy/blob/master/firmware/RP2/firmware.uf2
# https://www.coderdojotc.org/micropython/displays/graph/14-lcd-st7789V/

# for pure micropython:- https://github.com/russhughes/st7789py_mpy

# deleting the flashed firmware:- https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html#resetting-flash-memory
from machine import Pin, SPI, ADC
# from rotary_irq_rp2 import RotaryIRQ # type: ignore
# https://github.com/MikeTeachman/micropython-rotary
import math
import st7789# type: ignore
import vga1_8x8 as font # type: ignore
# from rotary_irq_rp2 import RotaryIRQ # type: ignore
# https://docs.keyestudio.com/projects/KS3024/en/latest/MicroPython/Micropython.html
WIDTH, HEIGHT = 240, 240
 # SCK = SCL
 # MOSI = SDA
BACKLIGHT_PIN = 20
RST_PIN = 16
DC_PIN = 21
CS_PIN = 17
SCK_PIN = 18
MOSI_PIN = 19
SPI_NUM = 0

# Voltage Divider
Vin = 3.3
Ro = 10000  # 10k Resistor

# Steinhart Constants
A = 0.001129148
B = 0.000234125
C = 0.0000000876741

T1 = ADC(26)
T2 = ADC(27)

SPDT = Pin(22, Pin.IN)
SW = Pin(0, Pin.IN, Pin.PULL_UP)

# r = RotaryIRQ(pin_num_clk=2,
#               pin_num_dt=1,
#               min_val=0,
#               reverse=False,
#               range_mode=RotaryIRQ.RANGE_UNBOUNDED,
#               pull_up=True,
#               half_step=True
#               )

def NTC_Thermistor(thermistor:ADC):
  # Get Voltage value from ADC   
  adc = thermistor.read_u16()
  Vout = (3.3/65535)*adc
  
  # Calculate Resistance
  # Rt = (Vout * Ro) / (Vin - Vout) 
  Rt = Ro * (Vin - Vout) / Vout
  # Rt = 10000  # Used for Testing. Setting Rt=10k should give TempC=25
  
  # Steinhart - Hart Equation
  TempK = 1 / (A + (B * math.log(Rt)) + C * math.pow(math.log(Rt), 3))

  # Convert from Kelvin to Celsius
  TempC = TempK - 273.15

  return TempC

spi = SPI(SPI_NUM, baudrate=31250000, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN))
tft = st7789.ST7789(
    spi, WIDTH, HEIGHT,
    reset=Pin(RST_PIN, Pin.OUT),
    cs=Pin(CS_PIN, Pin.OUT),
    dc=Pin(DC_PIN, Pin.OUT),
    backlight=Pin(BACKLIGHT_PIN, Pin.OUT),
    rotation=3,
)
print(tft)
tft.init()    

    
# window = pyglet.window.Window(240,240,)#style=pyglet.window.Window.WINDOW_STYLE_BORDERLESS)

g = TimeGraph(40,20,180,80,100,15,40,["-0f", "-2f","-4f", "-6f", "-8f","-10f"],["15°C","20°C","25°C","30°C","35°C","40°C",])
g2 = TimeGraph(40,140,180,80,100,15,40,["-0f", "-2f","-4f", "-6f", "-8f","-10f"],["15°C","20°C","25°C","30°C","35°C","40°C",])
d = g.drawBox()
d2 = g2.drawBox()

tft.fill(st7789.color565(0,0,0))
for i in d+d2:
    if i["draw"] == "line":
        tft.line(i["x0"],i["y0"],i["x1"],i["y1"],st7789.color565(*i["colour"]))
        # d2.append(pyglet.shapes.Line(i["x0"],i["y0"],i["x1"],i["y1"],1,i["colour"]))
    if i["draw"] == "text":
        if i["anchor"] == "rightcenter":
            tft.text(font,i["text"],i["x"]-24-4,i["y"]-4,st7789.color565(*i["colour"]))
        if i["anchor"] == "topcenter":
            tft.text(font,i["text"],i["x"]-16,i["y"]+4,st7789.color565(*i["colour"]))
        # d2.append(pyglet.text.Label(i["text"],font_name="Times New Roman",font_size=10,x=i["x"],y=i["y"],anchor_x=i["anchor_x"],anchor_y=i["anchor_y"],color=i["colour"]))
while True:
    T1_temp = NTC_Thermistor(T1)
    T2_temp = NTC_Thermistor(T2)
    g.add(T1_temp)
    g2.add(T2_temp)
    # tft.fill_rect(g.xPos+1,g.yPos,g.width,g.height, st7789.color565(0,0,0))
    tft.blit_buffer(g.drawData(),g.xPos,g.yPos,g.width,g.height)
    tft.blit_buffer(g2.drawData(),g2.xPos,g2.yPos,g2.width,g2.height)
            # d2.append(pyglet.shapes.Line(i["x0"],i["y0"],i["x1"],i["y1"],1,i["colour"]))
    # time.sleep(0.2)
    # T1_temp = str(NTC_Thermistor(T1))[:5]
    # T2_temp = str(NTC_Thermistor(T2))[:5]
    # SPDT_state = int(bool(SPDT.value()))
    # SW_state = int(not bool(SW.value()))
    # val = r.value()

    # tft.text(font, f"T1:{T1_temp}, B:{SPDT_state}          ",10, 0, st7789.color565(255,255,255), st7789.color565(0,0,0))
    # tft.text(font, f"T2:{T2_temp}, S:{SW_state}",10, 50, st7789.color565(255,0,0), st7789.color565(0,0,0))
    # tft.text(font, f"Rt Enc:{val}",10, 100, st7789.color565(0,255,0), st7789.color565(0,0,0))