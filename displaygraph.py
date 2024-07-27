import time as _time
import framebuf as _framebuf
try:
    import st7789 as _st7789 # type: ignore
except ModuleNotFoundError:
    try:
        import st7789py as _st7789 # type: ignore
    except ModuleNotFoundError:
        raise ModuleNotFoundError("Could not find either st7789_mpy or st7789py_mpy")
try:
    import vga1_8x8 as _vga1_8x8 # type: ignore
    import vga2_8x8 as _vga2_8x8 # type: ignore
except ModuleNotFoundError:
    raise ModuleNotFoundError("Could not find vga1_8x8 font / vga2_8x8 font")

class _DrawObject():
    def __init__(self, color: object):
        self.color = color
    
    def draw(self):
        raise NotImplementedError

class _Line(_DrawObject):
    def __init__(self, *_, x0: int, y0: int, x1: int, y1: int, color: object):
        super().__init__(color)
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1

    def draw(self, *_, buffer: _framebuf.FrameBuffer | None = None, display: _st7789.ST7789 | None = None):
        if buffer is not None:
            buffer.line(self.x0, self.y0, self.x1, self.y1, self.color)
        if display is not None:
            display.line(self.x0, self.y0, self.x1, self.y1, self.color)
        raise RuntimeError("No display or buffer provided")

    @classmethod
    def generate(cls, points: list[tuple[int, int]], color: object):
        return [
            cls(
                x0 = round(i[0]),
                y0 = round(i[1]),
                x1 = round(j[0]),
                y1 = round(j[1]),
                color = color
            ) for i,j in zip(points[0:-1], points[1:])
        ]

class _Text(_DrawObject):
    def __init__(self, *_, anchor: str, x: int, y: int, text: str, vga2: bool, color: object):
        super().__init__(color)
        self.anchor = anchor
        self.x = x
        self.y = y
        self.text = text
        self.vga2 = vga2
        if self.anchor == "rightcenter":
            self.x -= (8*len(self.text))
            self.y -= 4
        if self.anchor == "leftcenter":
            self.y -= 4
        if self.anchor == "topcenter":
            self.x -= (8*len(self.text))//2
        if self.anchor == "bottomcenter":
            self.x -= (8*len(self.text))//2
            self.y -= 8
        if self.anchor == "center":
            self.x -= (8*len(self.text))//2
            self.y -= 4
        

    def draw(self, *_, display: _st7789.ST7789 | None = None):
        if display is not None:
            display.text(_vga2_8x8 if self.vga2 else _vga1_8x8, self.text, self.x, self.y, self.color)
        raise RuntimeError("No display provided")


class TimePlot():
    def __init__(self, *_, x: int, y: int, width: int, height: int, maxDataPoints: int, minDataRange: int, maxDataRange: int, dataPointsDivisions: int = 3, dataRangeDivisions: int = 1, dataPointsFormatter: function = lambda a: f"-{a}f", dataRangeFormatter: function = lambda a: f"{a}C"):
        self.xPos = x
        self.yPos = y
        self.width = width
        self.height = height
        self.xMin = 0
        self.xMax = maxDataPoints
        self.yMin = minDataRange
        self.yMax = maxDataRange
        self.yDivisions = sorted(set([(i*(self.yMax-self.yMin)//(dataRangeDivisions+1))+self.yMin for i in range(dataRangeDivisions+2)]))
        self.xDivisions = sorted(set([(i*(self.xMax-self.xMin)//(dataPointsDivisions+1))+self.xMin for i in range(dataPointsDivisions+2)]))

        self.buffer = _framebuf.FrameBuffer(bytearray(self.width * self.height * 4), self.width, self.height, _framebuf.RGB565)
        self.dataPoints = [self.yMin for i in range(self.xMax-self.xMin)]

#TODO: Refactored uptil this point

    def drawPlotData(self, *_, display: _st7789.ST7789):
        ps = []
        for xTime,yTemp in enumerate(self.dataPoints):
            yTemp = (yTemp - self.yMin) / (self.yMax - self.yMin)
            y = self.height - (yTemp * self.height)
            xTime = (xTime - self.xMin) / (self.xMax - self.xMin)
            x = ((xTime) * self.width)
            ps.append((x,y))

        ls = _Line.generate(ps)

        self.buffer.fill(0)

        for l in ls:
            l.draw(self.buffer)
        
        display.blit_buffer(self.buffer,self.xPos,self.yPos,self.width,self.height)

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

g = TimePlot(
    x=40,
    y=20,
    width=180,
    height=80,
    maxDataPoints=100,
    minDataRange=15,
    maxDataRange=40,
)
g2 = TimePlot(
    x=40,
    y=140,
    width=180,
    height=80,
    maxDataPoints=100,
    minDataRange=15,
    maxDataRange=40,
)
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

    g.drawPlotData(display=tft)
    g2.drawPlotData(display=tft)