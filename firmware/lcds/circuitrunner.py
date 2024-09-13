#! Open Source https://github.com/aedile/circuit_python_wsRP2040128 



import random
import time
from math import floor

import board
import busio
import displayio
import gc9a01
import terminalio
import analogio
import digitalio
import vectorio


from adafruit_display_text import label
from adafruit_bitmap_font import bitmap_font


# A class to interface with a GC9A01 round display
# This class is based on the Adafruit CircuitPython GC9A01 library
# It is modified to work with the WS RP2040 1.28" dev board
class GC9A01_Display(object):
    # Initialize the display
    # autoshow: if True, the display will be updated after each draw operation
    #           if False, the display will not be updated until show() is called
    #           default is True
    # returns: nothing
    def __init__(self, autoshow=True):
        # Pins
        self.clk = board.GP10
        self.mosi = board.GP11
        self.rst = board.GP12
        self.dc = board.GP8
        self.cs = board.GP9
        self.bl = board.GP25
        
        self.auto_show = autoshow

        self.width = 240
        self.height = 240
        
        self.spi = busio.SPI(clock=self.clk, MOSI=self.mosi)
        self.display_bus = displayio.FourWire(self.spi, command=self.dc, chip_select=self.cs, reset=self.rst)
        self.display  = gc9a01.GC9A01(self.display_bus, width=self.width, height=self.height, backlight_pin=self.bl)

        self.groups = {
            'default': displayio.Group()
        }

    # Turn off the backlight
    # returns: nothing
    def off(self):
        self.display.brightness = 0

    # Turn on the backlight
    # returns: nothing
    def on(self):
        self.display.brightness = 1

    # Add a new group to the display
    # groupname: the name of the group to add
    # returns: nothing
    def add_group(self, groupname):
        self.groups[groupname] = displayio.Group()

    # Draw a polygon on the display
    # points: a list of points, each point is a Tuple of 2 integers
    # x: the x coordinate of the 0,0 origin of the polygon
    # y: the y coordinate of the 0,0 origin of the polygon
    # color: a list of colors for the palette
    # groupname: the name of the group to add the polygon to
    # returns: the vectorio.Polygon object that was created
    def draw_polygon(self,points,x,y,color,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        polygon = vectorio.Polygon(pixel_shader=palette, points=points, x=x, y=y)
        self.groups[groupname].append(polygon)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return polygon

    # Draw a rectangle on the display
    # x: the x coordinate of the upper left corner of the rectangle
    # y: the y coordinate of the upper left corner of the rectangle
    # w: the width of the rectangle
    # h: the height of the rectangle
    # color: a list of colors for the palette
    # groupname: the name of the group to add the rectangle to
    # returns: the vectorio.Rectangle object that was created
    def draw_rectangle(self,x,y,w,h,color,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        rectangle = vectorio.Rectangle(pixel_shader=palette, width=w, height=h, x=x, y=y)
        self.groups[groupname].append(rectangle)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return rectangle
    

    # Draw a circle on the display
    # x: the x coordinate of the center of the circle
    # y: the y coordinate of the center of the circle
    # r: the radius of the circle
    # color: a list of colors for the palette
    # groupname: the name of the group to add the circle to
    # returns: the vectorio.Circle object that was created
    def draw_circle(self,x,y,r,color,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        circle = vectorio.Circle(pixel_shader=palette, radius=r, x=x, y=y)
        self.groups[groupname].append(circle)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return circle
    
    # Draw text on the display
    # x: the x coordinate of the upper left corner of the text
    # y: the y coordinate of the upper left corner of the text
    # text: the text to display
    # color: a list of colors for the pallette
    # font: the font to use
    # groupname: the name of the group to add the text to
    # returns: the label.Label object that was created
    def draw_text(self,x,y,text,color,font,groupname='default'):
        palette = displayio.Palette(len(color))
        for i in range(len(color)):
            palette[i] = color[i]
        text_area = label.Label(font, text=text, color=palette[0])
        text_area.x = x
        text_area.y = y
        self.groups[groupname].append(text_area)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return text_area    

    # Draw a bitmap file on the display
    # NOTE: proceed with caution as memory is low on this device
    # x: the x coordinate of the upper left corner of the bitmap
    # y: the y coordinate of the upper left corner of the bitmap
    # bitmap_path: the path to the bitmap file on the device
    # transparent_color: the color to make transparent, default is None
    # groupname: the name of the group to add the bitmap to
    # returns: the displayio.TileGrid object that was created
    def draw_bitmap(self,x,y,bitmap_path,transparent_color=None,groupname='default'):
        return self.draw_sprite(x,y,bitmap_path,1,1,0,transparent_color,groupname)
    
    # Draw an animated sprite on the display
    # NOTE: proceed with caution as memory is low on this device
    # x: the x coordinate of the upper left corner of the sprite
    # y: the y coordinate of the upper left corner of the sprite
    # sprite_path: the path to the sprite file on the device
    # sprite_tiles_x: the number of tiles in the sprite in the x direction
    # sprite_tiles_y: the number of tiles in the sprite in the y direction
    # sprite_tile_width: the width of each tile in the sprite
    # sprite_tile_height: the height of each tile in the sprite
    # sprite_starting_tile: the starting tile in the sprite
    # transparent_color: the color to make transparent
    # groupname: the name of the group to add the sprite to
    # returns: the displayio.TileGrid object that was created
    def draw_sprite(self,x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile,transparent_color = None, groupname='default'):
        bitmap = displayio.OnDiskBitmap(sprite_path)
        if transparent_color != None:
            bitmap.pixel_shader.make_transparent(transparent_color)
        sprite_tile_width = bitmap.width // sprite_tiles_x
        sprite_tile_height = bitmap.height // sprite_tiles_y
        tile_grid = displayio.TileGrid(bitmap, pixel_shader=bitmap.pixel_shader, width=1, height=1, tile_width=sprite_tile_width, tile_height=sprite_tile_height, default_tile=sprite_starting_tile, x=x, y=y)
        self.groups[groupname].append(tile_grid)
        if self.auto_show:
            self.display.show(self.groups[groupname])
        return tile_grid

    # Fill the display with a color
    # color: a list of colors for the palette
    # groupname: the name of the group to add the rectangle to
    # returns: the displayio.TileGrid object that was created
    def fill(self,color,groupname='default'):
        return self.draw_rectangle(0,0,self.width,self.height,color,groupname)

    # Show the display
    # returns: nothing
    def show(self):
        for group in self.groups:
            if not self.groups[group].hidden:
                self.display.show(self.groups[group])
            self.display.show(self.groups[group])
            break
    
# CircuitPython class representing the Waveshare RP2040 1.28" 
# development board.  This class is used to initialize the
# display and the pins used to communicate with the display as 
# well as the pins used to communicate with the accelerometer.
# It also has helper functions to make it easier to draw to the
# display.
class wsRP2040128(object):
    # Initialize the board
    # returns: nothing
    def __init__(self,initAccel=True, initBattery=True, initDisplay=True):
        # What we're actually gonna use
        self._use_display = initDisplay

        # Important - release the display from dev stuff
        if(self._use_display):
            displayio.release_displays()
        
        # Initialize hardware
        if(self._use_display):
            self._display = GC9A01_Display(True)
            # This is where we'll track our sprites
            self.sprites = {}
        self.font = bitmap_font.load_font("font/spleen-16x32.bdf")

        # Time tracker
        self.time = time.monotonic()

    # Passthrough method to draw a polygon on the display
    # sprite_id: text identifier for the sprite
    # points: a list of points that make up the polygon
    # x: the x coordinate of the 0,0 origin of the polygon
    # y: the y coordinate of the 0,0 origin of the polygon
    # color: the color of the polygon
    # group: the index of the displayio.Group object to add 
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_polygon(self,sprite_id,points,x,y,color,group='default'):
        self.sprites[sprite_id] = self._display.draw_polygon(points,x,y,color,group)

    # Passthrough method to draw a circle on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the center of the circle
    # y: the y coordinate of the center of the circle
    # r: the radius of the circle
    # color: the color of the circle
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_circle(self,sprite_id,x,y,r,color,group='default'):
        self.sprites[sprite_id] = self._display.draw_circle(x,y,r,color,group)

    # Passthrough method to draw a rectangle on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the rectangle
    # y: the y coordinate of the upper left corner of the rectangle
    # w: the width of the rectangle
    # h: the height of the rectangle
    # color: the color of the rectangle
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing    
    def draw_rectangle(self,sprite_id,x,y,w,h,color,group='default'):
        self.sprites[sprite_id] = self._display.draw_rectangle(x,y,w,h,color,group)

    # Passthrough method to draw text on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the text
    # y: the y coordinate of the upper left corner of the text
    # text: the text to display
    # color: the color of the text
    # font: the font to use
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_text(self,sprite_id, x,y,text,color,font,group='default'):
        self.sprites[sprite_id] = self._display.draw_text(x,y,text,color,font,group)
    
    # Passthrough method to draw a bitmap file on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the bitmap
    # y: the y coordinate of the upper left corner of the bitmap
    # bitmap_path: the path to the bitmap file on the device
    # trans_color: the color to use for transparency
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_bitmap(self,sprite_id, x,y,bitmap_path,trans_color=None,group='default'):
        self.sprites[sprite_id] = self._display.draw_bitmap(x,y,bitmap_path,trans_color,group)

    # Passthrough method to draw an animated sprite on the display
    # sprite_id: text identifier for the sprite
    # x: the x coordinate of the upper left corner of the sprite
    # y: the y coordinate of the upper left corner of the sprite
    # sprite_path: the path to the sprite file on the device
    # sprite_tiles_x: the number of tiles in the sprite in the x direction
    # sprite_tiles_y: the number of tiles in the sprite in the y direction
    # sprite_starting_tile: the starting tile in the sprite
    # trans_color: the color to use for transparency
    # group: the index of the displayio.Group object to add
    #        the sprite to. Default is 'default'
    # returns: nothing
    def draw_sprite(self,sprite_id,x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile,trans_color=None,group='default'):
        self.sprites[sprite_id] = self._display.draw_sprite(x,y,sprite_path,sprite_tiles_x,sprite_tiles_y,sprite_starting_tile,trans_color,group)

    # Passthrough method to fill the display with a color
    # color: the color to fill the display with
    # returns: nothing
    def fill(self,color,group='default'):
        self.sprites['background'] = self._display.fill(color,group)

    # Helper method to convert a color string to a color value
    # colstr: the color string
    # returns: the color value
    def color(self,colstr):
        colors = {
            'black': 0x000000,
            'white': 0xFFFFFF,
            'gray': 0x808080,
            'lightgray': 0xC0C0C0,
            'darkgray': 0x404040,
            'red': 0xFF0000,
            'green': 0x00FF00,
            'blue': 0x0000FF,
            'yellow': 0xFFFF00,
            'cyan': 0x00FFFF,
            'magenta': 0xFF00FF,
            'orange': 0xFFA500,
            'purple': 0x800080,
            'brown': 0xA52A2A,
            'pink': 0xFFC0CB,
            'lime': 0x00FF00,
            'teal': 0x008080,
            'maroon': 0x800000,
            'navy': 0x000080,
            'olive': 0x808000,
            'violet': 0xEE82EE,
            'turquoise': 0x40E0D0,
            'silver': 0xC0C0C0,
            'gold': 0xFFD700,
            'indigo': 0x4B0082,
            'coral': 0xFF7F50,
            'salmon': 0xFA8072,
            'tan': 0xD2B48C,
            'khaki': 0xF0E68C,
            'plum': 0xDDA0DD,
            'darkgreen': 0x006400
        }
        try:
            return [colors[colstr]]
        except:
            return [0x0000]

    # Helper method to get incremental steps
    # between two numbers.
    # start: the starting number
    # end: the ending number
    # steps: the number of steps to take
    # returns: a list of integers
    def _steps(self,start,end,steps):
        return [round(start + (end - start) * i / (steps - 1)) for i in range(steps)]        

    # Helper method to convert a binary color value to 
    # a list of three integers representing the red, green,
    # and blue values.
    # color: the binary color value
    # returns: a list of three integers
    def _color_to_rgb(self,color):
        return [color[0] >> 16 & 0xFF, color[0] >> 8 & 0xFF, color[0] & 0xFF]
    
    # Helper method to convert a list of three integers
    # representing the red, green, and blue values to a
    # binary color value.
    # rgb: a list of three integers
    # returns: the binary color value
    def _rgb_to_color(self,rgb):
        return [rgb[0] << 16 | rgb[1] << 8 | rgb[2]]

    # Helper method for color fades.  This method
    # accepts two binary color values and the number
    # of steps to take between the two colors.  It will
    # return a list of colors that can be used to fade.
    # start: the starting color
    # end: the ending color
    # returns: the color value
    def fade(self,start,end,steps):
        start_rgb = self._color_to_rgb(start)
        end_rgb = self._color_to_rgb(end)
        r_steps = self._steps(start_rgb[0],end_rgb[0],steps)
        g_steps = self._steps(start_rgb[1],end_rgb[1],steps)
        b_steps = self._steps(start_rgb[2],end_rgb[2],steps)
        return [self._rgb_to_color([r_steps[i],g_steps[i],b_steps[i]]) for i in range(steps)]
    
    # Show the display
    # returns: nothing
    def _show(self):
        self._display.show()  

    # Update the hardware on the board for this pass
    # returns: nothing
    def update(self):
        if(self._use_display):
            self._show()

    # Demo code - run in the main loop, works if
    # you turn off hardware still.
    # sleep_time: the time to sleep between updates
    # returns: nothing
    def demo(self, sleep_time=0.05):
        if(self._use_display):
            # Fill the background with black
            self.draw_rectangle("demobg",0,0,240,240,self.color('black'))
            # Draw the title bar
            self.draw_rectangle("title",0,0,240,40,self.color('red'))
            self.draw_text("title_text", 60, 25, "WS RP2040 1.28 Demo", self.color('white'), terminalio.FONT)

            # Draw a cicle
            self.draw_circle("circle", 40, 170, 20, self.color('yellow'))
            # Draw a polygon
            points = [
                (15,0),
                (11,10),
                (0,10),
                (8,20),
                (5,29),
                (15,21),
                (25,29),
                (22,20),
                (29,10),
                (19,10)
            ]
            self.draw_polygon("polygon", points, 160, 160, self.color('brown'))
        else:
            print("Display not intialized")

        while True:
            if(self._use_display):
                # Title Animation
                if(self.sprites['title_text'].x < -100):
                    self.sprites['title_text'].x = 200
                else:
                    self.sprites['title_text'].x -= 1

                # Update sprite, spritecounter and display    
                if(self.time + sleep_time) < time.monotonic():
                    self.update()
                    self.time = time.monotonic()
            pass

    # Turn off the backlight to save power
    def off(self, sleep_time=0.05):
        # Initializations
        self._display.off()

    def main_menu(self, sleep_time=0.05):
        self.fill(self.color('purple'))
        self.draw_text("banner_text", 20, 100, "Hello World!",
                       self.color('white'),
                       self.font)

        self.draw_text("banner_text", 50, 150, "_ฅ^•ﻌ•^ฅ_",
                        self.color('white'),
                        self.font)
        # self.draw_text("banner_text", 60, 200, "ฅ",
 
        while True:
            self.update() 


