#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.1
# Functions for the Adafruit PiOLED [SSD1306_128_32] display
# Datasheet: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-pioled-128x32-mini-oled-for-raspberry-pi.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: display
   :synopsis: Functions for displaying text and graphics on the OLED display

There is a header on the YRL040 PCB to which an Adafruit PiOLED 128x32 pixel
display can be attached, either directly or via a ribbon cable.  This module
augments the functionality of the ``Adafruit_SSD1306`` library, providing
functions to display graphics and text on the display.

.. moduleauthor:: James Hilder <github.com/jah128>


"""

import time, logging, subprocess, Adafruit_SSD1306, os
import yrk.settings as s

from PIL import Image, ImageDraw, ImageFont

OLED_BUS = s.OLED_BUS  # The display is attached to bus 5, which translates to RPi4 bus i2c_12

disp = Adafruit_SSD1306.SSD1306_128_32(rst=None,i2c_bus=OLED_BUS)

#Fonts [filepaths in settings.py]
mono_small = ImageFont.truetype(s.SMALL_MONO,8)
small_font = ImageFont.truetype(s.SMALL_FONT, 8)
small_font_bold = ImageFont.truetype(s.SMALL_BOLD, 8)
medium_font = ImageFont.truetype(s.LARGE_FONT,16)
large_font = ImageFont.truetype(s.LARGE_FONT,24)

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = 128
height = 32
draw_image = Image.new('1', (width, height))

# Clear display.
def clear():
    """A function to clear the display"""
    disp.clear()
    disp.display()

# Initialize library.
def init_display():
    """A function to initialise and clear the display"""
    disp.begin()
    clear()

# Send the image to the display.  image is a PIL image, 128x32 pixels.
def display_image(image):
    """Function to display a 128x32 pixel PIL image on the display

    Args:
        image (PIL image): The 128x32 PIL image

    """

    if(s.DISPLAY_ROTATED): image = image.transpose(Image.ROTATE_180)
    try:
        disp.image(image)
        disp.display()
    except IOError:
        logging.warning("IO Error writing to OLED display")

# Display a warning with specified text
def warning(text):
    """Function to display a warning graphics alongside a text message on display

    Args:
        text (str): The text message to display

    """

    draw_image = Image.open("images/warning.pbm")
    draw = ImageDraw.Draw(draw_image)
    draw.text((40,16),text,font=mono_small,fill=255)
    display_image(draw_image)

# Load and display an image
def display_image_file(image_filename):
    """Function to display a 128x32 pixel PBM image from file on the display

    Args:
        image_filename (str): The filename for the 128x32 PBM image

    """

    image = Image.open(image_filename)
    display_image(image)

def one_line_text(text):
    """Function to display one line of 32-pixel high text on display [no wrapping]

    Args:
        text (str): The message to display [8-chars max to be displayed]

    """

    draw = ImageDraw.Draw(draw_image)
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    draw.text((0,4),text,font=large_font,fill=255)
    display_image(draw_image)

# Display one lins of text; if the line is longer than 14 characters will shrink font + wrap (64 characters max)
def one_line_text_wrapped(text):
    """Function to display one message of text on display

    32-pixel high [1-line] font will be used unless message > 14 characters long,
    in which case either 16-pixel or 8-pixel high fonts will be used.

    Args:
        text (str): The message to display [64-chars max to be displayed]

    """

    draw = ImageDraw.Draw(draw_image)
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    lt = len(text)
    if(lt<9): draw.text((0,4),text,font=large_font,fill=255)
    elif(lt<28):
        if(lt>14):
            draw.text((0,0),text[0:14],font=medium_font,fill=255)
            draw.text((0,16),text[14:lt],font=medium_font,fill=255)
        else:
            draw.text((0,8),text,font=medium_font,fill=255)
    else:
        draw.text((0,0),text[0:16],font=mono_small,fill=255)
        end_pos = 32
        if(lt<33): end_pos=lt
        draw.text((0,8),text[16:end_pos],font=mono_small,fill=255)
        if(lt>32):
            end_pos = 48
            if(lt<49): end_pos=lt
            draw.text((0,16),text[32:end_pos],font=mono_small,fill=255)
            if(lt>48):
                end_pos=64
                if(lt<64): end_pos=lt
                draw.text((0,24),text[48:end_pos],font=mono_small,fill=255)
    display_image(draw_image)


# Display two lines of text; max 14 characters per line
def two_line_text(line1_text,line2_text):
    """Function to display two lines of text on display in 16-pixel high font [no wrapping]

    Args:
        line1_text (str): The first line of text to display [14-chars max to be displayed]
        line2_text (str): The first line of text to display [14-chars max to be displayed]

    """

    draw = ImageDraw.Draw(draw_image)
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    draw.text((0,0),line1_text,font=medium_font,fill=255)
    draw.text((0,16),line2_text,font=medium_font,fill=255)
    display_image(draw_image)

# Display two lines of text; if either line is longer than 14 characters will shrink font + wrap for that line (32 characters max per line)
def two_line_text_wrapped(line1_text,line2_text):
    """Function to display two lines of text on display

    Where lengths are < 15 characters, 16-pixel font used, else 8-pixel font used

    Args:
        line1_text (str): The first line of text to display [32-chars max to be displayed]
        line2_text (str): The first line of text to display [32-chars max to be displayed]

    """

    draw = ImageDraw.Draw(draw_image)
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    if(len(line1_text)<15):
        draw.text((0,0),line1_text,font=medium_font,fill=255)
    else:
        if(len(line1_text)<17):draw.text((0,0),line1_text,font=mono_small,fill=255)
        else:
            draw.text((0,0),line1_text[0:16],font=mono_small,fill=255)
            draw.text((0,8),line1_text[16:len(line1_text)],font=mono_small,fill=255)
    if(len(line2_text)<15):
        draw.text((0,16),line2_text,font=medium_font,fill=255)
    else:
        if(len(line2_text)<17):draw.text((0,16),line2_text,font=mono_small,fill=255)
        else:
            draw.text((0,16),line2_text[0:16],font=mono_small,fill=255)
            draw.text((0,24),line2_text[16:len(line2_text)],font=mono_small,fill=255)
    display_image(draw_image)

def display_string(d_str):
   draw = ImageDraw.Draw(draw_image)
   split_text=[d_str[i:i+16] for i in range(0, len(d_str), 16)]
   draw.rectangle((0,0,width,height), outline=0, fill=0)
   y_pos = 0
   for line in split_text:
      if y_pos<32:
          draw.text((0,y_pos),line,font=mono_small,fill=255)
      y_pos+=8
   display_image(draw_image)

def display_stats():
    """Function to display current system stats (IP, cpu load, temperatures, memory usage)"""
    draw = ImageDraw.Draw(draw_image)
    x = 0
    padding = -1
    top = padding
    bottom = height-padding
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)
    draw.text((x, top),  ("IP: %s" % utils.get_ip()),  font=small_font, fill=255)
    draw.text((x, top+8),  ("Load: %2.1f%%  MHz: %d" % (utils.get_cpu_load(), utils.get_arm_clockspeed())),  font=small_font, fill=255)
    draw.text((x, top+16),  ("CPU: %2.1fC  GPU:%2.1fC" % (utils.get_cpu_temp(),utils.get_gpu_temp())),  font=small_font, fill=255)
    mem = utils.get_mem_usage()
    draw.text((x, top+24),  ("Mem: %d/%dMB %2.2f%%" % (mem[0],mem[1],mem[2])),  font=small_font, fill=255)
    display_image(draw_image)

def get_ip():
    """Function to get IP address as a string using hostname system process"""
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True ).strip()
    return IP.decode()

#Command line test [will run when display.py is run directly]
if __name__ == "__main__":
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    logging.info("York Robotics Kit: Display Test")
    init_display()
    display_image_file("/home/pi/yrk/images/yrl-white.pbm")
    #time.sleep(0.6)
    time.sleep(4)
    two_line_text_wrapped("IP Address:",get_ip())
    os._exit(1)
