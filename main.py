from ili934xnew import ILI9341, color565
from machine import Pin, SPI
import glcdfont
import tt24

text = 'Now is the time for all good men to come to the aid of the party.'

power = Pin(5, Pin.OUT)
power.value(0)
pi = SPI(2, baudrate=20000000, miso=Pin(25), mosi=Pin(23), sck=Pin(19))
display = ILI9341(pi, cs=Pin(22), dc=Pin(21), rst=Pin(18), w=320,h=240,r=3)

import output
output.set_print(display.write)
