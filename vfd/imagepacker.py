#!/usr/bin/python

import sys

image_input = [
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
	0,0,0,1,1,0,0,1,1,1,0,0,1,1,0,0,
	1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,
	1,1,1,0,0,1,1,1,1,1,0,0,0,0,0,1,
	1,1,1,1,0,0,1,1,0,0,1,1,1,1,1,1,
	0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
	0,0,0,1,1,0,0,1,1,1,0,0,1,1,0,0,
	1,1,0,0,0,1,1,0,0,1,1,0,0,0,1,1,
	0,0,0,0,0,1,1,0,0,1,1,0,0,0,1,1,
	0,0,0,1,0,0,1,1,0,0,0,0,1,1,0,0,
	0,0,1,1,0,0,1,1,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
	0,0,0,1,1,0,0,1,1,0,1,0,1,1,0,0,
	1,1,0,0,0,1,1,0,0,1,1,0,0,0,1,1,
	0,0,0,0,0,1,1,0,0,1,1,0,0,0,1,1,
	0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,
	0,0,0,1,1,1,1,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
	0,0,0,1,1,0,0,1,1,0,1,0,1,1,0,0,
	1,1,0,0,0,1,1,0,0,1,1,0,0,0,1,1,
	1,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,
	1,1,1,0,0,0,1,1,0,0,0,0,1,1,0,0,
	0,0,0,0,1,1,0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
	0,0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,
	1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,
	0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,
	0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,
	0,0,0,0,1,1,0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,0,1,0,1,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,
	0,0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,
	1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,
	0,0,0,0,0,1,1,0,0,1,1,0,0,0,1,0,
	0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,
	0,0,0,0,1,1,0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
	1,1,1,1,0,0,0,1,1,0,0,0,1,1,0,0,
	1,1,0,0,0,0,0,1,1,0,0,0,0,0,1,1,
	1,1,1,0,0,1,1,0,0,0,1,1,0,0,1,1,
	1,1,1,0,0,0,1,1,0,0,0,0,1,1,0,0,
	0,0,0,0,1,1,0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,0,0,1,0,0,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,1,1,1,1,1,0,0,0,1,1,1,1,1,0,
	0,0,0,0,0,1,1,0,0,0,0,0,1,1,1,1,
	1,0,0,1,1,1,1,1,0,0,1,1,1,1,1,0,
	0,0,0,0,1,1,1,1,1,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,0,0,1,0,1,0,0,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,1,1,0,0,0,1,1,0,0,1,1,0,0,0,0,
	0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,
	0,0,0,1,1,0,0,0,0,0,1,1,0,0,1,1,
	0,0,0,1,1,0,0,0,1,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,0,0,1,0,1,0,0,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,1,1,0,0,0,1,1,0,0,1,1,0,0,0,0,
	0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,
	0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,1,
	1,0,0,1,1,0,0,0,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,0,0,1,0,1,0,0,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,1,1,0,0,0,1,1,0,0,1,1,1,1,1,0,
	0,0,0,0,0,1,1,0,0,0,0,0,1,1,1,1,
	1,0,0,1,1,1,1,1,0,0,1,1,0,0,0,1,
	1,0,0,0,1,1,1,1,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,0,0,1,0,1,0,0,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,1,1,0,0,0,1,1,0,0,1,1,0,0,0,0,
	0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,
	0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,1,
	1,0,0,0,0,0,0,1,1,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,0,1,1,0,1,1,0,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,1,1,0,0,0,1,1,0,0,1,1,0,0,0,0,
	0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,
	0,0,0,1,1,0,0,0,0,0,1,1,0,0,1,1,
	0,0,0,1,0,0,0,1,1,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,0,1,1,0,1,1,0,1,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,1,1,1,1,1,0,0,0,1,1,0,0,0,0,
	0,0,0,0,0,1,1,1,1,1,0,0,1,1,1,1,
	1,0,0,1,1,1,1,1,0,0,1,1,1,1,1,0,
	0,0,0,1,1,1,1,1,0,0,0,1,1,1,1,1,
	1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,0
    ]

width = 112
height = 16

sys.stdout.write("struct vfd_sprite_t sprite_name = {%d, %d, %d, \n\t" % (width, height, len(image_input)))
sys.stdout.write("\"")

# pack 8 pixels at a time
for n in range(0, len(image_input), 8):
    byte = 0x00
    for i in range(0, 8):
        if image_input[n+i]:
            byte |= (1 << i)
    sys.stdout.write("\\x%02x" % byte)
    #if n != (len(image_input) - 8):
    #    sys.stdout.write(", ")
    #else:
    #    sys.stdout.write(" ")
    if (n + 8) % 128 == 0:
        sys.stdout.write("\" \\\n\t\"")

sys.stdout.write("\"};")
