#!/usr/bin/env python3
import sys, re
from bs4 import BeautifulSoup

model= 'brick_box_3x1x3'
model_pattern = re.compile(r'^'+model)

print('open '+ sys.argv[1])
fd= open( sys.argv[1], "r")
soup = BeautifulSoup( fd, 'html.parser' )
fd.close()

poses= {}

for elt in soup.sdf.find_all('model') :
    name= elt.attrs['name']
    if model_pattern.match( name ) :
        poses[name]= str(elt.pose.contents[0]).split(' ')

print( '['+ ",\n".join( ['['+ ', '.join(poses[i]) +']' for i in poses ] ) +']' )
