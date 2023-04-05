import requests
from PIL import Image, ImageDraw
import urllib, os
from io import StringIO, BytesIO 
from math import log, exp, tan, atan, pi, ceil
import urllib.request
# Enter your api key here
api_key = "_your_api_key_"
 
# url variable store url
url = "https://maps.googleapis.com/maps/api/staticmap?"
 
# center defines the center of the map,
# equidistant from all edges of the map.
center = "Dehradun"
 
# zoom defines the zoom
# level of the map
dataFolder = 'SattaliteImagery'
positions = ['43.262134,-79.930788', '43.262726,-79.929861','43.262879,-79.929357',
              '43.263048,-79.928084', '43.261914,-79.888995', '43.262050,-79.888570', 
              '43.263738,-79.859466', '43.340457,-79.812696', '43.735510,-79.343423',
              '43.743797,-79.311667', '43.778002,-79.231436', '43.829786,-79.121294'
              ]
zoom = 21
sz = 400
scale = 2
szStr = "%dx%d" %(sz,sz)

for i in range(len(positions)):
    final = Image.new("RGB", (sz, sz))
    urlparams = urllib.parse.urlencode({'key':'AIzaSyCDtvdqQzxtHOt9dMR4WQPU50tO-L5K_qU',
                                        'center': positions[i],
                                        'zoom': str(zoom),
                                        'size': '%dx%d' % (sz,sz),
                                        'maptype': 'satellite',
                                        'sensor': 'false',
                                        'scale': scale})
    url = 'http://maps.google.com/maps/api/staticmap?' + urlparams


    # f=urllib.request.urlopen(url)
    # im=Image.open(BytesIO(f.read()))
    # final.paste(im, (0, 0))

    r = requests.get(url + "center =" + center + "&zoom =" +
                    str(zoom) + "&size = "+szStr+"&key =" +
                                api_key + "maptype=satellite&sensor = false")
    im=Image.open(BytesIO(r.content))
    final.paste(im,(0,0))
    
    #final.show()
    final.save(os.path.join(dataFolder,'image'+str(i)+'.png'))





# EARTH_RADIUS = 6378137
# EQUATOR_CIRCUMFERENCE = 2 * pi * EARTH_RADIUS
# INITIAL_RESOLUTION = EQUATOR_CIRCUMFERENCE / 256.0
# ORIGIN_SHIFT = EQUATOR_CIRCUMFERENCE / 2.0

# def latlontopixels(lat, lon, zoom):
#     mx = (lon * ORIGIN_SHIFT) / 180.0
#     my = log(tan((90 + lat) * pi/360.0))/(pi/180.0)
#     my = (my * ORIGIN_SHIFT) /180.0
#     res = INITIAL_RESOLUTION / (2**zoom)
#     px = (mx + ORIGIN_SHIFT) / res
#     py = (my + ORIGIN_SHIFT) / res
#     return px, py

# def pixelstolatlon(px, py, zoom):
#     res = INITIAL_RESOLUTION / (2**zoom)
#     mx = px * res - ORIGIN_SHIFT
#     my = py * res - ORIGIN_SHIFT
#     lat = (my / ORIGIN_SHIFT) * 180.0
#     lat = 180 / pi * (2*atan(exp(lat*pi/180.0)) - pi/2.0)
#     lon = (mx / ORIGIN_SHIFT) * 180.0
#     return lat, lon

# ############################################

# # a neighbourhood in Lajeado, Brazil:

# upperleft =  '43.262134,-79.930788'  
# lowerright = '43.261831,-79.929575'

# zoom = 20   # be careful not to get too many images!

# ############################################

# ullat, ullon = map(float, upperleft.split(','))
# lrlat, lrlon = map(float, lowerright.split(','))

# # Set some important parameters
# scale = 2
# maxsize = 640

# # convert all these coordinates to pixels
# ulx, uly = latlontopixels(ullat, ullon, zoom)
# lrx, lry = latlontopixels(lrlat, lrlon, zoom)

# # calculate total pixel dimensions of final image
# dx, dy = lrx - ulx, uly - lry

# # calculate rows and columns
# cols, rows = int(ceil(dx/maxsize)), int(ceil(dy/maxsize))

# # calculate pixel dimensions of each small image
# bottom = 120
# largura = int(ceil(dx/cols))
# altura = int(ceil(dy/rows))
# alturaplus = altura + bottom


# final = Image.new("RGB", (int(dx), int(dy)))
# for x in range(cols):
#     for y in range(rows):
#         dxn = largura * (0.5 + x)
#         dyn = altura * (0.5 + y)
#         latn, lonn = pixelstolatlon(ulx + dxn, uly - dyn - bottom/2, zoom)
#         position = ','.join((str(latn), str(lonn)))
#         print(x, y, position)
#         urlparams = urllib.parse.urlencode({'key':'AIzaSyCDtvdqQzxtHOt9dMR4WQPU50tO-L5K_qU',
#                                         'center': position,
#                                       'zoom': str(zoom),
#                                       'size': '%dx%d' % (largura, alturaplus),
#                                       #'size': '800x800',
#                                       'maptype': 'satellite',
#                                       'sensor': 'false',
#                                       'scale': scale})
#         url = 'http://maps.google.com/maps/api/staticmap?' + urlparams
#         f=urllib.request.urlopen(url)
#         im=Image.open(BytesIO(f.read()))
#         final.paste(im, (int(x*largura), int(y*altura)))
# final.show()
# final.save('image.png')