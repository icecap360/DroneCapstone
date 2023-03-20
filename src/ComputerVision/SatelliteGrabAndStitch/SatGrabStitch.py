import requests
from PIL import Image, ImageDraw
import urllib, os, math
from io import StringIO, BytesIO 
from math import log, exp, tan, atan, pi, ceil
import urllib.request

# Enter your api key here
api_key = 'Insert API KEY'

# url variable store url
url = "https://maps.googleapis.com/maps/api/staticmap?"

# center defines the center of the map,
# equidistant from all edges of the map.
center = "Dehradun"

# zoom defines the zoom
# level of the map

zoom = 19
sz = 640
scale = 2
szStr = "%dx%d" %(sz,sz)

poilat, poilong=43.262134,-79.930788

def getPointLatLng(lat,lng, x, y):
    parallelMultiplier = math.cos(lat * math.pi / 180)
    degreesPerPixelX = 360 / math.pow(2, zoom + 8)
    degreesPerPixelY = 360 / math.pow(2, zoom + 8) * parallelMultiplier
    pointLat = lat - degreesPerPixelY * ( y - sz / 2)
    pointLng = lng + degreesPerPixelX * ( x  - sz / 2)
    return (pointLat, pointLng)
    
def latfactor(lat):
    return ((sz*360) / (512*2**(zoom - 1)) * math.cos(lat))

longfactor=(sz*360) / (512* 2**(zoom - 1))

poilat= round(poilat/latfactor(poilat))*latfactor(poilat)
poilong= round(poilong/longfactor) * longfactor


poilat, poilong=poilat+latfactor(poilat)/2, poilong+longfactor/2


positions=[]

positions.append(str(poilat+latfactor(poilat+latfactor(poilat))) + ',' + str(poilong-longfactor))
positions.append(str(poilat+latfactor(poilat+latfactor(poilat))) + ',' + str(poilong))
positions.append(str(poilat+latfactor(poilat+latfactor(poilat))) + ',' + str(poilong+longfactor))

positions.append(str(poilat) + ',' + str(poilong-longfactor))
positions.append(str(poilat) + ',' + str(poilong))
positions.append(str(poilat) + ',' + str(poilong+longfactor))

positions.append(str(poilat-latfactor(poilat-latfactor(poilat))) + ',' + str(poilong-longfactor))
positions.append(str(poilat-latfactor(poilat-latfactor(poilat))) + ',' + str(poilong))
positions.append(str(poilat-latfactor(poilat-latfactor(poilat))) + ',' + str(poilong+longfactor))

new_image = Image.new('RGB', (sz*3, sz*3))

for i in range(9):
    final = Image.new("RGB", (sz, sz))
    urlparams = urllib.parse.urlencode({'key':api_key,
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
    
    x = (i % 3) * sz
    y = (i // 3) * sz
        
    # Paste the image onto the new image
    new_image.paste(final, (x, y))

    #lat = float(positions[i].split(',')[0])
    #long = float(positions[i].split(',')[1])


    #print('Top Left: ', getPointLatLng(lat, long, 0, 0))
    #print('Top Right: ', getPointLatLng(lat, long, sz, 0))
    #print('Bottom Left: ', getPointLatLng(lat, long, 0, sz))
    #print('Bottom Right: ', getPointLatLng(lat, long, sz, sz))
    #final.show()
    
new_image.save(str(poilat)+", "+str(poilong) + '.jpg')
