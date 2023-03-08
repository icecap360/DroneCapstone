import cv2
import os
import numpy as np
import sys
 
fname = '10Drone-Sunny-MedAlt'
ext = '.jpg'
suffix= 'Nature'
img = cv2.imread(fname+ext)
print(img.shape) # Print image shape
cv2.imshow("original", img)
 
# Cropping an image
imlen, imwid = 400, 640

# offlen = -300
# offwid = -100
#cropped_image = img[(img.shape[0]-imlen-250):img.shape[0]-250, (img.shape[1]-imwid-300):img.shape[1]-300]

offlen = 350
offwid = 360
if offlen > img.shape[0]-imlen:
    raise Exception('Requested len offset too big, max is'+str(img.shape[0]-imlen))
if offwid > img.shape[1]-imwid:
    raise Exception('Requested wid offset too big, max is '+str(img.shape[1]-imwid))
print((0+offwid),imwid+offwid)
cropped_image = img[(0+offlen):imlen+offlen, (0+offwid):imwid+offwid]

cropped_image_copy = cropped_image.copy()
cv2.drawMarker(cropped_image_copy, ( imwid//2, imlen//2,), (0,0,255), markerType=cv2.MARKER_CROSS, thickness=5)
# Display cropped image
cv2.imshow("cropped", cropped_image_copy)
print(cropped_image.shape, (imlen//2, imwid//2))
# Save the cropped image
cv2.imwrite(fname+'-'+suffix+ext, cropped_image)
 
cv2.waitKey(0)
cv2.destroyAllWindows()