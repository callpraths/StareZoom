#!/usr/bin/python

import numpy as np
import Image
import sys
import Tkinter
import ImageTk
import time
import LookingAt


# The regions of the zoomed area are:
# \  D  /
#   ---
#C | O | A
#   ---
# /  B  \
# Region O is the zoomed out area, A-D are the skewed areas
# The rest of the image area is referred to as Z

root = None
canvas = None
displayed_image = None
image_array = None 
canvas_image = None
gbs = 0
gy = 0
gx = 0
gp = 0
gq = 0
gr = 0

def update_image(y, x, p, q, r, yd, xd):
  global image_array
  global displayed_image
  global gbs
  global gy, gx
  # Find out how many channels the image has
  try:
    ymax, xmax, num_channels = image_array.shape
  except:
    ymax, xmax = image_array.shape
    num_channels = 1

  # Sanity check boudaries
  if y - r + yd < 0:
    yd = r - y
  if y + r + yd > (ymax-1)/gbs:
    yd = (ymax-1)/gbs - r - y
  if x - r + xd < 0:
    xd = r - x
  if x + r + xd > (xmax-1)/gbs:
    xd = (xmax-1)/gbs - r - x

  # Create the index matrix for the relevant part of the picture
  ly = (y - r) + min(0,yd)
  lx = (x - r) + min(0,xd)
  base_indices = np.indices((2*r+abs(yd), 2*r+abs(xd)))
  base_indices[0,:,:] = base_indices[0,:,:] + ly
  base_indices[1,:,:] = base_indices[1,:,:] + lx
  base_indices = base_indices * gbs

  # Get center
  sy = r+max(0,yd)
  sx = r+max(0,xd)
  cy = base_indices[0,sy,sx]
  cx = base_indices[1,sy,sx]

  #print("cy = " + str(cy))
  #print("cx = " + str(cx))

  # Create the index matrix in region O [y-q,y+q] X [x-q,x+q] 
  o_indices = base_indices[:, sy-q:sy+q, sx-q:sx+q]
  o_indices_y = o_indices[0,:,:]
  o_indices_x = o_indices[1,:,:]
  o_indices_y_new = (o_indices_y.astype(float) * float(p)/float(q) + (float(q) - float(p))/float(q) * float(cy)).astype(int)
  o_indices_x_new = (o_indices_x.astype(float) * float(p)/float(q) + (float(q) - float(p))/float(q) * float(cx)).astype(int)
  o_indices_new = np.array([o_indices_y_new, o_indices_x_new])
  #print(o_indices_new)

  # Create the index matrix in region A [y-r,y+r] X [x+q,x+r] 
  a_indices = base_indices[:, sy-r:sy+r, sx+q:sx+r]
  a_indices_y = a_indices[0,:,:]
  a_indices_x = a_indices[1,:,:]
  a_indices_x_new = ((float(p-q) * float (cx + r *gbs) + float(r-p) * a_indices_x.astype(float))/float(r-q)).astype(int)
  a_indices_y_new = ((a_indices_y.astype(float) - cy) * (a_indices_x_new.astype(float) - cx) / (a_indices_x.astype(float) - cx) + cy).astype(int)
  a_indices_new = np.array([a_indices_y_new, a_indices_x_new])
  for i in range(r-q):
    a_indices_new[:,0:i,r-q-1-i] = 0  
    a_indices_new[:,2*r-i:2*r,r-q-1-i] = 0  

  # Create the index matrix in region B [y+q,y+r] X [x-r,x+r]
  b_indices = base_indices[:, sy+q:sy+r, sx-r:sx+r]
  b_indices_y = b_indices[0,:,:]
  b_indices_x = b_indices[1,:,:]
  b_indices_y_new = ((float(p-q) * float (cy + r *gbs) + float(r-p) * b_indices_y.astype(float))/float(r-q)).astype(int)
  b_indices_x_new = ((b_indices_x.astype(float) - cx) * (b_indices_y_new.astype(float) - cy) / (b_indices_y.astype(float) - cy) + cx).astype(int)
  b_indices_new = np.array([b_indices_y_new, b_indices_x_new])
  for i in range(r-q):
    b_indices_new[:,i,0:r-q-i] = 0  
    b_indices_new[:,i,q+r+i:2*r] = 0  

  # Create the index matrix in region C [y-r,y+r] X [x-r,x-q] 
  c_indices = base_indices[:, sy-r:sy+r, sx-r:sx-q]
  c_indices_y = c_indices[0,:,:]
  c_indices_x = c_indices[1,:,:]
  c_indices_x_new = ((float(p-q) * float (cx - r *gbs) + float(r-p) * c_indices_x.astype(float))/float(r-q)).astype(int)
  c_indices_y_new = ((c_indices_y.astype(float) - cy) * (c_indices_x_new.astype(float) - cx) / (c_indices_x.astype(float) - cx) + cy).astype(int)
  c_indices_new = np.array([c_indices_y_new, c_indices_x_new])
  for i in range(r-q):
    c_indices_new[:,0:i,i] = 0  
    c_indices_new[:,2*r-i:2*r,i] = 0  

  # Create the index matrix in region D [y-r,y-q] X [x-r,x+r]
  d_indices = base_indices[:, sy-r:sy-q, sx-r:sx+r]
  d_indices_y = d_indices[0,:,:]
  d_indices_x = d_indices[1,:,:]
  d_indices_y_new = ((float(p-q) * float (cy - r *gbs) + float(r-p) * d_indices_y.astype(float))/float(r-q)).astype(int)
  d_indices_x_new = ((d_indices_x.astype(float) - cx) * (d_indices_y_new.astype(float) - cy) / (d_indices_y.astype(float) - cy) + cx).astype(int)
  d_indices_new = np.array([d_indices_y_new, d_indices_x_new])
  for i in range(r-q):
    d_indices_new[:,r-q-1-i,0:r-q-i] = 0  
    d_indices_new[:,r-q-1-i,q+r+i:2*r] = 0  


  # Paste updated indices back
  base_indices[:, sy-r:sy+r, sx-r:sx+r] = 0
  base_indices[:, sy-q:sy+q, sx-q:sx+q] = o_indices_new
  base_indices[:, sy-r:sy+r, sx+q:sx+r] = a_indices_new
  base_indices[:, sy+q:sy+r, sx-r:sx+r] = base_indices[:, sy+q:sy+r, sx-r:sx+r] + b_indices_new
  base_indices[:, sy-r:sy+r, sx-r:sx-q] = base_indices[:, sy-r:sy+r, sx-r:sx-q] + c_indices_new
  base_indices[:, sy-r:sy-q, sx-r:sx+r] = base_indices[:, sy-r:sy-q, sx-r:sx+r] + d_indices_new



  # Create the tiny image by sub-sampling
  if num_channels > 1:
    tiny_array = image_array[base_indices[0,:,:], base_indices[1,:,:], :]
  else:
    tiny_array = image_array[base_indices[0,:,:], base_indices[1,:,:]]
  tiny_image = Image.fromarray(tiny_array)
  displayed_image.paste(tiny_image,(lx,ly))
  gx = x; gy = y
  return

def image_from_scratch(y, x, p, q, r):
  global image_array
  global displayed_image
  global gbs
  global gy, gx
  # Find out how many channels the image has
  try:
    ymax, xmax, num_channels = image_array.shape
  except:
    ymax, xmax = image_array.shape
    num_channels = 1
  # Sanity check boundaries
  if 2*r > (ymax-1)/gbs:
    print('image too small')
    return
  if 2*r > (xmax-1)/gbs:
    print('image too small')
    return
  if y - r < 0:
    y = r
  if y + r > (ymax-1)/gbs:
    y = (ymax-1)/gbs - r
  if x - r < 0:
    x = r
  if x + r > (xmax-1)/gbs:
    x = (xmax-1)/gbs - r

  #print("ymax = " + str(ymax))
  #print("xmax = " + str(xmax))
  #print("num_channels = " + str(num_channels))


  # Create the index matrix for the complete picture
  base_indices = np.indices(((ymax-1)/gbs, (xmax-1)/gbs))
  base_indices = base_indices * gbs

  # Get center
  cy = base_indices[0,y,x]
  cx = base_indices[1,y,x]

  #print("cy = " + str(cy))
  #print("cx = " + str(cx))

  # Create the index matrix in region O [y-q,y+q] X [x-q,x+q] 
  o_indices = base_indices[:, y-q:y+q, x-q:x+q]
  o_indices_y = o_indices[0,:,:]
  o_indices_x = o_indices[1,:,:]
  o_indices_y_new = (o_indices_y.astype(float) * float(p)/float(q) + (float(q) - float(p))/float(q) * float(cy)).astype(int)
  o_indices_x_new = (o_indices_x.astype(float) * float(p)/float(q) + (float(q) - float(p))/float(q) * float(cx)).astype(int)
  o_indices_new = np.array([o_indices_y_new, o_indices_x_new])
  #print(o_indices_new)

  # Create the index matrix in region A [y-r,y+r] X [x+q,x+r] 
  a_indices = base_indices[:, y-r:y+r, x+q:x+r]
  a_indices_y = a_indices[0,:,:]
  a_indices_x = a_indices[1,:,:]
  a_indices_x_new = ((float(p-q) * float (cx + r *gbs) + float(r-p) * a_indices_x.astype(float))/float(r-q)).astype(int)
  a_indices_y_new = ((a_indices_y.astype(float) - cy) * (a_indices_x_new.astype(float) - cx) / (a_indices_x.astype(float) - cx) + cy).astype(int)
  a_indices_new = np.array([a_indices_y_new, a_indices_x_new])
  for i in range(r-q):
    a_indices_new[:,0:i,r-q-1-i] = 0  
    a_indices_new[:,2*r-i:2*r,r-q-1-i] = 0  

  # Create the index matrix in region B [y+q,y+r] X [x-r,x+r]
  b_indices = base_indices[:, y+q:y+r, x-r:x+r]
  b_indices_y = b_indices[0,:,:]
  b_indices_x = b_indices[1,:,:]
  b_indices_y_new = ((float(p-q) * float (cy + r *gbs) + float(r-p) * b_indices_y.astype(float))/float(r-q)).astype(int)
  b_indices_x_new = ((b_indices_x.astype(float) - cx) * (b_indices_y_new.astype(float) - cy) / (b_indices_y.astype(float) - cy) + cx).astype(int)
  b_indices_new = np.array([b_indices_y_new, b_indices_x_new])
  for i in range(r-q):
    b_indices_new[:,i,0:r-q-i] = 0  
    b_indices_new[:,i,q+r+i:2*r] = 0  

  # Create the index matrix in region C [y-r,y+r] X [x-r,x-q] 
  c_indices = base_indices[:, y-r:y+r, x-r:x-q]
  c_indices_y = c_indices[0,:,:]
  c_indices_x = c_indices[1,:,:]
  c_indices_x_new = ((float(p-q) * float (cx - r *gbs) + float(r-p) * c_indices_x.astype(float))/float(r-q)).astype(int)
  c_indices_y_new = ((c_indices_y.astype(float) - cy) * (c_indices_x_new.astype(float) - cx) / (c_indices_x.astype(float) - cx) + cy).astype(int)
  c_indices_new = np.array([c_indices_y_new, c_indices_x_new])
  for i in range(r-q):
    c_indices_new[:,0:i,i] = 0  
    c_indices_new[:,2*r-i:2*r,i] = 0  

  # Create the index matrix in region D [y-r,y-q] X [x-r,x+r]
  d_indices = base_indices[:, y-r:y-q, x-r:x+r]
  d_indices_y = d_indices[0,:,:]
  d_indices_x = d_indices[1,:,:]
  d_indices_y_new = ((float(p-q) * float (cy - r *gbs) + float(r-p) * d_indices_y.astype(float))/float(r-q)).astype(int)
  d_indices_x_new = ((d_indices_x.astype(float) - cx) * (d_indices_y_new.astype(float) - cy) / (d_indices_y.astype(float) - cy) + cx).astype(int)
  d_indices_new = np.array([d_indices_y_new, d_indices_x_new])
  for i in range(r-q):
    d_indices_new[:,r-q-1-i,0:r-q-i] = 0  
    d_indices_new[:,r-q-1-i,q+r+i:2*r] = 0  


  # Paste updated indices back
  base_indices[:, y-r:y+r, x-r:x+r] = 0
  base_indices[:, y-q:y+q, x-q:x+q] = o_indices_new
  base_indices[:, y-r:y+r, x+q:x+r] = a_indices_new
  base_indices[:, y+q:y+r, x-r:x+r] = base_indices[:, y+q:y+r, x-r:x+r] + b_indices_new
  base_indices[:, y-r:y+r, x-r:x-q] = base_indices[:, y-r:y+r, x-r:x-q] + c_indices_new
  base_indices[:, y-r:y-q, x-r:x+r] = base_indices[:, y-r:y-q, x-r:x+r] + d_indices_new

  # Create the image by sub-sampling
  if num_channels > 1:
    base_array = image_array[base_indices[0,:,:], base_indices[1,:,:], :]
  else:
    base_array = image_array[base_indices[0,:,:], base_indices[1,:,:]]
  displayed_image = Image.fromarray(base_array)
  gy = y; gx = x
  return

def incr_callback():
  global gr
  gr = gr + 10
  StareZoomUpdate(gy,gx)

def incq_callback():
  global gq, gr
  gq = gq + 10
  if(gq >= gr):
    gr = gr + 10
  StareZoomUpdate(gy,gx)

def incp_callback():
  global gp, gq, gr
  gp = gp + 10
  if(gp >= gq):
    gq = gq + 10
    if(gq >= gr):
      gr = gr + 10
  StareZoomUpdate(gy,gx)

def decp_callback():
  global gp
  if gp <= 10:
    return
  gp = gp - 10
  StareZoomUpdate(gy,gx)

def decq_callback():
  global gq, gp
  if gq <= 10 or (gq - gp <= 10 and gp <= 10):
    return
  gq = gq - 10
  if(gq <= gp):
    gp = gp - 10
  StareZoomUpdate(gy,gx)

def decr_callback():
  global gp, gq, gr
  if gr <= 10 or (gr - gq <= 10 and gq <= 10) or (gr - gq <= 10 and gq - gp <= 10 and gp <= 10):
    return
  gr = gr - 10
  if(gr <= gq):
    gq = gq - 10
    if(gq <= gp):
      gp = gp - 10
  StareZoomUpdate(gy,gx)

def closb_callback():
  sys.exit(0)

import tkFileDialog
import os
def open_callback():
  global tkpi, image_array
  imagename=tkFileDialog.askopenfilename(initialdir=os.getcwd())
  im = Image.open(imagename)
  image_array = np.array(im)
  image_from_scratch(gy,gx,gp,gq,gr)
  tkpi = ImageTk.PhotoImage(displayed_image)
  canvas.itemconfigure(canvas_image, image=tkpi)
  canvas.update_idletasks()

def StareZoomLaunch(imagename, bs, y, x, p, q, r):
  global root, tkpi, canvas, canvas_image
  global image_array
  global displayed_image
  global gbs
  global gy, gx, gp, gq, gr, gbs
  gp = p; gq = q; gr = r; gbs = bs
  im = Image.open(imagename)
  image_array = np.array(im)
  image_from_scratch(y,x,p,q,r)
  root = Tkinter.Tk()
  root.geometry('%dx%d'%(displayed_image.size[0]+100, displayed_image.size[1]))
  tkpi = ImageTk.PhotoImage(displayed_image)
  canvas = Tkinter.Canvas(root)
  canvas_image = canvas.create_image(0, 0, anchor='nw', image=tkpi)
  canvas.place(x=0,y=0,width=displayed_image.size[0],height=displayed_image.size[1])

  #buttons to increase/decrease stuff
  incp = Tkinter.Button(root, text="zoom area + ", command=incp_callback)
  incp.pack()
  incp.place(x=displayed_image.size[0], y=0)
  incq = Tkinter.Button(root, text="zoom level +", command=incq_callback)
  incq.pack()
  incq.place(x=displayed_image.size[0], y=30)
  incr = Tkinter.Button(root, text="zoom blend +", command=incr_callback)
  incr.pack()
  incr.place(x=displayed_image.size[0], y=60)
  decp = Tkinter.Button(root, text="zoom area - ", command=decp_callback)
  decp.pack()
  decp.place(x=displayed_image.size[0], y=90)
  decq = Tkinter.Button(root, text="zoom level -", command=decq_callback)
  decq.pack()
  decq.place(x=displayed_image.size[0], y=120)
  incr = Tkinter.Button(root, text="zoom blend -", command=decr_callback)
  incr.pack()
  incr.place(x=displayed_image.size[0], y=150)
  openb = Tkinter.Button(root, text="load image ", command=open_callback)
  openb.pack()
  openb.place(x=displayed_image.size[0], y=180)

  closb = Tkinter.Button(root, text="Exit   ", command=closb_callback)
  closb.pack()
  closb.place(x=displayed_image.size[0], y=300)
  root.mainloop()
  root.mainloop()

def StareZoomUpdate(y, x):
  global root, tkpi, canvas, canvas_image
  global gx, gy
  xd = x - gx; yd = y - gy
  update_image(gy, gx, gp, gq, gr, yd, xd)
  tkpi = ImageTk.PhotoImage(displayed_image)
  canvas.itemconfigure(canvas_image, image=tkpi)
  canvas.update_idletasks()

abs_lt = [30, 50]
abs_rb = [400, 200]

if __name__ == "__main__" :
  import thread
  thread.start_new_thread(StareZoomLaunch, ('pic5.jpg', 10, 450, 400, 40, 150, 200))
  
  time.sleep(3)
  while (True) :
    # averaging over 3 values for noise reduction
    faceTile = LookingAt.faceDetect(3)
    [yFace, xFace, waste, piece] = np.array(faceTile)[0].tolist()
    yPic = int((abs_rb[0]-yFace+0.0)/(abs_rb[0]-abs_lt[0])*displayed_image.size[0])
    xPic = int((xFace-abs_lt[1]+0.0)/(abs_rb[1]-abs_lt[1])*displayed_image.size[1])
    StareZoomUpdate(xPic,yPic)
