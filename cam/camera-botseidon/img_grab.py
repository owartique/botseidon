import cv2 as cv

cam= cv.VideoCapture(0)
cam.set(cv.CAP_PROP_FRAME_WIDTH,640)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cv.namedWindow("test")

img_counter=0

while True:
    ret,frame=cam.read()
    if not ret:
        print("failed to grab frame")
    
    if ret:
        cv.imshow("test",frame)
        
        k=cv.waitKey(1)
        if k%256 ==27:
            #ESC pressed
            print("Escape pressed")
            break
        
        elif k%256 == 32:
            #Space pressed
            img_name="opencv_frame_{}.png".format(img_counter)
            cv.imwrite(img_name,frame)
            print("{} written".format(img_name))
            img_counter +=1
cam.release()
cv.destroyAllWindows()
