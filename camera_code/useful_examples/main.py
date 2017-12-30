import numpy as np
import cv2


def resizeImage(srcImg,destImg,factor=1):

    dsize = (srcImg.shape[1]//factor,srcImg.shape[0]//factor)

    destImg = cv2.resize( srcImg, dsize)
    print("after resizing ...")
    print(destImg.shape)
    return destImg


def display_webCam():
    print("hey")
    cap = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(1)

    if(cap.isOpened() or cap2.isOpened()):
        print("It is open!")
        while(True):
            # Capture frame-by-frame , ret is a boolean value
            ret, frame = cap.read()
            ret,frame2 = cap2.read()

            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

            # Display the resulting frame
            cv2.imshow('original',frame)
            cv2.imshow('grayscale',gray)
            cv2.imshow('web cam2',frame2)
            cv2.imshow('grayscale 2',gray2)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break 

    # When everything done, release the capture
    cap.release()
    cap2.release()
    cv2.destroyAllWindows()

import numpy as np
import cv2
from matplotlib import pyplot as plt

def tryCornerDetection():
	
	img = cv2.imread('lena.jpg')
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	corners = cv2.goodFeaturesToTrack(gray,25,0.6,5)
	corners = np.int0(corners)

	for i in corners:
		x,y = i.ravel()
		cv2.circle(img,(x,y),3,255,-1)

	plt.imshow(img),plt.show()

def main():
    tryCornerDetection()
    

    
if __name__ == "__main__":
    main()
