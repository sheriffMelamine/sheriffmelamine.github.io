---
title: "Aruco Tag detection using OpenCV and Python"
date: 2024-02-03
draft: false
tags: ["image-processing","opencv"]

---
Aruco Tag, also known as AR Tag, is widely used in navigation and tracking purposes in robotics field. With machine learning and modern image processing techniques, it is not difficult to detect and identify AR tags. However, in this post, I will try to use the most basic image processing tools of OpenCV for the approach to that problem. 
##	Problem Statement
There are four photos of random settings where there are posters containing the AR Tags. Each of the posters are somewhat distorted due to perspective of the camera. For demonstration purposes, only the first setting will be shown in the latter sections.
|![Figure 1](/posts/a-simple-opencv/scene1.jpg#gallery) |![Figure 2](/posts/a-simple-opencv/scene2.jpg#gallery) |
|---|---|
|||
|![Figure 3](/posts/a-simple-opencv/scene3.jpg#gallery) |![Figure 4](/posts/a-simple-opencv/scene4.jpg#gallery) |

In these settings, there are two unique AR Tags as follows. Each of the previously shown photos have either one of these two AR tags. 

|![Figure 5](/posts/a-simple-opencv/marker1.jpg#gallery) |![Figure 6](/posts/a-simple-opencv/marker2.jpg#gallery)|
|---|---|
|AR Tag 1|AR Tag 2|

The task is to use image processing to identify which picture contains which AR tag and mark them accordingly. In this approach, only raw image processing will be used, not using any machine learning algorithms. Let's break down the code in segments for understanding it better.
##	Definition Segment
Only two modules are imported, they are NumPy and OpenCV. The operations will be mostly using tools from OpenCV, NumPy being the module to only provide the basis of the data structure.  

{{< highlight python >}}
import numpy as np
import cv2 as cv
{{< /highlight >}}

The first user defined function is the ARCtoNPC() function which maps the AR tags firstly to a numpy array of 9x9 dimension, since the AR tags can be imagined to be a 9x9 grid of black and white blocks. In this 9 blocks, only the 5x5 blocks that form the inner square is useful as the blocks surrounding them are fully black containing no useful information. Therefore, the returned array is of 5x5 size consisting of 1 (balck) or 0 (white).  
{{< highlight python >}}

def ARCtoNPC(img):
    img_shape=np.shape(img)[0]
    img_n=9
    dn=5
    start=(img_n-dn)*0.5
    parts=img_shape/img_n
    ret=np.zeros((dn,dn))
    for i in range(0,dn):
        for j in range(0,dn):
            a=int((start+i)*parts)
            b=int((start+i+1)*parts)
            c=int((start+j)*parts)
            d=int((start+j+1)*parts)
            m=np.sum(img[a:b,c:d].astype(np.int32))/(255*(b-a)*(d-c))
            ret[i,j]=0 if m<0.5 else 1
    return ret
{{< /highlight >}}

This function then can be used to convert two images, one containing the AR Tag of reference, and other containing the AR Tag to be detected. One of the array can be rotated by 90 degrees consecutively so that it is confirmed that at least one of the rotation matches with the AR Tag. 

{{< highlight python >}}
def matchDICT(img1,img2):
    arr=ARCtoNPC(img1)
    arr2=ARCtoNPC(img2)
    ret=False
    for i in range(0,4):
        text=""
        arr=np.rot90(arr)
        if np.array_equal(arr,arr2):
            ret=True
            break
    return ret
{{< /highlight >}}

These functions will be later used in final stages of the code after image processing. There are also many lists predefined to avert the array resizing complexity which will contain the array of pixels read from the image files of scenes.

{{< highlight python >}}
mark=[]			
img=[]			
imghsv=[]		
mask=[]			
res=[]			
thresh=[]		
cnt=[]
shapes=[]
roi=[]
marks=[]
approx=[]
{{< /highlight >}}

This ends the first segment of the code. In the following segments, the defined data structures or functions will be used, as well as the OpenCV tools.

##	Image Processing Tools
At first, the images would be converted to NumPy arrays from the .jpg format. The ```img``` list would contain all the Numpy arrays corresponding to the images, and the ```shapes``` list would contain their corresponding dimensions for convenience of calculation.

{{< highlight python >}}
for i in range(0,4):
    img.append(cv.imread("AR_Tag_Task/scene"+str(i+1)+".jpg"))
    shapes.append(np.shape(img[i]))
{{< /highlight >}}

Then, in the next loop, the major portion of image filtering and processing takes place. The following code contains all of that and it is explained in the next paragraph.

{{< highlight python >}}
for i in range(0,4):
    imghsv.append(cv.cvtColor(img[i],cv.COLOR_BGR2HSV)) 
    mask.append(cv.inRange(imghsv[i],np.array([0,0,150]).astype(np.uint8),np.array([179,50,255]).astype(np.uint8)))
    res.append(cv.bitwise_and(img[i],img[i],mask=mask[i]))
    res[i]=cv.cvtColor(res[i],cv.COLOR_BGR2GRAY)
    thresh.append(cv.threshold(res[i],100,255,0)[1])
    cnt.append(cv.findContours(thresh[i], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[0])
{{< /highlight >}}

At first, the list ```imghsv``` is formed by taking the converted HSV (Hue, Saturation, Value) values of the images. HSV values are useful here because we know that the AR Tags would have low saturation and the white blocks of the AR Tags would have higher values as they would be quite bright.
![Figure 7](/posts/a-simple-opencv/scene0HSV.jpg#gallery)
Using this idea, inRange function is used and the image is masked and appended to the list ```mask```. ```[0,0,150]``` is the lower bound representing moderate value and zero saturation and ```[179,0,255]``` is the upper bound representing zero saturation and high value (fully white). 
![Figure 8](/posts/a-simple-opencv/scene0mask.jpg#gallery)
Then that mask is used in ```bitwise_and``` operation with the BGR format of the images previously save in the list ```img``` and appended in the list ```res```. This list is then converted to grayscale format since grayscale format is less cumbersome for computations and also it has all the information necessary. 
![Figure 9](/posts/a-simple-opencv/scene0res.jpg#gallery)
The grayscale format is then thresholded and appended to ```thresh``` list so that the array only contains the values 255 and 0 which helps in binary operation. 
![Figure 10](/posts/a-simple-opencv/scene0thresh.jpg#gallery)
Then, finally, contours are detected from the thresholded arrays which are the boundaries of the black islands in the white sea for analogy and appended to the list ```cnt```.
![Figure 11](/posts/a-simple-opencv/scene0HSVwithContours.jpg#gallery)

The contours are detected, but we have to determine which contours are nearly squared shaped. Because the AR Tags would have a squared shaped outer boundary. For that, the principle of geometry is used that the square of the arc length of the contour should be approximately 16 times the area of the contour. two values ```12.8``` and ```19.2``` are chosen for the range of tolerance. Before that, contours are filtered on the basis of their area, as there might be some tiny contours that would actually be square shaped because of the nature or the shape of pixels. The following code shows these processes.

{{< highlight python >}}
for i in range(0,4):
    for cntss in cnt[i]:
        temp1=cv.contourArea(cntss)
        if 25000<temp1:
            temp2=cv.arcLength(cntss,True)
            if 12.8*temp1<temp2**2<19.2*temp1:
                approx.append(cv.approxPolyDP(cntss,0.1*cv.arcLength(cntss,True),True))
{{< /highlight >}}

The list ```approx``` is appended with the final operation to convert the corresponding contours to their approximate polygon vertices, which should be the corners of the square shaped boundaries of the detected AR Tags. In the next loop, the polygon vertices are used to transform the perspective to form squares, which is how the actual square shaped AR Tags would look like. They are appended in the ```roi``` list.

{{< highlight python >}}
for i in range(0,4):
    temp=cv.getPerspectiveTransform(approx[i].astype(np.float32),np.float32([[0,0],[383,0],[383,383],[0,383]]))
    roi.append(cv.warpPerspective(thresh[i],temp,(384,384)))
{{< /highlight >}}

![Figure 12](/posts/a-simple-opencv/scene0detectedAR.jpg#gallery)

##	Detecting AR Tags

The two unique AR tags ,as previously mentioned, are then converted to numpy array of readable format and their center of black square is extracted and appended to the list ```marks```. 


{{< highlight python >}}
for i in range(0,2):
    mark.append(cv.imread("AR_Tag_Task/marker"+str(i+1)+".jpg",0))
    marks.append((mark[i])[64:447,64:447])	
{{< /highlight >}}

Now the function that we previously defined to compare two AR tags comes handy, and it is used as follows.

{{< highlight python >}}
for i in range(0,4):
    text="MARKER 1" if matchDICT(marks[0],roi[i]) else "MARKER 2"
    for j in range(0,4):
        cv.line(img[i],approx[i][j,0],approx[i][j+1 if not j==3 else 0,0],(0,255,0),20)
{{< /highlight >}}

This not only detects the AR Tag, but also draws line surrounding its perimeters to visualize it. The next segment of the codes are just for convenience, as they are two properly position the text showing the detection results, and convert the array of the images to the .jpg format and save them.


{{< highlight python >}}
    mom= cv.moments(approx[i])
    org=(int(mom['m10']/mom['m00'])-200,int(mom['m01']/mom['m00'])+200)
    cv.putText(img[i],text,org,cv.FONT_HERSHEY_SIMPLEX,6,(0,255,255),20,cv.LINE_AA)
for i in range(0,4):
    cv.imwrite("AR_Tag_Task/scene"+str(i+1)+"final.jpg",img[i]) 
{{< /highlight >}}

![Figure 13](/posts/a-simple-opencv/scene0final.jpg#gallery)

Similary, Two or more contours in the same image can also be identified. In my GitHub repository named [A-Simple-OpenCV-Code-For-ARTag-Detection](https://github.com/sheriffMelamine/A-Simple-OpenCV-Code-For-ARTag-Detection), I have put two codes, both following the same approach. The first code is the one discussed here, and the other one is for detecting two AR Tags in the same picture and determining the midpoint between their center-to-center distance.

This problem was given as a task for recruitment in Team Interplaneter Software Sub-Team, which is the Mars Rover Team of BUET, in 2021. This is the solution that I had came up with, which I wanted to share for the readers.
##	The Complete Code

{{< highlight python >}}
import numpy as np
import cv2 as cv


def ARCtoNPC(img):
    img_shape=np.shape(img)[0]
    img_n=9
    dn=5
    start=(img_n-dn)*0.5
    parts=img_shape/img_n
    ret=np.zeros((dn,dn))
    for i in range(0,dn):
        for j in range(0,dn):
            a=int((start+i)*parts)
            b=int((start+i+1)*parts)
            c=int((start+j)*parts)
            d=int((start+j+1)*parts)
            m=np.sum(img[a:b,c:d].astype(np.int32))/(255*(b-a)*(d-c))
            ret[i,j]=0 if m<0.5 else 1
    return ret
    
    
def matchDICT(img1,img2):
    arr=ARCtoNPC(img1)
    arr2=ARCtoNPC(img2)
    ret=False
    for i in range(0,4):
        text=""
        arr=np.rot90(arr)
        if np.array_equal(arr,arr2):
            ret=True
            break
    return ret
    
mark=[]
img=[]
imghsv=[]
mask=[]
res=[]
thresh=[]
cnt=[]
shapes=[]
roi=[]
marks=[]
approx=[]

for i in range(0,4):
    img.append(cv.imread("AR_Tag_Task/scene"+str(i+1)+".jpg"))
    shapes.append(np.shape(img[i]))

for i in range(0,4):
    imghsv.append(cv.cvtColor(img[i],cv.COLOR_BGR2HSV)) 
    mask.append(cv.inRange(imghsv[i],np.array([0,0,150]).astype(np.uint8),np.array([179,50,255]).astype(np.uint8)))
    res.append(cv.bitwise_and(img[i],img[i],mask=mask[i]))
    res[i]=cv.cvtColor(res[i],cv.COLOR_BGR2GRAY)
    thresh.append(cv.threshold(res[i],100,255,0)[1])
    cnt.append(cv.findContours(thresh[i], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[0])

for i in range(0,4):
    for cntss in cnt[i]:
        temp1=cv.contourArea(cntss)
        if 25000<temp1:
            temp2=cv.arcLength(cntss,True)
            if 12.8*temp1<temp2**2<19.2*temp1:
                approx.append(cv.approxPolyDP(cntss,0.1*cv.arcLength(cntss,True),True))
                                                         
for i in range(0,4):
    temp=cv.getPerspectiveTransform(approx[i].astype(np.float32),np.float32([[0,0],[383,0],[383,383],[0,383]]))
    roi.append(cv.warpPerspective(thresh[i],temp,(384,384)))

for i in range(0,2):
    mark.append(cv.imread("AR_Tag_Task/marker"+str(i+1)+".jpg",0))
    marks.append((mark[i])[64:447,64:447])  

for i in range(0,4):
    text="MARKER 1" if matchDICT(marks[0],roi[i]) else "MARKER 2"
    for j in range(0,4):
        cv.line(img[i],approx[i][j,0],approx[i][j+1 if not j==3 else 0,0],(0,255,0),20)

    mom= cv.moments(approx[i])
    org=(int(mom['m10']/mom['m00'])-200,int(mom['m01']/mom['m00'])+200)
    cv.putText(img[i],text,org,cv.FONT_HERSHEY_SIMPLEX,6,(0,255,255),20,cv.LINE_AA)
for i in range(0,4):
    cv.imwrite("AR_Tag_Task/scene"+str(i+1)+"final.jpg",img[i])
    
{{ < /highlight > }}

---
