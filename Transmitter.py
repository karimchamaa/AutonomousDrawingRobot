import serial
import time
import picamera
import math
import array
import PIL
import numpy
from PIL import Image
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
buttonPin = 22
ledPin1 = 23
ledPin2 = 24
GPIO.setup(buttonPin,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(ledPin1,GPIO.OUT)
GPIO.setup(ledPin2,GPIO.OUT)

while True:
    GPIO.output(ledPin1,GPIO.HIGH)
    GPIO.output(ledPin2,GPIO.LOW)
    print("System started")
    while True:
        inputValue=GPIO.input(buttonPin)
        if(inputValue == False):
            print("Black button pressed")
            GPIO.output(ledPin1,GPIO.LOW)
            Selection='1'
            break

    #Selection=input("Press 1 to take Image or 0 to process paint Image  ")

    if Selection=='0':
        MatrixImage=numpy.asarray(Image.open('Paint.jpg').convert('L'))
    elif Selection=='1':
        with picamera.PiCamera() as camera:
            camera.resolution=(2592,1944) 
            camera.brightness=40
            camera.sharpness=100
            camera.start_preview()
            time.sleep(5)
            camera.capture('CameraImage.jpg')
            camera.stop_preview()
 
        MatrixImage=[[255 for x in range(210)] for y in range(90)]#Define Matrix 90-270
        #scale the image to a smaller one by changing the basewidth 115- 86
        basewidth=80
        img=Image.open('CameraImage.jpg')
        img=img.crop((880,580,1820,1100))
        wpercent=(basewidth/float(img.size[0]))
        hsize=int((float(img.size[1])*float(wpercent)))
        img=img.resize((basewidth,hsize),PIL.Image.ANTIALIAS)
        img.save('ScaledCamera.jpg')
        #Scaled Iage to Matrix
        CameraImage=numpy.asarray(Image.open('ScaledCamera.jpg').convert('L'))
        #Add Values to the workspace side of Matrix
        for i in range(len(CameraImage)):#Scan Matrix up to down
            for j in range (len(CameraImage[0])):# Row outer loop and column inner
                MatrixImage[i+45][j+130]=CameraImage[i][j]
               


    #Find X and Y not mapped
    #Replace by indexes and get Mapped ValuesXNotMapped= list()
    XNotMapped=list()
    YNotMapped=list()
    XNotMappedGlobal=list()
    YNotMappedGlobal=list()
    for i in range(len(MatrixImage)):#Scan Matrix up to down
        for j in range (len(MatrixImage[0])):# Row outer loop and column inner
            if MatrixImage[i][j]<50:
                XNotM=i;
                YNotM=j;
                XNotMapped.append(XNotM)
                YNotMapped.append(YNotM)
                XNotMappedGlobal.append(XNotM)#Store them as global since they will be changed later 
                YNotMappedGlobal.append(YNotM)

    #print(len(XNotMapped))
    #Map x and Y
    indexArray=list()
    IndexArrayMapped=list()
    LiftArray=list()
    Newi=0 #Since I am not able to change indices of for loop
            # Python Start at 0
    for i in range(len(XNotMapped)):
        for j in range(len(XNotMapped)):
            MinDis=math.sqrt(pow((XNotMapped[j]-XNotMapped[Newi]),2) + pow((YNotMapped[j]-YNotMapped[Newi]),2))
            indexArray.append(MinDis)
        if 1 in indexArray:
            index=indexArray.index(1)
            Lift=0
        elif math.sqrt(2) in indexArray:
            index=indexArray.index(math.sqrt(2))
            Lift=0
        else:
            if(i==len(XNotMapped)-1):
                Lift=1
                index=indexArray.index(math.sqrt(0))
            else:
                for k in range(len(indexArray)):
                    if (indexArray[k]<100 and indexArray[k]>0):
                        index=k
                        Lift=1
                    
        XNotMapped[Newi]=10000
        YNotMapped[Newi]=10000
        IndexArrayMapped.append(Newi)
        LiftArray.append(Lift)
        Newi=index
        indexArray=list()#Need to empty it otherwise it append
        
    #Replace by indexes and get Mapped Values
    XMapped= list()
    YMapped=list()
    for i in range(len(IndexArrayMapped)):
        XMapped.append(XNotMappedGlobal[IndexArrayMapped[i]])
        YMapped.append(YNotMappedGlobal[IndexArrayMapped[i]])
    print("XMapped=",XMapped)
    print("YMapped=",YMapped)

    #Apply Inverse Kinematics Equations 
    Angle2= list()
    Angle1=list()
    Step1Array=list()
    Step2Array=list()
    for i in range(len(XMapped)):
        D=(pow(XMapped[i],2)+pow(YMapped[i],2)-pow(132,2)-pow(100,2))/(2*132*100)
        Ang2=math.atan2(math.sqrt(1-pow(D,2)),D)
        Angle2.append(math.degrees(Ang2))    
        A=100*math.sin(Ang2)
        B=132+(100*math.cos(Ang2))
        Ang1=math.atan2(YMapped[i],XMapped[i])-math.atan2(A,B)
        Angle1.append(math.degrees(Ang1))
        Step1=int(round((math.degrees(Ang1)*470)/9))
        Step2=int(round((math.degrees(Ang2)*470)/9))
        Step1Array.append(Step1)
        Step2Array.append(Step2)
        
    #print("Angle1=",Angle1)
    #print("Angle2=",Angle2)
    if Selection == '0':     
       print("Step1=",Step1Array)
       print("Step2=",Step2Array)
    #print("step3=",LiftArray)


    #Check if Computation is correct
    XCheck=list()
    YCheck=list()
    for i in range(len(XMapped)):
        XCh=132*math.cos(math.radians(Angle1[i]))+100*math.cos(math.radians(Angle1[i])+math.radians(Angle2[i]));
        YCh=132*math.sin(math.radians(Angle1[i]))+100*math.sin(math.radians(Angle1[i])+math.radians(Angle2[i]));
        XCheck.append(XCh)
        YCheck.append(YCh)
    #print("XCheck=",XCheck)
    #print("YCheck=",YCheck)
    
    GPIO.output(ledPin2,GPIO.HIGH)
    print("Computation done")
    #Serial Part
    ser=serial.Serial('/dev/ttyAMA0',9600)
    startmsg=str(b'r')
    ser.write(bytes(startmsg,encoding="ascii"))
    while True:
                 
          if ser.read(1) == b'S':
             GPIO.output(ledPin2,GPIO.LOW)
             print("Red button pressed")
             break
           
    for i in range(len(Step1Array)):
        message1=str(Step1Array[i]+1000)
        message2=str(Step2Array[i]+1000)
        message3=str(LiftArray[i])
        ser.write(bytes(message1,encoding="ascii"))
        ser.write(bytes(message2,encoding="ascii"))
        ser.write(bytes(message3,encoding="ascii"))
        #time.sleep(0.5)
        while True:
             if ser.read(1) == b'S':
                break
            
               
                       
