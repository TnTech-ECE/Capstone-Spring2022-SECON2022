from shapedetector import ShapeDetector
from colorlabeler import ColorLabeler
from shapeLength import ShapeLength
import imutils
import cv2
import serial
import digitalio
import board
from PIL import Image, ImageDraw, ImageFont
from adafruit_rgb_display import ili9341
from adafruit_rgb_display import st7789  # pylint: disable=unused-import
from adafruit_rgb_display import hx8357  # pylint: disable=unused-import
from adafruit_rgb_display import st7735  # pylint: disable=unused-import
from adafruit_rgb_display import ssd1351  # pylint: disable=unused-import
from adafruit_rgb_display import ssd1331  # pylint: disable=unused-import

# First define some constants to allow easy resizing of shapes.
BORDER = 20
FONTSIZE = 48

# Configuration for CS and DC pins (these are PiTFT defaults):
cs_pin = digitalio.DigitalInOut(board.CE0)
dc_pin = digitalio.DigitalInOut(board.D25)
reset_pin = digitalio.DigitalInOut(board.D24)

# Config for display baudrate (default max is 24mhz):
BAUDRATE = 24000000

# Setup SPI bus using hardware SPI:
spi = board.SPI()

disp = st7735.ST7735R(spi, rotation=90,
    cs=cs_pin,
    dc=dc_pin,
    rst=reset_pin,
    baudrate=BAUDRATE,
)


# Create blank image for drawing.
# Make sure to create image with mode 'RGB' for full color.
if disp.rotation % 180 == 90:
    height = disp.width  # we swap height/width to rotate it to landscape!
    width = disp.height
else:
    width = disp.width  # we swap height/width to rotate it to landscape!
    height = disp.height
image = Image.new("RGB", (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0, 0, width, height), outline=0, fill=(0, 0, 0))
disp.image(image)

image = Image.open("/home/ttu-secon/Desktop/Vision/tech.bmp")

# Scale the image to the smaller screen dimension
image_ratio = image.width / image.height
screen_ratio = width / height
if screen_ratio < image_ratio:
    scaled_width = image.width * height // image.height
    scaled_height = height
else:
    scaled_width = width
    scaled_height = image.height * width // image.width
image = image.resize((scaled_width, scaled_height), Image.BICUBIC)

# Crop and center the image
x = scaled_width // 2 - width // 2
y = scaled_height // 2 - height // 2
image = image.crop((x, y, x + width, y + height))

# Load a TTF Font
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", FONTSIZE)

# Draw Some Text

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

text1 = "Happy"
(font_width, font_height) = font.getsize(text1)
draw.text(
    (width // 2 - font_width // 2, (height // 2 - font_height // 2) - (height//3)),
    text1,
    font=font,
    fill=(255, 221, 0),
)
text2 = "Mardi"
(font_width, font_height) = font.getsize(text2)
draw.text(
    (width // 2 - font_width // 2, height // 2 - font_height // 2),
    text2,
    font=font,
    fill=(255, 221, 0),
)
text3 = "Gras!"
(font_width, font_height) = font.getsize(text3)
draw.text(
    (width // 2 - font_width // 2, (height // 2 - font_height // 2) + (height//3)),
    text3,
    font=font,
    fill=(255, 221, 0),
)


# Display image.
disp.image(image)


# init video; USB cams on ports 0 and 2
video_capture_0 = cv2.VideoCapture(0)
video_capture_1 = cv2.VideoCapture(2)

ser = serial.Serial(port='/dev/serial0', baudrate=9600,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
ser.reset_input_buffer()
alreadyWritten0 = False
ser.write(b"Hello from Pi!\n")
#ser.close()
#print(ser.readline())

#load the image, convert it to grayscale, blur it slightly, and threshold it
while True:
    # init cameras
    ret0, frame0 = video_capture_0.read()
    ret1, frame1 = video_capture_1.read()
    frame0Contour = frame0.copy()
    frame1Contour = frame1.copy()
    cropped0 = frame0[0:300, 0:1000]
    cropped1 = frame1[0:300, 0:1000]
    resized0 = imutils.resize(cropped0, width=300)
    ratio0 = cropped0.shape[0] / float(resized0.shape[0])
    resized1 = imutils.resize(cropped1, width=300)
    ratio1 = cropped1.shape[0] / float(resized1.shape[0])


    blurred0 = cv2.GaussianBlur(resized0, (5,5), 0)
    gray0 = cv2.cvtColor(blurred0, cv2.COLOR_BGR2GRAY)
    lab0 = cv2.cvtColor(blurred0, cv2.COLOR_BGR2LAB)
    #elimWhite0 = cv2.threshold(gray0, 225, 255, cv2.THRESH_TOZERO_INV)[1]
    #elimDark0 = cv2.threshold(elimWhite0, 80, 255, cv2.THRESH_TOZERO)[1]
    thresh0 = cv2.threshold(gray0, 150, 255, cv2.THRESH_BINARY)[1]


    blurred1 = cv2.GaussianBlur(resized1, (5,5), 0)
    gray1 = cv2.cvtColor(blurred1, cv2.COLOR_BGR2GRAY)
    lab1 = cv2.cvtColor(blurred1, cv2.COLOR_BGR2LAB)
    #elimWhite1 = cv2.threshold(gray1, 225, 255, cv2.THRESH_TOZERO_INV)[1]
    #elimDark1 = cv2.threshold(elimWhite1, 80, 255, cv2.THRESH_TOZERO)[1]
    thresh1 = cv2.threshold(gray1, 150, 255, cv2.THRESH_BINARY)[1]

    cnts0 = cv2.findContours(thresh0.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts0 = imutils.grab_contours(cnts0)

    cnts1 = cv2.findContours(thresh1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts1 = imutils.grab_contours(cnts1)

    sd = ShapeDetector()
    cl = ColorLabeler()
    sl = ShapeLength()
    numCircles0 = 0

    for c in cnts0:
        #compute the center of the contour
        area = cv2.contourArea(c)
        '''M=cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / (M["m00"])) + 250
            cY = int(M["m01"] / (M["m00"]))
        else:
            cX = int(M["m10"] / (M["m00"] + 1))
            cY = int(M["m01"] / (M["m00"] + 1))'''
        shape = sd.detect(c)
        color = cl.label(lab0, c)
        length = sl.detect(c)
        #text = "{} {} {} {}".format(area, color,shape,length)
        

        '''c = c.astype(float)
        c *= ratio0
        c = c.astype(int)'''

        if area > 3000 and area < 5000 and color == "white" and shape == "circle":
            #draw the contour an dcenter of the shape on the image
            numCircles0 = numCircles0 + 1
            '''cv2.drawContours(cropped0, [c], -1, (0,255, 0), 2)
            cv2.circle(cropped0, (cX, cY), 7, (255,255,255), -1)
            cv2.putText(cropped0, text, (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)'''
            #ser.write(b'X')
            #print('X')

    #if numCircles0 == 0:
    #   ser.write(b'A')
    #    print('A')
        
    numCircles1 = 0

    for c in cnts1:
        #compute the center of the contour
        area = cv2.contourArea(c)
        
        '''M=cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / (M["m00"])) + 250
            cY = int(M["m01"] / (M["m00"]))
        else:
            cX = int(M["m10"] / (M["m00"] + 1))
            cY = int(M["m01"] / (M["m00"] + 1))'''
        shape = sd.detect(c)
        color = cl.label(lab1, c)
        length = sl.detect(c)
        #text = "{} {} {} {}".format(area, color,shape,length)
        

        '''c = c.astype(float)
        c *= ratio1
        c = c.astype(int)'''

        if area > 3000 and area < 5000 and color == "white" and shape == "circle":
            #draw the contour an dcenter of the shape on the image
            numCircles1 = numCircles1 + 1
            '''cv2.drawContours(cropped1, [c], -1, (0,255, 0), 2)
            cv2.circle(cropped1, (cX, cY), 7, (255,255,255), -1)
            cv2.putText(cropped1, text, (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)'''
            #ser.write(b'Z')
            #print('Z')
    
    if numCircles0 == 0 and numCircles1 == 0:
        ser.write(b'A')
        print('A')
    elif numCircles0 > 0 and numCircles1 == 0:
        ser.write(b'X')
        print('X')
    elif numCircles0 == 0 and numCircles1 > 0:
        ser.write(b'Z')
        print('Z')
    else:
        ser.write(b'W')
        print('W')
        
    ''' cv2.imshow("Video0", frame0)
    cv2.imshow("Video1", frame1)
    cv2.imshow("Cropped0", cropped0)
    cv2.imshow("Cropped1", cropped1)
    cv2.imshow("Thresh0", thresh0)
    cv2.imshow("Thresh1", thresh1)'''
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break


