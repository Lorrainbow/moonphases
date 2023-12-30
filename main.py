import math
from tm1650 import TM1650
import network
import socket
from time import sleep
import machine
import json
import gc
import urequests
gc.collect()
from machine import Pin
import utime
a
def Keypad4x4Read(cols,rows):
  for r in rows:
    r.value(0)
    result=[cols[0].value(),cols[1].value(),cols[2].value()]
    if min(result)==0:
      key=key_map[int(rows.index(r))][int(result.index(0))]
      r.value(1) # manages key keept pressed
      return(key)
    r.value(1)



# the light
from neopixel import Neopixel
pixels = Neopixel(1, 1, 18, "RGBW")


#motor
from SimplyRobotics import KitronikSimplyRobotics
board = KitronikSimplyRobotics()

# the led displays
left_pin1 = 21
left_pin2 = 22

right_pin1 = 26
right_pin2 = 27

left = TM1650(left_pin1, left_pin2)
right = TM1650(right_pin1, right_pin2)


'''
wiring for the keypad

PIPIN 	PIN
15        1 	COL 2
14        2 	ROW 1
13        3 	COL 1
12        4 	ROW 4
19        5 	COL 3
16        6 	ROW 3
17        7 	ROW 2
'''
#The servo pins are 15,14,13,12,19,18,17,16 for servo 0 -> servo 7
col_list=[13,15,19]
row_list=[14,17,16,12]

# set row pins to output and change array elements from
#    int to Pin objects, all set to high
for x in range(0,4):
    row_list[x]=Pin(row_list[x], Pin.OUT)
    row_list[x].value(1)

# set columns pins to input and change array elements 
#   from int to Pin objects. We'll read user input here
for x in range(0,3):
    col_list[x] = Pin(col_list[x], Pin.IN, Pin.PULL_UP)

# Create a map between keypad buttons and chars
key_map=[["1","2","3"],\
         ["4","5","6"],\
         ["7","8","9"],\
         ["*","0","#"]]


# turn off the light
pixels.fill((0, 0, 0))
pixels.show()





#calibrate the motor
def calibrateRoutine():
    left.display_integer(9999)
    right.display_integer(9999)
    print("--- Calibrate ---")
    calibration = False
    while calibration is not True :
        key=Keypad4x4Read(col_list, row_list)
        if key != None:
            print("You pressed: "+key)
            utime.sleep(0.3)        
            if key == "1":
                for i in range(10):
                    board.steppers[0].step("f")
                    utime.sleep(0.01)
            elif key == "3":
                for i in range(10):
                    board.steppers[0].step("r")
                    utime.sleep(0.01)
            elif key == "7":
                for i in range(1):
                    board.steppers[0].step("f")
                    utime.sleep(0.01)
            elif key == "9":
                for i in range(1):
                    board.steppers[0].step("r")
                    utime.sleep(0.01)

            elif key == "*":
                calibration = True
                

calibrateRoutine()
position = 100

# connect to wifi
ssid = "yourwifiname"
password = "yourwifipassword"

def connect():
    #Connect to WLAN
   
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    try:
        wlan.connect(ssid, password, channel=12)
    except OSError as error:
        print(f'error is {error}')
        
    while wlan.isconnected() == False:
        print('Waiting for connection...')
        sleep(1)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    return ip


try:
    ip = connect()
except KeyboardInterrupt:
    machine.reset()


# turn off the light
pixels.fill((50, 50, 50))
pixels.show()

#clear the leds
left.display_clear()
right.display_clear()


#api settings
headers = { 'Authorization': "Basic really long string goes here"}

# Start the main loop
print("--- Ready to get user inputs ---")
count=0
date=""
day=0
while True:
    key=Keypad4x4Read(col_list, row_list)
    
    if key != None:
        print("You pressed: "+key)
        utime.sleep(0.3)        
        if key == "*":
            print("start again")
            date=""
            count=0
            day=0
            left.display_clear()
            right.display_clear()
        elif key == "#":
            calibrateRoutine()
            date=""
            count=0
            day=0
            left.display_clear()
            right.display_clear()
            
        else:
            count+=1

            date = date+key        
            #05
            
            
            if count == 1:
                left.display_integer(int(date[0]))
            elif count == 2:
                day = int(date[0])*10
                day = day + int(date[1])
                if date[0] == "0":
                    day = (day/10)
                left.display_float(day)
            elif count == 3:
                print("day")
                print(day)
                if date[0] == "0":
                    day = float(day)
                    adding = float(date[2])                    
                    adding = adding/100
                    adding = float(adding)
                    adding =round(adding,2)
                    day = adding + day
                    #day = day + float(date[2] / 100)
                    left.display_float(day) 
                else:
                    day = day*10
                    day = day + int(date[2])
                    
                    left.display_integer(day)
                print(day)
            elif count == 4:
                if date[0] == "0": #we're in decimals 0.51
                    day = float(day)
                    adding = float(date[3])                    
                    adding = adding/1000
                    adding = float(adding)
                    adding =round(adding,3)
                    day = adding + day
                    #day = day + float(date[2] / 100)
                    left.display_float(day) 
                else:
                    day = day*10
                    day = day + int(date[3])
                    left.display_integer(day)            
            elif count == 5:
                right.display_integer(int(date[4]))
            elif count == 6:
                day = int(date[4])*10
                day = day + int(date[5])
                right.display_integer(day)        
            elif count == 7:
                day = day*10
                day = day + int(date[6])
                right.display_integer(day)
            elif count == 8:
                day = day*10
                day = day + int(date[7])
                right.display_integer(day)
                print("let's gooo")
                count = 0
                date = date[4:8]+"-"+date[2:4]+"-"+date[0:2]
                print(date)
                res = urequests.get("https://api.astronomyapi.com/api/v2/bodies/positions?longitude=-84.39733&latitude=33.775867&elevation=1&from_date="+date+"&to_date="+date+"&time=10%3A47%3A33", headers=headers)
                data = res.json()

                moon = data["data"]["table"]["rows"][1]["cells"][0]["extraInfo"]["phase"]["string"]
                print(moon)
                angel = data["data"]["table"]["rows"][1]["cells"][0]["extraInfo"]["phase"]["angel"]
                angel = float(angel)
                angel = int(angel)
                angel = (angel/360)*200
                print(angel)
                move = angel - position
                if move > 0:    
                    for i in range(move):
                        board.steppers[0].step("r")
                        utime.sleep(0.01)
                else:
                    for i in range(-move):
                        board.steppers[0].step("f")
                        utime.sleep(0.01)
                    
                position = angel
                
                
                date = ""
                left.display_clear()
                right.display_clear()
                 


conn.request("GET", "/api/v2/bodies/positions?longitude=-84.39733&latitude=33.775867&elevation=1&from_date=2023-12-11&to_date=2023-12-11&time=23%3A47%3A33", headers=headers)

res = conn.getresponse()
data = res.read()

