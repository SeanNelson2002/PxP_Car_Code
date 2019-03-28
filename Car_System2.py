from socket import *#Broadcasting
import sys#Normal system functions
import logging#logging data
import os#Operating System functions
import subprocess#CPU processing functions
import platform#Part of IMU
import re#Also part of IMU
import time#Used to get time
import math#Used for mathematical calculations
import datetime#Get time zone date time data
import RPi.GPIO as GPIO#Enables GPIO functionality
import smbus#Gives smbus functionality
import serial#Gets serial data
import board#get board functionalities
import busio#used for bus operations
import adafruit_vl6180x#time of flight distance sensor
from math import trunc#used for truncating data
import IMU#Import IMU for actiavting outside modules
# Sets up Broadcast connection
client_socket = socket(AF_INET, SOCK_DGRAM)
client_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
client_socket.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
client_socket.settimeout(5)
server_address = ('255.255.255.255',7777)
# Sets up Gyroscope
IMU_UPSIDE_DOWN = 0
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070 # [deg/s/LSB]
AA =  0.40# Complementary filter constant
################# Compass Calibration values ############
magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0
class Gyroscope():
    def kalmanFilterY ( accAngle, gyroRate, DT):
        y=0.0
        S=0.0
        global KFangleY
        global Q_angle
        global Q_gyro
        global y_bias
        global YP_00
        global YP_01
        global YP_10
        global YP_11
        KFangleY = KFangleY + DT * (gyroRate - y_bias)
        YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
        YP_01 = YP_01 + ( - DT * YP_11 )
        YP_10 = YP_10 + ( - DT * YP_11 )
        YP_11 = YP_11 + ( + Q_gyro * DT )
        y = accAngle - KFangleY
        S = YP_00 + R_angle
        K_0 = YP_00 / S
        K_1 = YP_10 / S
        KFangleY = KFangleY + ( K_0 * y )
        y_bias = y_bias + ( K_1 * y )
        YP_00 = YP_00 - ( K_0 * YP_00 )
        YP_01 = YP_01 - ( K_0 * YP_01 )
        YP_10 = YP_10 - ( K_1 * YP_00 )
        YP_11 = YP_11 - ( K_1 * YP_01 )
        return KFangleY
    def kalmanFilterX ( accAngle, gyroRate, DT):
        x=0.0
        S=0.0
        global KFangleX
        global Q_angle
        global Q_gyro
        global x_bias
        global XP_00
        global XP_01
        global XP_10
        global XP_11
        KFangleX = KFangleX + DT * (gyroRate - x_bias)
        XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
        XP_01 = XP_01 + ( - DT * XP_11 )
        XP_10 = XP_10 + ( - DT * XP_11 )
        XP_11 = XP_11 + ( + Q_gyro * DT )
        x = accAngle - KFangleX
        S = XP_00 + R_angle
        K_0 = XP_00 / S
        K_1 = XP_10 / S
        KFangleX = KFangleX + ( K_0 * x )
        x_bias = x_bias + ( K_1 * x )
        XP_00 = XP_00 - ( K_0 * XP_00 )
        XP_01 = XP_01 - ( K_0 * XP_01 )
        XP_10 = XP_10 - ( K_1 * XP_00 )
        XP_11 = XP_11 - ( K_1 * XP_01 )
        return KFangleX
    def GYRO(activated):
        if activated==True:
            a = datetime.datetime.now()
            IMU.detectIMU()#Detect if BerryIMUv1 or BerryIMUv2 is connected.
            IMU.initIMU()#Initialise the accelerometer, gyroscope and compass
            gyroXangle = 0.0
            gyroYangle = 0.0
            gyroZangle = 0.0
            CFangleX = 0.0
            CFangleY = 0.0
            kalmanX = 0.0
            kalmanY = 0.0
            #Read the accelerometer,gyroscope and magnetometer values
            ACCx = IMU.readACCx()
            ACCy = IMU.readACCy()
            ACCz = IMU.readACCz()
            GYRx = IMU.readGYRx()
            GYRy = IMU.readGYRy()
            GYRz = IMU.readGYRz()
            MAGx = IMU.readMAGx()
            MAGy = IMU.readMAGy()
            MAGz = IMU.readMAGz()
            #Apply compass calibration
            MAGx -= (magXmin + magXmax) /2 
            MAGy -= (magYmin + magYmax) /2 
            MAGz -= (magZmin + magZmax) /2
            ##Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            #Convert Gyro raw to degrees per second
            rate_gyr_x =  GYRx * G_GAIN
            rate_gyr_y =  GYRy * G_GAIN
            rate_gyr_z =  GYRz * G_GAIN
            #Calculate the angles from the gyro.
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP
            #Convert Accelerometer values to degrees
            if not IMU_UPSIDE_DOWN:
                # If the IMU is up the correct way (Skull logo facing down), use these calculations
                AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
                AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
            else:
                #Us these four lines when the IMU is upside down. Skull logo is facing up
                AccXangle =  (math.atan2(-ACCy,-ACCz)*RAD_TO_DEG)
                AccYangle =  (math.atan2(-ACCz,-ACCx)+M_PI)*RAD_TO_DEG
            #Change the rotation value of the accelerometer to -/+ 180 and
            #move the Y axis '0' point to up.  This makes it easier to read.
            if AccYangle > 90:
                AccYangle -= 270.0
            else:
                AccYangle += 90.0
            #Complementary filter used to combine the accelerometer and gyro values.
            CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
            CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
            #Kalman filter used to combine the accelerometer and gyro values.
            kalmanY = Gyroscope.kalmanFilterY(AccYangle, rate_gyr_y,LP)
            kalmanX = Gyroscope.kalmanFilterX(AccXangle, rate_gyr_x,LP)
            if IMU_UPSIDE_DOWN:
                MAGy = -MAGy   #If IMU is upside down, this is needed to get correct heading.
            #Calculate heading
            heading = 180 * math.atan2(MAGy,MAGx)/M_PI
            #Only have our heading between 0 and 360
            if heading < 0:
                heading += 360
            ###################Tilt compensated heading#########################
            #Normalize accelerometer raw values.
            if not IMU_UPSIDE_DOWN:
                #Use these two lines when the IMU is up the right way. Skull logo is facing down
                accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
                accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            else:
                #Us these four lines when the IMU is upside down. Skull logo is facing up
                accXnorm = -ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
                accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            #Calculate pitch and roll
            pitch = math.asin(accXnorm)
            roll = -math.asin(accYnorm/math.cos(pitch))
            #Calculate the new tilt compensated values
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
            #The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
            #is also reversed. This needs to be taken into consideration when performing the calculations
            if(IMU.LSM9DS0):
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
            else:
                magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)
            #Calculate tilt compensated heading
            tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
            if tiltCompensatedHeading < 0:
                tiltCompensatedHeading += 360
            new_x=kalmanX
            new_y=kalmanY
            #Return array of tilt and pitch
            Angles=[new_x,new_y]
            return Angles
        else:
            #Return array of tilt and pitch as 0 if Gyroscope is still processig or in the process of activation
            Angles=[0,0]
            return Angles
#Returns GPS data
def parseGPS(data):
    #Checks for GPS data availability
    if data[0:6] == "$GPRMC":
        #Parse response
        sdata = data.split(",")
        #Return results with 0 if still activating or processing information
        if sdata[2] == 'V':
            return [0.0,0.0,0.0,0.0,0.0,0.0]
        #Concatenated string of time
        time = sdata[1][0:2] + ":" + sdata[1][2:4] + ":" + sdata[1][4:6]
        # decoded latitude
        lat = decode(sdata[3])
        # directional latitude
        dirLat = sdata[4]
        # decoded longitude
        lon = decode(sdata[5])
        # directional longitude
        dirLon = sdata[6]
        # speed
        speed = sdata[7]
        # track course
        trCourse = sdata[8]
        # concatenated string of the date
        date = sdata[9][0:2] + "/" + sdata[9][2:4] + "/" + sdata[9][4:6]
        # returns variables for the parseGPS method
        return [lat,dirLat,lon,dirLon,speed,trCourse]
def decode(coord):
    #Parse coordinate data
    x = coord.split(".")
    #Parse head data
    head = x[0]
    #Parse tail data
    tail = x[1]
    #Parse degrees data
    deg = head[0:-2]
    #parse minimal degrees data
    min = head[-2:]
    #return integer from a concatenated string of parsed data
    return int(min + "." + tail)
#Activates SMBus
bus = smbus.SMBus(1)
#Boolean variable if lights are on or off
Lights_on=0
# Defaults to luminosity index of 100
lux = 100
# Gets current time. Used for battery calculations
clocktime=time.time()
# While loop starts program and starts producing results
while True:
    # try to make a call to the board otherwise set everything to 0
    try:
        b1 = bus.read_i2c_block_data(0x77, 0x88, 24)
    except:
        b1=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    # Goes from lines 265-296 of calculating Raspberry Pi incoming data from GPIO Pins
    dig_T1 = b1[1] * 256 + b1[0]
    dig_T2 = b1[3] * 256 + b1[2]
    if dig_T2 > 32767 :
        dig_T2 -= 65536
    dig_T3 = b1[5] * 256 + b1[4]
    if dig_T3 > 32767 :
        dig_T3 -= 65536
    dig_P1 = b1[7] * 256 + b1[6]
    dig_P2 = b1[9] * 256 + b1[8]
    if dig_P2 > 32767 :
        dig_P2 -= 65536
    dig_P3 = b1[11] * 256 + b1[10]
    if dig_P3 > 32767 :
        dig_P3 -= 65536
    dig_P4 = b1[13] * 256 + b1[12]
    if dig_P4 > 32767 :
        dig_P4 -= 65536
    dig_P5 = b1[15] * 256 + b1[14]
    if dig_P5 > 32767 :
        dig_P5 -= 65536
    dig_P6 = b1[17] * 256 + b1[16]
    if dig_P6 > 32767 :
        dig_P6 -= 65536
    dig_P7 = b1[19] * 256 + b1[18]
    if dig_P7 > 32767 :
        dig_P7 -= 65536
    dig_P8 = b1[21] * 256 + b1[20]
    if dig_P8 > 32767 :
        dig_P8 -= 65536
    dig_P9 = b1[23] * 256 + b1[22]
    if dig_P9 > 32767 :
        dig_P9 -= 65536
        ####################
    #Try to write data back to bus to get results otherwise skip results if not ready to write
    try:
        bus.write_byte_data(0x77, 0xF4, 0x27)
        bus.write_byte_data(0x77, 0xF5, 0xA0)
    except:
        pass
    #Try to read written results from bus for display otherwise return results as 0
    try:
        data = bus.read_i2c_block_data(0x77, 0xF7, 8)
    except:
        data=[0,0,0,0,0,0]
    #Try to calculate variables from incoming data, sets all variables to zero if error called on trying to divide by 0.
    try:
        adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
        adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16
        var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
        var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
        t_fine = (var1 + var2)
        cTemp = (var1 + var2) / 5120.0
        fTemp = cTemp * 1.8 + 32
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (dig_P6) / 32768.0
        var2 = var2 + var1 * (dig_P5) * 2.0
        var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
        var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * (dig_P1)
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (dig_P9) * p * p / 2147483648.0
        var2 = p * (dig_P8) / 32768.0
        pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100
        altitude = ((1-(pressure/1013.25)**.190284)*145366.45)*.3048
    except:
        # Sets all variables to 0
        adc_p = 0
        adc_t = 0
        var1 = 0
        var2 = 0
        t_fine = 0
        cTemp = 0
        fTemp = 0
        var1 = 0
        var2 = 0
        var2 = 0
        var2 = 0
        var1 = 0
        var1 = 0
        p = 0
        p = 0
        var1 = 0
        var2 = 0
        pressure = 0
        altitude = 0
    #activate port variable
    port = "/dev/serial0"
    #set defaults for other read information
    heading = 0.0
    lux = 0.0
    altitude = 0.0
    elevation = 0.0
    #Sets variables for broadcast
    motor_overheat = False
    front_steering_overheat = False
    rear_steering_overheat = False
    board_overheat = False
    rear_left_suspension_pressure = 0.0
    rear_right_suspension_pressure = 0.0
    front_left_suspension_pressure = 0.0
    front_right_suspension_pressure = 0.0
    rear_left_ground_clearance_distance = 0.0
    rear_right_ground_clearance_distance = 0.0
    front_left_ground_clearance_distance = 0.0
    front_right_ground_clearance_distance = 0.0
    front_right_obstical_detection = False
    front_left_obstical_detection = False
    rear_right_obstical_detection = False
    rear_left_obstical_detection = False
    front_right_clearance_distance = 0.0
    front_left_clearance_distance = 0.0
    rear_right_clearance_distance = 0.0
    rear_left_clearance_distance = 0.0
    rear_steering_activated = False
    front_steering_activated = False
    satelite_link_count = 0
    #Giant aray of satelite geolocation
    satelite_geolocation = (str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+
                            str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+
                            str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+
                            str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0)+" "+str(0.0))
    #Sets up for electrical readings
    SHUNT_OHMS = 0.1
    MAX_EXPECTED_AMPS = 0.2
    #Try to read in data if available. Otherwise set to defaults
    try:
        ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
        ina.configure(ina.RANGE_16V, ina.GAIN_AUTO)
        Bus_voltage = ina.voltage()
        Bus_current = ina.current()
        Bus_power = ina.power()
        Shunt_voltage = ina.shunt_voltage()
        Supply_Voltage = ina.supply_voltage()
        info = [0.0,0.0,0.0,0.0,0.0,0.0]
        satelite_count = 0
    except:
        #Sets defaults if faliure to recieve data
        Bus_voltage = 7.2
        Bus_current = 1
        Bus_power = 1
        Shunt_voltage = 1
        Supply_Voltage = 1
        info = [0.0,0.0,0.0,0.0,0.0,0.0]
        satelite_count = 0
    try:#Try getting satelite data from the Raspberry Pi GPS component
       ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
       info = parseGPS(ser.readline())
       satelite_count = 16
    except OSError:#Default to 0 if data not available
       info = [0.0,0.0,0.0,0.0,0.0,0.0]
       satelite_count = 0
    try:#Try to read light sensor data
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        #Setup the GPIO pin number 22
        GPIO.setup(22, GPIO.OUT)
        GPIO.output(22,GPIO.HIGH)
        #Setup I2C connection
        i2c = busio.I2C(board.SCL,board.SDA)
        #Activate light sensor
        sensor = adafruit_vl6180x.VL6180X(i2c)
        #Read light sensor lux (luminosity index)
        lux = sensor.read_lux(adafruit_vl6180x.ALS_GAIN_1)
    except:#Default to 100 if faliure to recieve data
        lux=lux
    try:#Try to read in pitch and tilt data
        tilt = Gyroscope.GYRO(True)[0]
        pitch = Gyroscope.GYRO(True)[1]
    except OSError:#If error set to level
        tilt = 0
        pitch = 0
    try:#Try to go through headlight algorithm
        if ((tilt>25)or(tilt<-25)):#Blink headlights if tilt is over -25 or 25 degrees
            if Lights_on==0:
                #Activates headlights if currently off
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(21, GPIO.OUT)
                GPIO.setup(20, GPIO.OUT)
                GPIO.output(21,GPIO.HIGH)
                GPIO.output(20,GPIO.HIGH)
                GPIO.setup(16, GPIO.OUT)
                GPIO.setup(19, GPIO.OUT)
                GPIO.output(16,GPIO.HIGH)
                GPIO.output(19,GPIO.HIGH)
                Lights_on=1
            else:
                #Deactivates headlights if currently on
                Lights_on=0
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(21, GPIO.OUT)
                GPIO.setup(20, GPIO.OUT)
                GPIO.output(21,GPIO.LOW)
                GPIO.output(20,GPIO.LOW)
                GPIO.setup(16, GPIO.OUT)
                GPIO.setup(19, GPIO.OUT)
                GPIO.output(16,GPIO.LOW)
                GPIO.output(19,GPIO.LOW)
        else:
            if lux<25:
                #Turn headlights on with solid light
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(21, GPIO.OUT)
                GPIO.setup(20, GPIO.OUT)
                GPIO.output(21,GPIO.HIGH)
                GPIO.output(20,GPIO.HIGH)
                GPIO.setup(16, GPIO.OUT)
                GPIO.setup(19, GPIO.OUT)
                GPIO.output(16,GPIO.HIGH)
                GPIO.output(19,GPIO.HIGH)
                Lights_on=1
            else:
                #Turn headlights off if lux 25 and up
                Lights_on=0
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(21, GPIO.OUT)
                GPIO.setup(20, GPIO.OUT)
                GPIO.output(21,GPIO.LOW)
                GPIO.output(20,GPIO.LOW)
                GPIO.setup(16, GPIO.OUT)
                GPIO.setup(19, GPIO.OUT)
                GPIO.output(16,GPIO.LOW)
                GPIO.output(19,GPIO.LOW)
    except OSError: #Pass if data currently unavailable
        pass
    level = 0.00
    #Read battery level file
    file= open("//home//pi//Battery_Level.txt","r")
    try:#Try to read results
        #Get float data as a string from the file
        string = (str(file.read())).strip(" ")
    except:#Reset if results are unavailable
        file.write(str(100.0))
        file.close()
        string = 100.0
    if string <= "0":#Reset if results are 0 and below
        file.write(str(100.0))
        file.close()
    #Cast level from string to floating number
    level=float(string)
    #Add battery level if voltage is more than stable state
    #It means the battery is charging
    if Bus_voltage>7.35:
        #Get seconds of battery available
        Battery_Time_Seconds2 = (((1200/100)*level)/Bus_current)*60*60
        Battery_percentage_interval = ((1200/Bus_current)*60*60)/100.0
        #Get current interval of the loop time
        interval=(time.time()-clocktime)
        #Increase clocktime
        clocktime+=interval
        #Get total battery mw
        Battery = Battery_Time_Seconds2+interval
        #Convert battery mw to percentage
        Battery_percentage = round((Battery/Battery_percentage_interval),2)
        #Write results to file
        file.close()
        file = open("//home//pi//Battery_Level.txt","w")
        file.write(str(Battery/Battery_percentage_interval))
        file.close()
    else:
        #This means the battery is being used. Battery level will be decreasing
        #Same process as before except decreasing the percentage to battery level 
        Battery_Time_Seconds2 = (((1200/100)*level)/Bus_current)*60*60
        Battery_percentage_interval = ((1200/Bus_current)*60*60)/100.0
        interval=(time.time()-clocktime)
        clocktime+=interval
        Battery = Battery_Time_Seconds2-interval
        Battery_percentage = round((Battery/Battery_percentage_interval),2)
        file.close()
        file = open("//home//pi//Battery_Level.txt","w")
        file.write(str(Battery/Battery_percentage_interval))
        file.close()
    #Tells you if headlights are activated or deactivated
    Headlights_Activated=bool(Lights_on)
    #Create a large data packet of information to be broadcasted
    data_packet = (str(tilt)+
                   " "+str(pitch)+
                   " "+str(heading)+
                   " "+str(cTemp)+
                   " "+str(lux)+
                   " "+str(pressure)+
                   " "+str(altitude)+
                   " "+str(satelite_count)+
                   " "+str(info[0])+
                   " "+str(info[1])+
                   " "+str(info[2])+
                   " "+str(info[3])+
                   " "+str(info[4])+
                   " "+str(info[5])+
                   " "+str(Bus_voltage)+
                   " "+str(Bus_current)+
                   " "+str(Bus_power)+
                   " "+str(Shunt_voltage)+
                   " "+str(Supply_Voltage)+
                   " "+str(Battery_percentage)+
                   " "+str(Headlights_Activated))
    try:#Try to broadcast the data packet encoded
        client_socket.sendto(data_packet.encode(), server_address)
    except OSError:
        pass
