# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 14:41:59 2020

@author: BlackyCode
"""
##################################################################################################################
# Librerias
##################################################################################################################
from ppadb.client import Client
from PIL import ImageGrab
import cv2
import numpy as np
#import pyautogui as au
import time
##################################################################################################################
#variables y colores
##################################################################################################################
negroB=np.array([0, 0, 0], np.uint8)
negroA=np.array([5, 5, 5], np.uint8)
rojoB=np.array([120, 170, 80], np.uint8)
rojoA=np.array([255, 255, 255], np.uint8)
font = cv2.FONT_HERSHEY_SIMPLEX

f = []
g = []
asdasd = False
##################################################################################################################
# conexión al device
##################################################################################################################
adb = Client(host='127.0.0.1', port=5037)
devices = adb.devices()

if len(devices) == 0:
    print('no device attached')
    quit()

device = devices[0]
##################################################################################################################
# deteccion del color negro 
##################################################################################################################
def detectorblack():
    global asdasd
    x = 0
    while True:
        #736 cambio del 500
        frame = np.array(ImageGrab.grab(bbox=(0,790,570,850)))
        frame00 = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV,negroB,negroA)
        asd = False
        contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(frame, contornos, -1, (255,0,0), 3)
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 0:
                M = cv2.moments(c)
                if (M["m00"]==0): M["m00"]=1
                x = int(M["m10"]/M["m00"])
                y = int(M['m01']/M['m00'])
                #print(x,y)
                cv2.circle(frame00, (x,y), 7, (0,255,0), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame00, '{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                nuevoContorno = cv2.convexHull(c)
                cv2.drawContours(frame00, [nuevoContorno], 0, (255,0,0), 3)
                asd = True
                f.append(int(x))
       
        if asd:
            asdasd = True
            break
    cv2.destroyAllWindows()
##################################################################################################################
#deteccion del color rojo 
##################################################################################################################
def detectorred():
    global asdasd
    x = 0
    while True:
        #736 cambio del 500
        frame = np.array(ImageGrab.grab(bbox=(0,690,174,730)))
        frame00 = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frameHSV,rojoB,rojoA)
        asd = False
        contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(frame, contornos, -1, (255,0,0), 3)
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 50:
                M = cv2.moments(c)
                if (M["m00"]==0): M["m00"]=1
                x = int(M["m10"]/M["m00"])
                y = int(M['m01']/M['m00'])
                #print(x,y)
                cv2.circle(frame00, (x,y), 7, (0,255,0), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame00, '{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
                nuevoContorno = cv2.convexHull(c)
                cv2.drawContours(frame00, [nuevoContorno], 0, (255,0,0), 3)
                asd = True
                g.append(int(x))
       
        if asd:
            asdasd = True
            break
    cv2.destroyAllWindows()
##################################################################################################################
# Calcularemos el tiempo a presionar touch
##################################################################################################################
def play():
    detectorblack()
    detectorred()
    xtarget = f[0]
    print("target "+str(xtarget))
    #Eje x pantalla pc
    resXpc = 564
    #Eje x telefono
    resXan = 1080
    
    # El 20 corresponde a la distancia del personaje con la orilla
    xchar = g[0]+20
    
    print("char pos "+str(xchar))
    #print("posision mono "+str(xchar))
    #print("detector "+str(xtarjet))
    
    dif = (xtarget - xchar)
    print("Largo de la Barra PC "+ str(dif))
    dif = (dif *100) / resXpc
    
    dif = ((dif / 100) * resXan)*.98
    print("Tiempo de touch"+ str(dif))
    #print("diferencia de RRRs "+str(dif))
 

 
    device.shell(f'input touchscreen swipe 500 500 500 500 {int(dif)}')
##################################################################################################################
#
##################################################################################################################
while True:
    play()
    time.sleep(3)
    f=[]
    g = []
