# -*- coding: cp1252 -*-
import sys                      # pour sys.exit
import serial                   # package de gestion du port serie
from struct import unpack       # pour decoder le "bytearray" reçu
import numpy as np              # pour creer les vecteurs
from matplotlib import pyplot   # pour l'affichage
import time
import csv


# Configuration du port serie
arduino = serial.Serial()    # creation de l'objet associe au port serie
arduino.baudrate = 115200    # definition du baudrate
arduino.port = 'COM14'       # selection du port serie
arduino.timeout = 0.5

print("synchro with the Arduino")
# Connexion au dsPIC
try:
    arduino.open()               # on tente d'etablir la connexion
except:
    print('Echec de la connexion')  # En cas de probleme,
    sys.exit()                      # on ferme l'application
data = 'e'
while data != b'':
    data = arduino.read()
print('Connected')
time.sleep(2)

# Parametres de l'acquisition
sampleNb = 1000                  # Nombre d'echantillons a lire
#samplePer = 10E-3                   # Periode d'echantillonnage



# Recuperation des donnees
mpuFile = open('mpu.csv', mode='w', newline='')
mpuWriter = csv.writer(mpuFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
mpuWriter.writerow(["accX", "accY", "accZ"])
elecFile = open('elec.csv', mode='w', newline='')
elecWriter = csv.writer(elecFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
elecWriter.writerow(["Voltage", "Current"])
           
print("Acquisition started")
arduino.write(b'S')               # Envoi du caractere de synchro
count = 0
a = 0
b = 0
c = 0
while max(a,c) < sampleNb:
    try:
        tmp = unpack('B',arduino.read(1))[0]
        if tmp == 0x55:
            data = np.ones(3)
            for b in range(3):
                tmp = arduino.read(2)
                tmp = unpack('h',tmp)       # convertit le 'Bytes' en le flottant correspondant (tmp devient un tuple de float)
                data[b] = tmp[0]            # extrait le flottant du tuple (tmp devient un float)
            mpuWriter.writerow(data)
            a += 1
        elif tmp == 0xF0:
            data = np.ones(2)
            for b in range(2):
                tmp = arduino.read(4)
                tmp = unpack('f',tmp)       # convertit le 'Bytes' en le flottant correspondant (tmp devient un tuple de float)
                data[b] = tmp[0]            # extrait le flottant du tuple (tmp devient un float)
            elecWriter.writerow(data)
            c += 1
        else:
            print("err = %i" % tmp)
    except:
        count += 1
        if count > 10:
            print('Acquisition aborted')
            c = sampleNb
arduino.write(b'E')               # Envoi du caractere de synchro
mpuFile.close()
elecFile.close()
print("Acquisition finished")
print(a)
print(c)

# Deconnexion du dsPIC
arduino.close()
#print(status)
