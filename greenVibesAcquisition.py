# -*- coding: cp1252 -*-
import sys                              # pour sys.exit
import serial                           # package de gestion du port serie
from struct import unpack               # pour decoder le "bytearray" reçu
import numpy as np                      # pour creer les vecteurs
from matplotlib import pyplot as plt    # pour l'affichage
import time
import csv
import os



# Parametres de l'acquisition
sampleFreq = 100

# Configuration du port serie
arduino = serial.Serial()    # creation de l'objet associe au port serie
arduino.baudrate = 115200    # definition du baudrate
arduino.port = 'COM6'        # selection du port serie
arduino.timeout = 0.5

print("Connection with the Arduino")
# Connexion au dsPIC
try:
    arduino.open()               # on tente d'etablir la connexion
except:
    print('Connection failed')   # En cas de probleme,
    sys.exit()                   # on ferme l'application
data = 'e'
while data != b'':
    data = arduino.read()
print('Connected')
time.sleep(2)

active = True

while active:
    while True:
        try:
            sampleNb = sampleFreq*int(input("Entrez la durée de l'acquisition en secondes (0 pour sortir) : "))
            if sampleNb >= 0:
                break
        except:
            pass
        print("La durée d'acquisition doit être un nombre entier strictement positif")


    if sampleNb == 0:
        active = False
    else:
        # Création du fichier de données
        test = False
        for folder in os.listdir():
            if folder == 'data':
                test = True
                break;
        if test == False:
            os.mkdir('data')

        test = True
        a = 0
        while test:
            a += 1
            filename = "./data/experiment%i.csv" % a
            test = os.access(filename, os.W_OK)

        labels = ["accX", "accY", "accZ", "E (V)", "I (A)"]
        csvFile = open(filename, mode='w', newline='')
        csvWriter = csv.writer(csvFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        csvWriter.writerow(labels)

        print("Acquisition started")
        arduino.write(b'S')               # Envoi du caractere de synchro
        count = 0
        a = 0
        data = np.zeros((5, sampleNb))
        while a < sampleNb:
            try:
                tmp = unpack('B',arduino.read(1))[0]
                if tmp == 0x55:
                    for b in range(3):
                        tmp = arduino.read(2)
                        tmp = unpack('h',tmp)       # convertit les 'Bytes' en l'entier correspondant (tmp devient un tuple de float)
                        data[b, a] = tmp[0]         # extrait le flottant du tuple (tmp devient un float)
                    for b in range(2):
                        tmp = arduino.read(4)
                        tmp = unpack('f',tmp)       # convertit les 'Bytes' en le flottant correspondant (tmp devient un tuple de float)
                        data[b+3, a] = tmp[0]       # extrait le flottant du tuple (tmp devient un float)
                    csvWriter.writerow(data[:,a])
                    a += 1
                else:
                    print("Error: '0x55' expected, %s read" % hex(tmp))
            except:
                count += 1
                if count > 10:
                    print('Acquisition aborted')
                    a = sampleNb
        arduino.write(b'E')               # Envoi du caractere de synchro
        csvFile.close()
        print("Acquisition finished")

        t = np.arange(sampleNb)/sampleFreq
        fig, ax = plt.subplots(5,1)
        fig.suptitle(filename)
        for a in range(5):
            ax[a].plot(t, data[a])
            ax[a].set_ylabel(labels[a])
            ax[a].set_xlabel("t (s)")
        plt.show()

# Deconnexion du dsPIC
arduino.close()

