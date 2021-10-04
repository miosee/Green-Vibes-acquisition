# -*- coding: cp1252 -*-
import sys                      # pour sys.exit
import numpy as np              # pour creer les vecteurs
from matplotlib import pyplot as plt  # pour l'affichage
import time
import csv
import os

sampleFreq = 100

print("Fichiers trouvés :")
for root, dirs, files in os.walk('data'):
    for f in files:
        print(f)
print()
filename = ""
while True:
    try:
        a = input("Entrez le numéro du fichier à ouvrir : ")
        if a != "":
            filename = "data/experiment%i.csv" % int(a)
            csvFile = open(filename, mode='r', newline='')
        break
    except:
        print("Erreur : fichier non trouvé !")


if filename == "":
    sys.exit()

csvReader = csv.reader(csvFile, delimiter = ',', quotechar='"')
acc = []
E = []
I = []
labels = csvReader.__next__()
for row in csvReader:
    acc.append(float(row[0]))
    E.append(float(row[1]))
    I.append(float(row[2]))
csvFile.close()

n = len(acc)
data = [acc, E, I]
t = np.arange(n)/sampleFreq

fig, ax = plt.subplots(3,1)
fig.suptitle(filename)
for a in range(3):
    ax[a].plot(t, data[a])
    ax[a].set_ylabel(labels[a])
    ax[a].set_xlabel("t (s)")
plt.show()



    
