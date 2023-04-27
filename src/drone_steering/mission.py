import numpy as np
import math

class DroneMission:
    def __init__(self, length, width):
        self.length = length
        self.width = width

    def __str__(self):
        return f"Mission length: {self.length}, mission width: {self.width}"

    def create_map(self):
        map = np.empty(length, width, 2) #2 bo muszą być 2 współrzędne

        for i in range(length):
            for j in range(width):
                    map[i][j][0] = i*4
                    map[i][j][1] = j*4

        for i in range(length):
            for j in range(width):
                    print("x: ", map[i][j][0], ", y: ", map[i][j][1])
        #chuj wie czy to zadziała tak jak ja chcę
        #tu można dopisać np trzeci atrybut mówiący o zarażeniu albo braku, można zmieniać numerek np

    #todo plan jest taki żeby stworzyć tabelę kółek które będą miały wsp xy i będzie mozna im zmienić stan zakażenia
    #tylko jak
    #chyba zależy jak tego mamy potem uzywac
    #bo możnaby dupnąć tak żeby tabela była wypełniona informacjami dla kolejnych pól z wsp xy oddalonymi o 4 od siebie
    #ale możnaby stworzyć jakąś tabelę kółek które będą realnie miały te wartości
    #to chyba musi być tabela 3d xDD no ale chuj


    def detection(self):
        # do dopracowania co ma się dziać jak tutaj się uda coś wykryć, czy zmieniać numerek w mapie czy co
        pass


orchard = DroneMission(25, 4)
orchard.create_map()


print(orchard)

# TODO na ten moment by się przydało napisać funkcje takie które mogą się przydać do oblatywania drona
#   chyba spoko byłoby zmontować wstępny sposób nadlatywania nad grupy drzew