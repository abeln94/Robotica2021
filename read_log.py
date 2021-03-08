import time
import numpy
import sys
from argparse import ArgumentParser
from classes.Map import Map
import matplotlib.pyplot as plt
"""
1) Que por defecto solo muestre la gráfica (de golpe, sin animaciones ni robot dibujado, solo el recorrido). Para esto no hay que usar el Map, un if-else y un plot y ya está. La animación de la gráfica se debería mostrar si además se le pasa un parámetro adicional, por ejemplo un -a (de animate) o alguna otra cosa.
2) Que la reproducción dinámica se haga acorde al timestamp. Ahora mismo se 'reproduce' a full speed, lo rápido que el programa es capaz de dibujar, por lo que no se ve bien lo que realmente le ha costado (para esto hay que hacer sleeps acorde al tiempo del fichero y a lo que le cuesta al programa dibujar)
3) Algún parámetro extra que permita guardar la gráfica en una imagen (por ejemplo -i imagen.png). Esto debería ser relativamente facil, hay por ahí un imsave, pero tampoco es tan importante ya que manualmente podemos guardarla nosotros desde el propio plot
"""

parser = ArgumentParser()

parser.add_argument("-f", "--file", help="LogFile to print", type=str, required=True)
parser.add_argument("-i", "--image", help="Outputs an image file", type=str, default=None)
parser.add_argument("-a", "--animation", help="Animates the robot movement", action="store_true")

args = parser.parse_args()

if __name__ == "__main__":
    data = numpy.genfromtxt("./logs/" + args.file, delimiter=',')
    map = Map()
    if args.animation:
        old_time = data[1][0]
    else:
        plt.ion()
        plt.figure('Robot simulation')

    for i in range(1, len(data)):
        if args.animation:
            time.sleep( data[i][0] - old_time )
            old_time = data[i][0]
            map.update( [data[i][1], data[i][2], data[i][3]])
        else:
            map.update( [data[i][1], data[i][2], data[i][3]], i == (len(data)-1))

    if args.image:
        map.save("./" + args.image)

    time.sleep(5)