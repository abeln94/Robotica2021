import numpy
import sys
from classes.Map import Map

if __name__ == "__main__":
    if len(sys.argv) > 1:
        map = Map()
        data = numpy.genfromtxt("./logs/" + sys.argv[1], delimiter=',')
        for i in range(1, len(data)):
            map.update( [data[i][1], data[i][2], data[i][3]] )
    else:
        print("Introduce el nombre del fichero a mostrar")