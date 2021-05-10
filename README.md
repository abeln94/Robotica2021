# Robótica 2020-2021

Grupo 011

- Abel Naya Forcano, 544125
- Adrián Martín Marcos, 756524
- Alberto Martínez Rodríguez, 764900

---

## Ejecución del script del trabajo


Ejecutar recorrido completo detectando a R2D2 (BB8 en caso de especificarlo) y usando desplazamiento por tiempos (odometría en caso de indicarlo), volcando el log en el fichero logs/trabajo.csv

```bash
python trabajo.py [-odo] [-bb8] -f trabajo.csv
```

Mostrar gráfica con el recorrido del log guardado

```bash
python read_log.py -f trabajo.csv
```

## Ejecución de los scripts de las prácticas

Ejecutar con radio 300mm y guardar el log en el fichero logs/ocho_d300.csv

```bash
python p2.py -d 300 -f ocho_d300.csv
```

Ejecutar el tracker volcando el log en el fichero logs/p3.csv

```bash
python p3.py -f p3.csv
```

Ejecutar con mapa 0 empezando en la casilla [0,0] con meta en la [2,0], volcando el log en el fichero logs/p4.csv

```bash
python p4.py -start 0 0 -end 2 0 -m mapa0.txt -f p4.csv
```
