USO DEL SCRIPT QUE DAN:

# PREVIO: cambiar valor de variable PI a False en el fichero sample_matching.py

# DEBUG > 1 => Se muestran dos imágenes, una con todos los matches, y otra con solo los más importantes (los que se muestran con DEBUG = 1)

#------------------------------------------------------------------------------------------------------------
# Busca emparejamiento con lo que ve en la imágen dada con el flag -i
python sample_matching.py -i test[1|2|3].jpg -r [R2-D2_s|BB8_s].png

#------------------------------------------------------------------------------------------------------------
# Busca emparejamiento con lo que se ve por la webcam (se sale pulsando ESC, se guarda captura pulsando s)
python sample_matching.py -r [R2-D2_s|BB8_s].png

si has leido esto tienes mi respeto <3