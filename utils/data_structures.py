"""
    @modified: Fri Feb 8, 2019
    @author: Ingrid Navarro 
    @file: data_structures.py
    @version: 1.0
    @brief: Testing data structures for ros packets
"""

bbox = (10, 20, 30, 30)
# El primer key seria el ID convertido a string
# objects_1 = { '1' : { 'class': 'sea marker', 'bbox' : bbox, 'color': 'red', 'lives': 50} }

# Aqui el ID puede ser la posicion del objeto dentro de la lista. 
objects_2 = { 'sea marker' : [{'bbox': bbox,  'color': 'red',   'lives': 40}, 
                              {'bbox': bbox,  'color': 'green', 'lives': 40}
                             ],

              'buoy' :       [{'bbox': bbox,  'color': 'white', 'lives': 40}, 
                              {'bbox': bbox,  'color': 'green', 'lives': 40}
                             ]
}

# Iterar con la estructura de datos 

def iterate_all(dic):
    for cls, obj in dic.iteritems():
        print cls 
        for i in range(len(obj)):
            print obj[i]


iterate_all(objects_2)

# Agregar elemento 
new_dict = {'bbox': bbox, 'color':'green', 'lives' : 40}
objects_2['sea marker'].append(new_dict)
iterate_all(objects_2)

# Modificar lives 
object_id = 0
objects_2['buoy'][object_id]['lives'] -= 1
iterate_all(objects_2)


