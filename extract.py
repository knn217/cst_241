# Python program to convert JSON to Python
import os
import numpy as np

def getDir(file_name):
    full_path = os.path.realpath(__file__)
    path, filename = os.path.split(full_path)
    dir = os.path.join(path, file_name)
    print(dir)
    return dir

def saveToTxt(data, name):
    dir = getDir(name)
    with open(dir, 'w', encoding='utf8') as f:
        for line in data:
            #print(line)
            f.write(line)
    return

def smartParse(str):
    str = str.replace("\n","")
    #print(str)
    try:
        i = int(str)
        return i
    except:
        pass
    try:
        f = float(str)
        return f
    except:
        pass
    return str

def extractCSV(txt, filter=[]):
    title = None
    data = []
    for line in txt:
        line_data = line.split(',')
        for i in filter:
            line_data.pop(i)
        if not title:
            title = [smartParse(i) for i in line_data]
        new_data = {}
        for i in range(len(line_data)):
            new_data.update({title[i]: smartParse(line_data[i])})
        #print(new_data)
        data.append(new_data)
    return data[1:]

def extractData():
    dir_edges = getDir('train.csv')
    data_edges = open(dir_edges, encoding='utf8')
    data_edges = extractCSV(data_edges, [13, 10, 9, 5, 4, 3, 2, 1, 0]) # pop filter from last to 1st

    dir_nodes = getDir('nodes.csv')
    data_nodes = open(dir_nodes, encoding='utf8')
    data_nodes = extractCSV(data_nodes)
    #print(data_nodes[0])
    
    # TRAIN.CVS ==========================================
    # s_node_id, e_node_id, length, street_id, street_name, long_snode, lat_snode, long_enode, lat_enode
    # [
    #    [..],
    #    [..],
    #    [..],
    #    ...
    #    [..],    
    # ]
    
    # NODES.CVS ==========================================
    # _id, long, lat
    # [
    #    [..],
    #    [..],
    #    [..],
    #    ...
    #    [..],    
    # ]
    
    print(np.asarray(data_edges))
    return data_nodes, data_edges
