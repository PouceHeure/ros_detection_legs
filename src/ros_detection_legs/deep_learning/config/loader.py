import json
import os 

PATH_FILE_CURRENT = os.path.dirname(os.path.realpath(__file__))
PATH_FILE_PARAMETERS = os.path.join(PATH_FILE_CURRENT,"parameters.json")

def load_parameters(path_file=PATH_FILE_PARAMETERS): 
    data = None
    with open(path_file) as f:
        data = json.load(f)
    return data 