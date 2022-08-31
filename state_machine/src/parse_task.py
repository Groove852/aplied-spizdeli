import json
import os

def get_task(table_f,table_t):
    dir_path = os.path.dirname(os.path.realpath(__file__))	
    with open(dir_path+"/state_machine.json", "r") as read_file:
        data = json.load(read_file)
        from_str = data[table_f]
        table_from = list(data[table_f].keys())
        table_to = list(data[table_t].keys())
        table_from.sort(reverse = False)
        table_to.sort(reverse = True)
        object_from = [data[table_f][k] for k in table_from]
        object_to = [data[table_t][k] for k in table_to]
    return table_from,table_to,object_from,object_to
