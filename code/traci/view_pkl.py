import pickle
import json
import numpy as np
from random import sample

with open('buffer.pkl', 'rb') as file:
    buffer=pickle.load(file)

# print(buffer[0][0].vehicle_data)

batch = sample(buffer,32)

print(batch)

# with open('.\\susang\\traci\\env.json','r') as file:
#     json_obj = json.load(file)

# with open('.\\susang\\traci\\env.json','w') as file:
#     json.dump(json_obj, file, indent=2)


# with open('.\\susang\\traci\\weights.pkl','rb') as file:
#     dict_obj=pickle.load(file)


