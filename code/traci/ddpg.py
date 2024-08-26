# taking our search radius as 75
# we can have ten states with traffic light on (1) or off (-1) or absent(0) for our ego/emergency vehicle
# 1.no car ahead in current lane and no vehicles in adjacent lane --Traffic light(on/off/abs)
# 2.car ahead in current lane and vehicles in left lane --Traffic light(on/off/abs)
# 3.car ahead in current lane and vehicles in right lane --Traffic light(on/off/abs)
# 4.car ahead in current lane and vehicles in left and right lane --Traffic light(on/off/abs)
# 5.no car ahead in current lane and vehicles in left and right lane --Traffic light(on/off/abs)



# state format
# vehicle data = emergency_vehicle_id, speed, acceleration, lane_index
# neighbour data = vehicle_id, speed, acceleration, lane_index, distance_from_emergency_vehicle
# traffic light data = closest_traffic_light_id, distance_from_emergency_vehicle

import tensorflow as tf
from keras.models import Sequential
from keras.optimizers import Adam
from keras.layers import Dense
import numpy as np
import pickle
import random

class EmergencyVehicleState:
    def __init__(self, vehicle_data: list, neighbour_data: list, traffic_light_data: list):
        self.vehicle_data=vehicle_data
        self.neighbour_data=neighbour_data
        self.traffic_light_data=traffic_light_data
    
    def __repr__(self):
        return f"State {self.vehicle_data}, {self.neighbour_data}, {self.traffic_light_data}"

class EmergencyVehicleAction:
    def __init__(self):
        self.actions = ["M", "L", "R"]
        self.num_actions = len(self.actions)

    def action(self):
        return

class DQNAgent:
    def __init__(self, initial_state: EmergencyVehicleState, action_space: EmergencyVehicleAction, epsilon, epsilon_decay, epsilon_min, gamma=0, batch_size=0, episode=1, update_target_freq=5):
        self.state = initial_state
        self.q_network = self.build_q_network( 16, 3 )
        self.target_network = self.build_q_network( 16, 3 )
        self.episode=episode
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.gamma=gamma
        self.action_space = action_space
        self.action_size=3
        self.state_size=16
        self.batch_size=batch_size
        self.total_reward=0
        self.req_neighbour=3
        self.update_target_freq=update_target_freq

        if episode%5==0:
            self.update_flag=1

        if episode!=1:
            
            with open('.\\susang\\traci\\weights.pkl','rb') as file:
                dict_obj=pickle.load(file)
            
            self.q_network.set_weights(dict_obj['q_network_weights'])
            self.target_network.set_weights(dict_obj['target_network_weights'])

    def build_q_network(self, state_size, action_size):

        model = Sequential()
        model.add(Dense(units=64, input_dim=state_size, activation='relu'))
        model.add(Dense(units=64, activation='relu'))
        model.add(Dense(units=action_size, activation='linear'))
        optimizer = Adam(learning_rate=0.001)
        model.compile(optimizer=optimizer, loss='mse')

        return model
    
    def select_action(self):
        
        if np.random.rand()<=self.epsilon:
            if len(self.state.neighbour_data)>=self.req_neighbour:
                return np.random.choice(self.action_size)
            else:
                return -1
        else:
            if len(self.state.neighbour_data)>=self.req_neighbour:
                j = [] # flattening the list of neighbours into this variable

                for i in self.state.neighbour_data[0:self.req_neighbour]:
                    j=j+i[1:]

                data = self.state.vehicle_data[1:] + j + self.state.traffic_light_data[1:]

                data = np.array(data)

                data = data.reshape(-1, self.state_size)

                self.qvalues = self.q_network.predict(data)

                return np.argmax(self.qvalues)
            else:
                return -1
        
    def reward(self, action_index):

        tolerance=30 # very close to emergency vehicle
        experience=None
        next_state=None
        done=False
        tlreward=0
        
        if self.state.traffic_light_data[1]<10 and self.state.vehicle_data[1]==0:
            tlreward-=1

        if action_index == -1:
            return

        if self.action_space.actions[action_index] == 'M':
            if any(i[3]==self.state.vehicle_data[3] for i in self.state.neighbour_data):
                self.total_reward-=1 + tlreward
                experience = [self.state, 'M', -1 + tlreward, self.state, done]
                return experience
            else:
                self.total_reward+=0 + tlreward
                experience = [self.state, 'M', 0 + tlreward, self.state, done]
                return experience
        
        if self.action_space.actions[action_index] == 'L':
            if self.state.vehicle_data[3]==2 or any(i[3]==self.state.vehicle_data[3]+1 and i[4]<tolerance for i in self.state.neighbour_data):
                self.total_reward-=1 + tlreward
                experience = [self.state, 'L', -1 + tlreward, self.state, done]
                return experience
            else:
                self.total_reward+=1 + tlreward
                next_state=self.state
                next_state.vehicle_data[3]+=1
                experience = [self.state, 'L', 10 + tlreward, next_state, done]
                return experience
        
        if self.action_space.actions[action_index] == 'R':
            if self.state.vehicle_data[3]==0 or any(i[3]==self.state.vehicle_data[3]-1 and i[4]<tolerance for i in self.state.neighbour_data):
                self.total_reward-=1 + tlreward
                experience = [self.state, 'R', -1 + tlreward, self.state, done]
                return experience
            else:
                self.total_reward+=1 + tlreward
                next_state=self.state
                next_state.vehicle_data[3]-=1
                experience = [self.state, 'R', 10 + tlreward, next_state, done]
                return experience
        
    def train(self,experience):
        
        if len(self.state.neighbour_data)>=self.req_neighbour:
            j = [] # flattening the list of neighbours into this variable
            for i in self.state.neighbour_data[0:self.req_neighbour]:
                j=j+i[1:]
            nextdata = self.state.vehicle_data[1:] + j + self.state.traffic_light_data[1:]
            nextdata = np.array(nextdata)
            nextdata = nextdata.reshape(-1, self.state_size)
            self.target_qvalues = self.target_network.predict(nextdata)
            max_target_qvalues = np.max(self.target_qvalues)
            self.target_qvalues = experience[2] + self.gamma * max_target_qvalues * (1 - experience[4])
            # print(self.target_qvalues)

            j = [] # flattening the list of neighbours into this variable
            for i in experience[0].neighbour_data[0:self.req_neighbour]:
                j=j+i[1:]
            data = experience[0].vehicle_data[1:] + j + experience[0].traffic_light_data[1:]
            # print(data)
            data = np.array(data)
            data = data.reshape(-1, self.state_size)

            # print('qvalues ',self.qvalues)

            self.qvalues=self.q_network.predict(data)

            self.qvalues[0][np.argmax(self.qvalues)-1]=self.target_qvalues

            # print('new qvalues ',self.qvalues)

            self.q_network.train_on_batch(data, self.qvalues)

            # self.batch_trainer()

            if self.episode % self.update_target_freq == 0 and self.update_flag==1:
                self.update_flag=0
                self.target_network.set_weights(self.q_network.get_weights())

            return
        
    def decay(self):
        self.epsilon = max(self.epsilon*self.epsilon_decay, self.epsilon_min)

    def update_state(self, new_state: EmergencyVehicleState):
        self.state = new_state

    def save_experience_buffer(self, buffer, filename):
        with open(filename, 'wb') as file:
            pickle.dump(buffer, file)

    def load_experience_buffer(self, filename):
        with open(filename, 'rb') as file:
            buffer=pickle.load(file)
        return buffer
    
    def batch_trainer(self):

        with open('buffer.pkl', 'rb') as file:
            buffer=pickle.load(file)

        batch = random.sample(buffer,self.batch_size)
        
        for batch_exp in batch:

            j = [] # flattening the list of neighbours into this variable
            for i in batch_exp[3].neighbour_data[0:self.req_neighbour]:
                j=j+i[1:]
            nextdata = batch_exp[3].vehicle_data[1:] + j + batch_exp[3].traffic_light_data[1:]
            nextdata = np.array(nextdata)
            nextdata = nextdata.reshape(-1, self.state_size)
            tar_qval = self.target_network.predict(nextdata)
            max_target_qvalues = np.max(tar_qval)
            tar_qval = batch_exp[2] + self.gamma * max_target_qvalues * (1 - batch_exp[4])

            j=[]
            for i in batch_exp[0].neighbour_data[0:self.req_neighbour]:
                j=j+i[1:]
            batch_data = batch_exp[0].vehicle_data[1:] + j + batch_exp[0].traffic_light_data[1:]
            batch_data = np.array(batch_data)
            batch_data = batch_data.reshape(-1, self.state_size)
            qval = self.q_network.predict(batch_data)
            qval[0][np.argmax(qval)-1]=tar_qval
            self.q_network.train_on_batch(batch_data, qval)

        return 

    def __repr__(self):
        return f"State {self.state.vehicle_data}, {self.state.neighbour_data}, {self.state.traffic_light_data}"


