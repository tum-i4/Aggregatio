#!/usr/bin/env python

import rosbag
import os
import copy
import pickle
import re
import numpy as np
from datetime import datetime, timedelta
from collections import defaultdict
from matplotlib import pyplot as plt
from matplotlib import rcParams
from matplotlib.legend_handler import HandlerTuple
rcParams['mathtext.fontset'] = 'stix' # latex font
rcParams['font.family'] = 'STIXGeneral'


class DataAnalysis:
    """
    DataAnalysis is a class that extracts the data from the rosbag files and analysis it
    """
    def __init__(self, bag_path, bag_file):
        self.bag = rosbag.Bag(os.path.join(bag_path, bag_file + ".bag"))
        # filename
        self.filename = bag_file

        # Data containers
        self.task_data = defaultdict(dict) # task spawning/completion
        self.task_distr = defaultdict(dict) # spawned task distriution
        self.opinion_map = defaultdict(dict) # knowledge aggregated map from logics
        self.observations_map = defaultdict(dict) # knowledge aggregated map from logics
        self.observations_data = defaultdict(dict) # saves list of opinioins with key = [counter]
        self.task_id_distr = defaultdict(dict) # saves task_id, same format as task_distr, [time][gird index] = task id
        #self.robot_map = np.zeros([20,20])
        self.robot_pos = defaultdict(dict)
        self.occupancy_map = None

        # Plotting
        self.fig_counter = 1
        plt.rcParams["figure.figsize"] = [20,6]

        # Get the map resolution/width etc.
        saved_map_info = True
        if saved_map_info:
            self.extract_map_info()
        else:
            # manually saved. Preferred for long tests as saving modified_occupancy_grid
            # requires a lot of memory and is always the same
            self.grid_map_origin_x = -5.0
            self.grid_map_origin_y = -5.0
            self.grid_map_resolution = 0.5
            self.grid_map_width = 20
            self.grid_map_height = 20


    def get_cell_index(self, x, y):
        cell_x = int((x - self.grid_map_origin_x) / self.grid_map_resolution)
        cell_y = int((y - self.grid_map_origin_y) / self.grid_map_resolution)

        index = cell_x + cell_y * self.grid_map_width
        return index

    def extract_map_info(self):
        """
        This function extracts the information from the topic "/modified_occupancy_grid"
        to get the basic information about the map
        """
        for topic, msg, time in self.bag.read_messages(topics='/modified_occupancy_grid'):
            #self.occupancy_map = msg
            self.grid_map_origin_x = msg.info.origin.position.x
            self.grid_map_origin_y = msg.info.origin.position.y
            self.grid_map_resolution = msg.info.resolution
            self.grid_map_width = msg.info.width
            self.grid_map_height = msg.info.height
            
        data = np.array(msg.data)
        data = data.reshape([20,20])
   
        # plotting map grid
        #print(data[:,0:18])
        #data2 = np.rot90(data)
        #print(data2[:,1:19])
        #plt.pcolormesh(np.transpose(data))
        #plt.show()
    
    def extract_observation_info(self):
        """
        This function extracts the information from the topic "/modified_occupancy_grid"
        to get the basic information about the map
        """
        msg_num = 1
        for topic, msg, time in self.bag.read_messages(topics='/partial_observation'):
            time_sec = time.to_nsec() / float(10**9)
            frequency = []
            net_believe = [] # avg (belief - disbelief)
            index_list = []
            so_opinions = [] # list of lists of format [index, belief, disbelief, uncertainty, baserate]
            for so in msg.partial_observation:
                occ_index = self.get_cell_index(so.pose.position.x, so.pose.position.y)
                net_bel = so.belief - so.disbelief
                #print(so.pose.position.x, " ", so.pose.position.y)
                #print(so.belief, " ", so.disbelief)
                so_opinions.append([occ_index, so.belief, so.disbelief, so.uncertainty,so.base_rate])
                try:
                    # if already inside index_list
                    tmp_index = index_list.index(occ_index)
                    net_believe[tmp_index] += net_bel
                    frequency[tmp_index] += 1.

                except ValueError:
                    index_list.append(occ_index)
                    frequency.append(1)
                    net_believe.append(net_bel)

            #print(frequency)
            #print(net_believe)
            #print(index_list)
            for i in range(len(index_list)):
                # divide by respective frequencies to get avg.
                self.observations_map[datetime.fromtimestamp((float(time_sec)))][index_list[i]] = net_believe[i] / frequency[i]
            #print(self.observations_map[datetime.fromtimestamp((float(time_sec)))])
            
            # store opinoins list in observations_data
            self.observations_data[msg_num] = so_opinions
            msg_num += 1
                
    def extract_robot_pos(self):
        """
        This function extracts the position of the robots as a function of time
        """
        for topic, msg, time in self.bag.read_messages(topics='/robot_0/amcl_pose'):
            time_sec = time.to_nsec() / float(10**9)
            
            # x,y position of robot at this time
            pos_list = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.robot_pos['robot_0'][datetime.fromtimestamp(time_sec)] = pos_list
        
        for topic, msg, time in self.bag.read_messages(topics='/robot_1/amcl_pose'):
            time_sec = time.to_nsec() / float(10**9)
            
            # x,y position of robot at this time
            pos_list = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            self.robot_pos['robot_1'][datetime.fromtimestamp(time_sec)] = pos_list

    def extract_opinion_map(self):
        """
        This function extracts the information from the topic "/opinion_map" to retrieve 
        the aggregated knowledge map craeted by the different logic types.
        """
        for topic, msg, time in self.bag.read_messages(topics='/opinion_map'):
            for index in range(len(msg.so_list)):
                time_sec = time.to_nsec() / float(10**9)
                self.opinion_map[datetime.fromtimestamp((float(time_sec)))][index] = msg.so_list[index]
   
        # Add initial grid of vacuous opinions.
        # To add it at the beginning give it time of 5min before the first published opinion.
        time_delta = timedelta(seconds = 5 * 60)
        time_keys = self.opinion_map.keys()
        init_time = min(time_keys) - time_delta
        vac_opinion = copy.deepcopy(self.opinion_map[time_keys[0]][self.opinion_map[time_keys[0]].keys()[0]])
        vac_opinion.belief = 0.
        vac_opinion.disbelief = 0.
        vac_opinion.uncertainty = 1.
        vac_opinion.base_rate = 0.5

        for i in self.opinion_map[time_keys[0]].keys():

            if self.opinion_map[time_keys[0]][i].belief != -1:
                # not a wall
                self.opinion_map[init_time][i] = vac_opinion
            else:
                self.opinion_map[init_time][i] = self.opinion_map[time_keys[0]][i]

    def extract_all_compeleted_tasks(self):
        """
        This function extracts information from "/all_completed_tasks" topic. The information
        is, which robot completed which task and when. This augments the information in the 
        self.task_data dictionary started from active_tasks.

        Execute AFTER extract_active_tasks.
        """
        for topic, msg, t in self.bag.read_messages(topics='/all_completed_tasks'):
            time_sec = t.to_nsec() / float(10**9)
            time = datetime.fromtimestamp(time_sec)
            # extract information
            robot = msg.robot_id
            task_x = msg.x
            task_y = msg.y
            
            # find task ID from position by checking same cell at time previous to this message
            pos_index = self.get_cell_index(task_x, task_y)
            
            # find closest time that is younger then 'time' from completing the task
            arr_td = np.array([(i - time).total_seconds() for i in self.task_id_distr.keys()])
            
            if np.all(arr_td >= 0):
                # take closest one
                closest_time_td = min(self.task_id_distr.keys(), key=lambda x: np.absolute(x - time))
                print('ALL TIMES OLDER THEN COMPLETION TIME! SHOULD NOT HAPPEN')
            else:
                neg_mask = np.where(arr_td <= 0)
                closest_time_index =np.where(arr_td== -1*min(np.absolute(arr_td[arr_td<=0])))[0][0]
                

                # check that index within keys, else choose last one
                if closest_time_index < len(self.task_id_distr.keys()):
                    closest_time = self.task_id_distr.keys()[closest_time_index]
                else:
                    print("COULD NOT FIND CLOSET TIME??")
                    print(arr_td)
                    closest_time = -9999
            
            ind = -1
            if pos_index in self.task_id_distr[closest_time].keys():
                ind = pos_index
            elif pos_index-1 in self.task_id_distr[closest_time].keys():
                ind = pos_index-1
            elif pos_index+1 in self.task_id_distr[closest_time].keys():
                ind = pos_index+1
            elif pos_index-20 in self.task_id_distr[closest_time].keys():
                ind = pos_index-20
            elif pos_index+20 in self.task_id_distr[closest_time].keys():
                ind = pos_index+20
            
            if ind != -1:
                task_id = self.task_id_distr[closest_time][ind]
                
                # save in dictionary
                self.task_data[task_id]['completed'] = time
                self.task_data[task_id]['robot_id'] = robot


    def extract_active_tasks(self):
        """
        This function extracts the information from the topic "/active_tasks" and puts it
        into the dictionary self.task_data to calculate the average cleaning times.

        Secondly it also creates a dictionary self.task_distr with [times][position index]
        as keys and contains the false positive list (empty for ground truth otherwise robot id) 
        """
        prev_msg_flag = False
        for topic, msg, time in self.bag.read_messages(topics='/active_tasks'):
            time_sec = time.to_nsec() / float(10**9)
            for task in msg.goal_list:
                ID = task.id
                if ID not in self.task_data.keys() or list(task.fp) != self.task_data[ID]['false_positive']:
                    self.task_data[ID]['spawned'] = datetime.fromtimestamp(time_sec)
                    self.task_data[ID]['false_positive'] = list(task.fp)
            
            
            # only use if all_tasks_completed not available. This counts FP despawning as 
            # completing tasks
            # check if a goal is achieved, i.e. task in previous msgs but not in this one
            '''
            if prev_msg_flag:
                for task in prev_msg.goal_list:
                    ID = task.id
                    msg_ids = [msg_task.id for msg_task in msg.goal_list]
                    if ID not in msg_ids:
                        self.task_data[ID]['completed'] = datetime.fromtimestamp(time_sec)
            '''
            # add to self.task_distr
            if len(msg.goal_list) != 0:
                for task in msg.goal_list:
                    # convert position into index
                    pos_index = self.get_cell_index(task.pose.position.x, task.pose.position.y)
                    self.task_distr[datetime.fromtimestamp(time_sec)][pos_index] = list(task.fp)
                    self.task_id_distr[datetime.fromtimestamp(time_sec)][pos_index] = task.id 
            else:
                # empty map, so add unoccpied space value (-1) to beginning
                self.task_distr[datetime.fromtimestamp(time_sec)][0] = [-1] 
            
            prev_msg = msg
            prev_msg_flag = True
            
        
        # extract completion tasks
        self.extract_all_compeleted_tasks()

    def ground_truth_completion_time(self):
        """
        This function calculates the average and variance time needed for the robots to
        complete a task after it is spawned.
        """
        completion_times = []
        for task_id in self.task_data.keys():
            # check that it is ground truth
            if len(self.task_data[task_id]['false_positive']) == 0:
                if 'completed' in self.task_data[task_id].keys():
                    duration = self.task_data[task_id]['completed'] - self.task_data[task_id]['spawned']
                    completion_times.append(duration.total_seconds())

        # calculate average
        avg_compl_time = np.mean(completion_times)

        # calculate the variance, ddof = 1 for unbiased estiamte.
        compl_times_stdev = np.std(completion_times, ddof=1) 
        #print("mean = {} and stdev = {}".format(avg_compl_time, compl_times_stdev))
        return avg_compl_time, compl_times_stdev

    def completion_time_history(self, plotting_flag):
        """
        This function plots the time history of when ground truth and false positive tasks
        were completed
        """
        gt_completion_times = []
        fp_completion_times = []

        first_spawn = self.task_data[1]['spawned']
        for task_id in self.task_data.keys():
            # check that it is ground truth
            if len(self.task_data[task_id]['false_positive']) == 0:
                if 'completed' in self.task_data[task_id].keys():
                    completed_time = self.task_data[task_id]['completed'] - first_spawn 
                    gt_completion_times.append(completed_time.total_seconds())
            else:
                if 'completed' in self.task_data[task_id].keys():
                    completed_time = self.task_data[task_id]['completed'] - first_spawn 
                    fp_completion_times.append(completed_time.total_seconds())


        # sort list from lowest to highest completion time
        #sorted_indexes = numpy.argsort(gt_completion_times)
        gt_completion_times = sorted(gt_completion_times)
        fp_completion_times = sorted(fp_completion_times)
        if plotting_flag:
            # Ground Thruths
            fig = plt.figure(self.fig_counter)
            gt_completion_number = range(1,len(gt_completion_times) + 1)
            plt.plot(gt_completion_times, gt_completion_number, label='GT completion times')
            plt.legend()
            plt.savefig('plots/gt_completion_history' + self.filename + '_{}.png'.format(self.fig_counter), bbox_inches='tight')
            self.fig_counter += 1  

            # False positives
            fig = plt.figure(self.fig_counter)
            fp_completion_number = range(1,len(fp_completion_times) + 1)
            plt.plot(fp_completion_times, fp_completion_number, label='fp completion times')
            plt.savefig('plots/fp_completion_history' + self.filename + '_{}.png'.format(self.fig_counter), bbox_inches='tight')
            self.fig_counter += 1

            #print(gt_completion_number)
            #print(gt_completion_times)
        return gt_completion_times, fp_completion_times

    def task_type_counter(self):
        """
        This function counts the number of ground truhts, false positives that have been cleaned 
        as well as the total number of ground truths and false positives that have been spawned. 
        """
        gt_cleaned = 0
        total_gt_spawned = 0
        fp_cleaned = 0
        total_fp_spawned = 0
        for task_id in self.task_data.keys():
            # check that it is ground truth
            if len(self.task_data[task_id]['false_positive']) == 0:
                total_gt_spawned += 1
                if 'completed' in self.task_data[task_id].keys():
                    gt_cleaned += 1
            else:
                total_fp_spawned += 1
                if 'completed' in self.task_data[task_id].keys():
                    fp_cleaned += 1
        
        return gt_cleaned, total_gt_spawned, fp_cleaned, total_fp_spawned
    
    def task_type_counter_r0(self):
        """
        This function counts the number of ground truths & false positives that have been cleaned 
        by robot 0. 
        """
        gt_cleaned = 0
        fp_cleaned = 0
        for task_id in self.task_data.keys():
            # check that it is ground truth
            if len(self.task_data[task_id]['false_positive']) == 0:
                if 'completed' in self.task_data[task_id].keys():
                    if self.task_data[task_id]['robot_id'] == 0: 
                        gt_cleaned += 1
            else:
                if 'completed' in self.task_data[task_id].keys():
                    if self.task_data[task_id]['robot_id'] == 0: 
                        fp_cleaned += 1
        
        return gt_cleaned, fp_cleaned
    
    def save_data(self):
        """
        This function saves the data in form of a list in a pickle dump. To do this, the datetime
        information mus be discareded. Only neccecary when performing SL analysis as SL requries
        python3 but extraction or rosbags not possible with python3.
        """
        tmp_opinion_map_dict = defaultdict(dict)
        sorted_keys = sorted(self.opinion_map.keys())
        
        print('LEN = ', len(self.opinion_map.keys()))
        for i in range(len(self.opinion_map.keys())):
            data_lists = []
            for ind in self.opinion_map[sorted_keys[i]].keys():
                tmp_so = self.opinion_map[sorted_keys[i]][ind]
                data_lists.append([ind, tmp_so.belief, tmp_so.disbelief, tmp_so.uncertainty, tmp_so.base_rate])         
                        
            tmp_opinion_map_dict[i] = data_lists


        save_file = open('./data/' + self.filename + '.pkl', 'wb')
        pickle.dump([self.observations_data, tmp_opinion_map_dict], save_file)
        save_file.close()


    def plot_opinion_maps(self):
        """
        This function plots both the opinion map and the task distribution side by side on a 
        2D grid, so that they can be visually compared.
        """
        # use robot positioning as temporal resolution
        if True:
            #  use robots as time
            time_list = sorted(self.robot_pos['robot_0'].keys())
        else:
            # alternatively use opinion map as temporal resolution
            time_list = sorted(self.opinion_map.keys())
            time_list = [tmp_t for tmp_t in time_list if tmp_t > datetime(year = 2020, month = 9, day = 8, hour = 16, minute=3, second = 19) and tmp_t < datetime(year = 2020, month = 8, day = 8, hour = 16, minute=7, second = 9)]
 
        for time in time_list:
            task_distr_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            
            # determine the times closest to 'time' in both arrays.
            # find closest time that is older i.e. smaller thatn time. (so dont see future)
            
            arr_td = np.array([(i - time).total_seconds() for i in self.task_distr.keys()])
            if np.all(arr_td >= 0):
                # take closest one
                closest_time_td = min(self.task_distr.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_td_index =np.where(arr_td== -1*min(np.absolute(arr_td[arr_td<0])))[0][0]
                closest_time_td = self.task_distr.keys()[closest_time_td_index]
            
            arr_op = np.array([(i - time).total_seconds() for i in self.opinion_map.keys()])
            if np.all(arr_op >= 0):
                # take closest one
                closest_time_op = min(self.opinion_map.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_op_index = np.where(arr_op==-1*min(np.absolute(arr_op[arr_op<0])))[0][0]
                closest_time_op = self.opinion_map.keys()[closest_time_op_index]
            
            # convert data to 2D array. The values are assigned as follows:
            # -1: unoccupied space
            #  0: false positive of robot 0
            #  1: fasle positive of robobt 1
            #  2: false positive of robot 0 and 1
            #  3: ground truth
            # This assignment only suitable for 1 or 2 robots.
            for pos_index in self.task_distr[closest_time_td].keys():
                fp_list = self.task_distr[closest_time_td][pos_index]
                if len(fp_list) == 0:
                    task_distr_array[0,pos_index] = 3
                elif len(fp_list) == 1:
                    task_distr_array[0, pos_index] = fp_list[0]
                elif len(fp_list) == 2:
                    task_distr_array[0, pos_index] = 2

            # believe array stores the belive that it is occupied
            # whilst uncertainty array stores the uncertainty
            believe_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            uncertainty_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            expected_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            base_rate_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            for pos_index in self.opinion_map[closest_time_op].keys():
                tmp_op = self.opinion_map[closest_time_op][pos_index]
                believe_array[0, pos_index] = tmp_op.belief
                uncertainty_array[0, pos_index] = tmp_op.uncertainty 
                base_rate_array[0, pos_index] = tmp_op.base_rate
                expected_array[0, pos_index] = tmp_op.belief + tmp_op.base_rate * tmp_op.uncertainty
                
            # add walls to the task_distr for reference
            wall_mask = [believe_array[0,:] == -1]
            task_distr_array[0,wall_mask[0]] = -2
               
            # reshape arrays into actual dimensions
            task_distr_array = np.reshape(task_distr_array, [self.grid_map_height, self.grid_map_width])
            believe_array = np.reshape(believe_array, [self.grid_map_height, self.grid_map_width])
            uncertainty_array = np.reshape(uncertainty_array, [self.grid_map_height, self.grid_map_width])
            expected_array = np.reshape(expected_array, [self.grid_map_height, self.grid_map_width])
            base_rate_array = np.reshape(base_rate_array, [self.grid_map_height, self.grid_map_width])
            # Determine the robot positions at this time
            closest_time_r_0 = min(self.robot_pos['robot_0'], key=lambda x: np.absolute(x-time))
            closest_time_r_1 = min(self.robot_pos['robot_1'], key=lambda x: np.absolute(x-time))
            robot_0_x, robot_0_y = self.robot_pos['robot_0'][closest_time_r_0]
            robot_1_x, robot_1_y = self.robot_pos['robot_1'][closest_time_r_1]
            
            # convert the robot pos into the correct indexes
            robot_0_x = (robot_0_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_0_y = (robot_0_y - self.grid_map_origin_y) / self.grid_map_resolution
            robot_1_x = (robot_1_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_1_y = (robot_1_y - self.grid_map_origin_y) / self.grid_map_resolution
                        
            # plot the arrays side by side
            # Note, pcolormesh automatically flips vertical axis when plotting. i.e. array[0,0] is
            # plotted in the lower left hand corner but top left corner when print(array).
            # Also first transpose the array (i.e. mirror along diagonal) so that combination of 
            # mirror along diagonal and vertical flip alligns the x axis to the right and y axis to
            # the top. This is needed so that robots are in correct position in map when plotted.
            fig, (ax0, ax1, ax2) = plt.subplots(1,3)
            

            #print(believe_array[:,1:19])
            test_list =[]
            for i in range(20):
                for j in range(20):
                    if believe_array[i,j] == -1:
                        test_list.append(100)
                    else:
                        test_list.append(0)

            
            # plotting
            im0 = ax0.pcolormesh(task_distr_array)
            ax0.set_title('Task Distribution')
            ax0.set_xlabel('min={}, sec = {}'.format(closest_time_td.minute, closest_time_td.second))
            fig.colorbar(im0, ax=ax0)
            
            im1 =  ax1.pcolormesh(uncertainty_array)
            #im1 =  ax1.pcolormesh(believe_array)
            ax1.set_title('Uncertainty Distributon, robot time = min={}, sec = {}'.format(time.minute, time.second))
            fig.colorbar(im1, ax=ax1)
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax1.scatter(robot_0_x, robot_0_y, c='b')
                ax1.scatter(robot_1_x, robot_1_y, c='r')
            ax1.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))
    

            im2 = ax2.pcolormesh(expected_array)
            ax2.set_title('Expected Distribution')
            
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax2.scatter(robot_0_x, robot_0_y, c='b')
                ax2.scatter(robot_1_x, robot_1_y, c='r')
            fig.colorbar(im2, ax=ax2)
            ax2.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))

            plt.savefig('plots/opinion_map_' + self.filename + '_op_{}.png'.format(self.fig_counter), bbox_inches='tight')
            plt.close(fig)
            #plt.show()
            self.fig_counter += 1
            

    def plot_belief_maps(self):
        """
        This function plots the individual opinions published in the observations data and their
        aggregation.
        """
      
        # alternatively use robot positioning as timing
        if False:
            #  use robots as time
            time_list = sorted(self.robot_pos['robot_0'].keys())
        else:
            # use opinion map as time
            time_list = sorted(self.observations_map.keys())
 
        for time in time_list:
            task_distr_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            
            # determine the times closest to 'time' in both arrays.
            # find closest time that is older i.e. smaller thatn time. (so dont see future)
            
            arr_td = np.array([(i - time).total_seconds() for i in self.task_distr.keys()])
            if np.all(arr_td >= 0):
                # take closest one
                closest_time_td = min(self.task_distr.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_td_index =np.where(arr_td== -1*min(np.absolute(arr_td[arr_td<0])))[0][0]
                closest_time_td = self.task_distr.keys()[closest_time_td_index]
            
            arr_op = np.array([(i - time).total_seconds() for i in self.opinion_map.keys()])
            if np.all(arr_op >= 0):
                # take closest one
                closest_time_op = min(self.opinion_map.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_op_index = np.where(arr_op==-1*min(np.absolute(arr_op[arr_op<0])))[0][0]
                closest_time_op = self.opinion_map.keys()[closest_time_op_index]
            


            arr_obs = np.array([(i - time).total_seconds() for i in self.observations_map.keys()])
            if np.all(arr_obs >= 0):
                # take closest one
                closest_time_obs = min(self.observations_map.keys(), key=lambda x: np.absolute(x - time))
            else:
                closest_time_obs_index = np.where(arr_obs==-1*min(np.absolute(arr_obs[arr_obs<0])))[0][0]
                closest_time_obs = self.observations_map.keys()[closest_time_obs_index]



            # convert data to 2D array. The values are assigned as follows:
            # -1: unoccupied space
            #  0: false positive of robot 0
            #  1: fasle positive of robobt 1
            #  2: false positive of robot 0 and 1
            #  3: ground truth
            for pos_index in self.task_distr[closest_time_td].keys():
                fp_list = self.task_distr[closest_time_td][pos_index]
                if len(fp_list) == 0:
                    task_distr_array[0,pos_index] = 3
                elif len(fp_list) == 1:
                    task_distr_array[0, pos_index] = fp_list[0]
                elif len(fp_list) == 2:
                    task_distr_array[0, pos_index] = 2

            # believe array stores the belive that it is occupied
            # whilst uncertainty array stores the uncertainty
            believe_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            uncertainty_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            expected_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            base_rate_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            for pos_index in self.opinion_map[closest_time_op].keys():
                tmp_op = self.opinion_map[closest_time_op][pos_index]
                believe_array[0, pos_index] = tmp_op.belief
                uncertainty_array[0, pos_index] = tmp_op.uncertainty 
                base_rate_array[0, pos_index] = tmp_op.base_rate
                expected_array[0, pos_index] = tmp_op.belief + tmp_op.base_rate * tmp_op.uncertainty
                
            
            net_belief_array = -1 * np.ones([1, self.grid_map_height * self.grid_map_width])
            for pos_index in self.observations_map[closest_time_obs].keys():
                net_belief_array[0,pos_index] = self.observations_map[closest_time_obs][pos_index]
            
            
            # add walls to the task_distr for reference
            wall_mask = [believe_array[0,:] == -1]
            task_distr_array[0,wall_mask[0]] = -2
               
            # reshape arrays into actual dimensions
            task_distr_array = np.reshape(task_distr_array, [self.grid_map_height, self.grid_map_width])
            believe_array = np.reshape(believe_array, [self.grid_map_height, self.grid_map_width])
            uncertainty_array = np.reshape(uncertainty_array, [self.grid_map_height, self.grid_map_width])
            expected_array = np.reshape(expected_array, [self.grid_map_height, self.grid_map_width])
            base_rate_array = np.reshape(base_rate_array, [self.grid_map_height, self.grid_map_width])
           

            net_belief_array = np.reshape(net_belief_array, [self.grid_map_height, self.grid_map_width])

            # Determine the robot positions at this time
            closest_time_r_0 = min(self.robot_pos['robot_0'], key=lambda x: np.absolute(x-time))
            closest_time_r_1 = min(self.robot_pos['robot_1'], key=lambda x: np.absolute(x-time))
            robot_0_x, robot_0_y = self.robot_pos['robot_0'][closest_time_r_0]
            robot_1_x, robot_1_y = self.robot_pos['robot_1'][closest_time_r_1]
            
            # convert the robot pos into the correct indexes
            robot_0_x = (robot_0_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_0_y = (robot_0_y - self.grid_map_origin_y) / self.grid_map_resolution
            robot_1_x = (robot_1_x - self.grid_map_origin_x) / self.grid_map_resolution
            robot_1_y = (robot_1_y - self.grid_map_origin_y) / self.grid_map_resolution
            
            # plot the arrays side by side
            # Note, pcolormesh automatically flips vertical axis when plotting. i.e. array[0,0] is
            # plotted in the lower left hand corner but top left corner when print(array).
            # Also first transpose the array (i.e. mirror along diagonal) so that combination of 
            # mirror along diagonal and vertical flip alligns the x axis to the right and y axis to
            # the top. This is needed so that robots are in correct position in map when plotted.
            fig, (ax0, ax1, ax2, ax3) = plt.subplots(1,4)            

            im0 = ax0.pcolormesh(task_distr_array)
            ax0.set_title('Task Distribution')
            ax0.set_xlabel('min={}, sec = {}'.format(closest_time_td.minute, closest_time_td.second))
            fig.colorbar(im0, ax=ax0)
            
            im1 =  ax1.pcolormesh(net_belief_array)
            ax1.set_title('Net Belief Distr, robot time = min={}, sec = {}'.format(time.minute, time.second))
            fig.colorbar(im1, ax=ax1)
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax1.scatter(robot_0_x, robot_0_y, c='b')
                ax1.scatter(robot_1_x, robot_1_y, c='r')
            ax1.set_xlabel('min={}, sec = {}'.format(closest_time_obs.minute, closest_time_obs.second))

            im2 =  ax2.pcolormesh(believe_array)
            ax2.set_title('Believe Distributon, robot time = min={}, sec = {}'.format(time.minute, time.second))
            fig.colorbar(im2, ax=ax2)
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax2.scatter(robot_0_x, robot_0_y, c='b')
                ax2.scatter(robot_1_x, robot_1_y, c='r')
            ax2.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))
    

            im3 = ax3.pcolormesh(expected_array)
            ax3.set_title('Expected Distribution')
            # add the robot positions to this plot
            if len(self.robot_pos.keys()) > 0:
                ax3.scatter(robot_0_x, robot_0_y, c='b')
                ax3.scatter(robot_1_x, robot_1_y, c='r')
            fig.colorbar(im3, ax=ax3)
            ax3.set_xlabel('min={}, sec = {}'.format(closest_time_op.minute, closest_time_op.second))

            plt.savefig('plots/opinion_map_' + self.filename + '_op_{}.png'.format(self.fig_counter), bbox_inches='tight')
            plt.close(fig)
            #plt.show()
            
            self.fig_counter += 1




'''
    Executing Analysis
'''


if __name__ == '__main__':
    
    bag_path = "./data"
    
    # Flag, deciding which test / plots to perform
    # The options are: 'faulty_robot', 'threshold', 'operators', 'fn_times', 'fp_times'
    # which are the thesis plots. Additional ones are 'single', 'multiple'. These plot motion of 
    # robot though the map at small time increments and the task distribution / expected value. One 
    # should probably use the single analysis for this purpose as 'multiple' might be a bit out-dated
    # as it contains some of the initial attempts to compare different tests with each other.

    analysis_type = 'fn_times' 
            

    if analysis_type == 'single':
        # single analysis, ploting opinion maps of a single test (these are many detailed plots)..

        # note filename without .bag ending
        filename = "r2_sl1_fp1_fn1_spi20_testing"
        
        # extract data
        data_analysis = DataAnalysis(bag_path, filename)
        data_analysis.extract_active_tasks()
        data_analysis.extract_opinion_map()
        #data_analysis.extract_observation_info()
        data_analysis.extract_robot_pos()
        
        # Plotting time history of completing ground truths
        data_analysis.completion_time_history(True)
        
        # Plotting the aggregated opinoin maps
        data_analysis.plot_opinion_maps()
        
        # Plotting the individual opinions that creat aggregated opinion map (many plots!)
        data_analysis.plot_opinion_maps()

    # compare the CBF, CCF and Combination operators to each other
    elif analysis_type == 'operators' or analysis_type == 'all':
        spi = 60 # 15 or 60 possible

        # get all filenames that were used for the test
        filename_list = []
        seed_list = [71,72,73,74,75]
        operator_list = ['comb','cbf','ccf']
        seed_order = []
        operator_order = []
        for op_type in operator_list:
            for tmp_seed in seed_list:
                if op_type == 'comb':
                    name = 'r2_sltrue_fpfalse_fnfalse_spi{}_seed{}_r0fpp0.2_r0fnp0.2_r1fpp0.2_r1fnp0.2'.format(spi,tmp_seed)
                else:
                    name = 'r2_sltrue_fpfalse_fnfalse_spi{}_seed{}_r0fpp0.2_r0fnp0.2_r1fpp0.2_r1fnp0.2_'.format(spi,tmp_seed) + op_type
                filename_list.append(name)
                operator_order.append(op_type)
                seed_order.append(tmp_seed)

        
        # store dictinary of format dict[operator][seed] = [avg_time, stdev] in dict list and 
        # the seed with which the test was done in the seed list.
        history_dict = defaultdict(dict)
        
        for (filename, tmp_op, tmp_seed) in zip(filename_list, operator_order, seed_order):
            data_analysis = DataAnalysis(bag_path, filename)
            data_analysis.extract_active_tasks()

            history_results = data_analysis.completion_time_history(False)
            history_dict[tmp_op][tmp_seed] = history_results

        # plotting
        fig = plt.figure(1)
        marker_list = [".","+", "*", "1", "x", "x","D"]
        m_size = 50
        comb_handles = []
        cbf_handles = []
        ccf_handles = []
        for op_type in history_dict.keys():
            counter = 0
            for tmp_seed in history_dict[op_type].keys():
                tp_times = history_dict[op_type][tmp_seed][0]
                
                if op_type == 'comb':
                    comb_h = plt.scatter(tp_times, range(1,len(tp_times) + 1, 1), color="k",marker=marker_list[counter], s=m_size)
                    comb_handles.append(comb_h)
                elif op_type == 'cbf':
                    cbf_h = plt.scatter(tp_times, range(1,len(tp_times) + 1, 1), color="blue",marker=marker_list[counter], s = m_size)
                    cbf_handles.append(cbf_h)
                elif op_type == 'ccf':
                    ccf_h = plt.scatter(tp_times, range(1,len(tp_times) + 1, 1), color="orange",marker=marker_list[counter], s=m_size)
                    ccf_handles.append(ccf_h)
                counter += 1
         
        plt.xlim([0,45*60])
        plt.xlabel('Time (s)',fontsize=25)
        plt.ylabel('Completed Tasks',fontsize=25)
        plt.legend([tuple(comb_handles),tuple(cbf_handles), tuple(ccf_handles)], ['Comb.', 'CBF', 'CCF'], numpoints=1, handler_map={tuple: HandlerTuple(ndivide=None)},fontsize=25)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        plt.savefig('plots/operators_comp_spi{}.png'.format(spi), bbox_inches='tight')

    # Determine the impact of false positives on the average time to complete tasks
    elif analysis_type == 'fp_times' or analysis_type == 'all':

        filename_list = []
        seed_list = [71,72,73,74,75]
        fp_list = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
        ka_list = ['true','false']
        seed_order = []
        fp_order = []
        ka_order = []
        for tmp_ka in ka_list:
            for tmp_fp in fp_list:
                for tmp_seed in seed_list:
                    name = 'r2_sl'+ tmp_ka + '_fptrue_fntrue_spi60_seed{}_r0fpp{}_r0fnp0.2_r1fpp{}_r1fnp0.2'.format(tmp_seed, tmp_fp, tmp_fp)
                    filename_list.append(name)
                    fp_order.append(tmp_fp)
                    seed_order.append(tmp_seed)
                    ka_order.append(tmp_ka)

        
        # store dictinary of format dict[KA][fp] = [avg_time, stdev] in dict list and 
        # the seed with which the test was done in the seed list.
        avg_time_dict = defaultdict(dict)
        
        for (filename, tmp_ka, tmp_fp) in zip(filename_list, ka_order, fp_order):
            data_analysis = DataAnalysis(bag_path, filename)
            data_analysis.extract_active_tasks()
            
            duration_results = data_analysis.ground_truth_completion_time() 
            

            if np.isnan(duration_results[0]) == False:
                # exxtract the average time to complete task
                if tmp_ka in avg_time_dict.keys() and tmp_fp in avg_time_dict[tmp_ka].keys():
                    prev_list = avg_time_dict[tmp_ka][tmp_fp]
                    prev_list.append(duration_results[0])
                    avg_time_dict[tmp_ka][tmp_fp] = prev_list
                else:
                    avg_time_dict[tmp_ka][tmp_fp] = [duration_results[0]]

        # Box plot
        data_ka = []
        data_noka = []
        for tmp_fp in fp_list:
            data_ka.append(avg_time_dict['true'][tmp_fp])
            data_noka.append(avg_time_dict['false'][tmp_fp])
        
        fig1, ax1 = plt.subplots()
        boxes_ka = plt.boxplot(data_ka, positions=np.arange(1,2 * len(data_ka)+1,2), patch_artist=True)
        boxes_noka = plt.boxplot(data_noka, positions=np.arange(2,2*len(data_ka)+2,2), patch_artist=True)
        c = "blue"
        c2 = "black"
        c3 = 'orange'
        for element in ['boxes','whiskers','fliers','caps','medians']:
            counter = 0
            for box in boxes_ka[element]:
                box.set(color=c)
                if element == 'boxes':
                    box.set(facecolor=c)
                elif element == 'fliers':
                    box.set(markeredgecolor=c)
                elif element == 'medians':
                    box.set(color=c3, linewidth=2)
            for box in boxes_noka[element]:
                box.set(color=c2)
                if element == 'boxes':
                    box.set(facecolor=c2)
                elif element == 'fliers':
                    box.set(markeredgecolor=c2)
                elif element == 'medians':
                    box.set(color=c3, linewidth=2)
                
        plt.xlim([0,19]) 
        plt.xlabel('False Positive Probability',fontsize=25)
        plt.ylabel('Avg. TP Completion Time (s)',fontsize=25)
        plt.legend([boxes_ka["boxes"][0], boxes_noka["boxes"][0]],['KA', 'No KA'],fontsize=25)
        plt.xticks([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18],[0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9], fontsize=20)
        plt.yticks(fontsize=20)
        plt.savefig('plots/box_plots_fp.png', bbox_inches='tight')


    # Determine the impact of false positives on the average time to complete tasks
    elif analysis_type == 'fn_times' or analysis_type == 'all':

        filename_list = []
        seed_list = [71,72,73,74,75]
        fn_list = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
        ka_list = ['true','false']
        fn_order = []
        ka_order = []
        for tmp_ka in ka_list:
            for tmp_fn in fn_list:
                for tmp_seed in seed_list:
                    name = 'r2_sl'+ tmp_ka + '_fptrue_fntrue_spi60_seed{}_r0fpp0.2_r0fnp{}_r1fpp0.2_r1fnp{}'.format(tmp_seed, tmp_fn, tmp_fn)
                    filename_list.append(name)
                    fn_order.append(tmp_fn)
                    ka_order.append(tmp_ka)

        # store dictinary of format dict[KA][fn] = [avg_time, stdev] in dict list and 
        # the seed with which the test was done in the seed list.
        avg_time_dict = defaultdict(dict)
        
        for (filename, tmp_ka, tmp_fn) in zip(filename_list, ka_order, fn_order):
            data_analysis = DataAnalysis(bag_path, filename)
            data_analysis.extract_active_tasks()
            
            duration_results = data_analysis.ground_truth_completion_time() 

            if np.isnan(duration_results[0]) == False:
                # extract the average time to complete task
                if tmp_ka in avg_time_dict.keys() and tmp_fn in avg_time_dict[tmp_ka].keys():
                    prev_list = avg_time_dict[tmp_ka][tmp_fn]
                    prev_list.append(duration_results[0])
                    avg_time_dict[tmp_ka][tmp_fn] = prev_list
                else:
                    avg_time_dict[tmp_ka][tmp_fn] = [duration_results[0]]

        # Box plot
        data_ka = []
        data_noka = []
        for tmp_fn in fn_list:
            data_ka.append(avg_time_dict['true'][tmp_fn])
            data_noka.append(avg_time_dict['false'][tmp_fn])
        
        fig1, ax1 = plt.subplots()
        boxes_ka = plt.boxplot(data_ka, positions=np.arange(1,2 * len(data_ka)+1,2), patch_artist=True)
        boxes_noka = plt.boxplot(data_noka, positions=np.arange(2,2*len(data_ka)+2,2), patch_artist=True)
        c = "blue"
        c2 = "black"
        c3 = "orange"
        for element in ['boxes','whiskers','fliers','caps','medians']:
            counter = 0
            for box in boxes_ka[element]:
                box.set(color=c)
                if element == 'boxes':
                    box.set(facecolor=c)
                elif element == 'fliers':
                    box.set(markeredgecolor=c)
                elif element == 'medians':
                    box.set(color=c3, linewidth=2)
            for box in boxes_noka[element]:
                box.set(color=c2)
                if element == 'boxes':
                    box.set(facecolor=c2)
                elif element == 'fliers':
                    box.set(markeredgecolor=c2)
                elif element == 'medians':
                    box.set(color=c3, linewidth=2)
                
        plt.xlim([0,19]) 
        plt.xlabel('False Negative Probability',fontsize=25)
        plt.ylabel('Avg. TP Completion Time (s)',fontsize=25)
        plt.legend([boxes_ka["boxes"][0], boxes_noka["boxes"][0]],['KA', 'No KA'],fontsize=25)
        plt.xticks([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18],[0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9], fontsize=20)
        plt.yticks(fontsize=20)
        plt.savefig('plots/box_plots_fn.png', bbox_inches='tight')


    # Determine the impact of false positives on the average time to complete tasks
    elif analysis_type == 'threshold' or analysis_type == 'all':

        filename_list = []
        seed_list = [71,72,73,74,75]
        fp_list = [0.2, 0.5, 0.8]
        threshold_list = [20, 40, 60, 80]
        fp_order = []
        thresh_order = []
        seed_order = []
        for tmp_fp in fp_list:
            for tmp_thresh in threshold_list:
                for tmp_seed in seed_list:
                    if tmp_thresh!=40 or tmp_fp!=0.8 or tmp_seed!=71:
                    #if tmp_thresh!=40 or tmp_fp!=0.5 or tmp_seed!=72:
                        name = 'r2_sltrue_fptrue_fntrue_spi60_seed{}_r0fpp{}_r0fnp0.2_r1fpp{}_r1fnp0.2_thresh{}'.format(tmp_seed, tmp_fp, tmp_fp, tmp_thresh) # SPI 60
                        filename_list.append(name)
                        fp_order.append(tmp_fp)
                        thresh_order.append(tmp_thresh)
                        seed_order.append(tmp_seed)

        # store dictinary of format dict[fp][thresh] = [[list of TP completed],[list of FP completed]] in dict list and 
        task_completed = defaultdict(dict)

        for (filename, tmp_thresh, tmp_fp, tmp_seed) in zip(filename_list, thresh_order, fp_order, seed_order):
            data_analysis = DataAnalysis(bag_path, filename)
            data_analysis.extract_active_tasks()
            
            counter_results = data_analysis.task_type_counter()
            
            if tmp_fp in task_completed.keys() and tmp_thresh in task_completed[tmp_fp].keys():
                completed_list = task_completed[tmp_fp][tmp_thresh]
                completed_list[0].append(counter_results[0]) # num TP completed
                completed_list[1].append(counter_results[2]) # num FP completed
                task_completed[tmp_fp][tmp_thresh] = completed_list
            else:
                task_completed[tmp_fp][tmp_thresh] = [[counter_results[0]],[counter_results[2]]]

        # plotting
        fig_num = 1
        for tmp_fp in fp_list:
            tp_mean = []
            fp_mean = []
            thresholds = []
            print('FPP = ', tmp_fp)
            for tmp_thresh in sorted(task_completed[tmp_fp].keys()):
                tp_mean.append(np.average(task_completed[tmp_fp][tmp_thresh][0]))
                fp_mean.append(np.average(task_completed[tmp_fp][tmp_thresh][1]))
                
                # normalize to 1
                total = tp_mean[-1] + fp_mean[-1]
                tp_mean[-1] = tp_mean[-1]/total
                fp_mean[-1] = fp_mean[-1]/total
                 
                print('THRESH = {}, Avg. completed tasks TP= {}, FP = {}, total= {}'.format(tmp_thresh, tp_mean[-1], fp_mean[-1], total))

                all_list= task_completed[tmp_fp][tmp_thresh]
                tp_list = all_list[0]
                fp_list = all_list[1]

                thresholds.append(tmp_thresh/100.)

            x_pos = [i for i,_ in enumerate(thresholds)]

            fig = plt.figure(fig_num)
            plt.bar(x_pos, tp_mean, label='True Positives', color='blue')
            plt.bar(x_pos, fp_mean, label='False Positives', color='red', bottom=tp_mean)
            plt.legend(fontsize=17)
            plt.xlabel('Threshold',fontsize=17)
            plt.ylabel('Completed Tasks',fontsize=17)
            plt.xticks(x_pos, thresholds, fontsize=15)
            plt.yticks(fontsize=15)
            plt.savefig('plots/threshold_fp{}.png'.format(tmp_fp), bbox_inches='tight')

            fig_num += 1

        
    # One faulty robot
    elif analysis_type == 'faulty_robot' or analysis_type == 'all':

        filename_list = []
        seed_list = [71,72,73,74,75]
        fp_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 0.9 bad results
        nr_list = [1,2,5] # number of robots
        fp_order = []
        nr_order = []
        seed_order = []
        for tmp_fp in fp_list:
            for tmp_nr in nr_list:
                for tmp_seed in seed_list:
                    if tmp_nr == 1:
                        name = 'r1_slfalse_fptrue_fntrue_spi60_seed{}_r0fpp{}_r0fnp0.2_thresh80'.format(tmp_seed, tmp_fp)
                    elif tmp_nr == 2:
                        name = 'r2_sltrue_fptrue_fntrue_spi60_seed{}_r0fpp{}_r0fnp0.2_r1fpp0.1_r1fnp0.2'.format(tmp_seed, tmp_fp)
                    elif tmp_nr == 5:
                        name = 'r5_sltrue_fptrue_fntrue_spi60_seed{}_r0fpp{}_r0fnp0.2_r1fpp0.1_r1fnp0.2_r2fpp0.1_r2fnp0.2_r3fpp0.1_r3fnp0.2_r4fpp0.1_r4fnp0.2_thresh80'.format(tmp_seed, tmp_fp)

                    filename_list.append(name)
                    fp_order.append(tmp_fp)
                    nr_order.append(tmp_nr)
                    seed_order.append(tmp_seed)

        # store dictinary of format dict[fp prob][num_robots] = [[list of TP completed (for seeds)],
        # [list of FP completed]] in dict list
        task_completed = defaultdict(dict)

        for (filename, tmp_nr, tmp_fp, tmp_seed) in zip(filename_list, nr_order, fp_order, seed_order):
            data_analysis = DataAnalysis(bag_path, filename)
            data_analysis.extract_active_tasks()
            
            counter_results = data_analysis.task_type_counter_r0()
            
            if tmp_fp in task_completed.keys() and tmp_nr in task_completed[tmp_fp].keys():
                completed_list = task_completed[tmp_fp][tmp_nr]
                completed_list[0].append(counter_results[0]) # num TP completed
                completed_list[1].append(counter_results[1]) # num FP completed
                task_completed[tmp_fp][tmp_nr] = completed_list
            else:
                task_completed[tmp_fp][tmp_nr] = [[counter_results[0]],[counter_results[1]]]

        # plotting bar charts
        fig_num = 1
        for tmp_fp in fp_list:
            tp_mean = []
            fp_mean = []
            num_robots = []

            for tmp_nr in sorted(task_completed[tmp_fp].keys()):
                tp_mean.append(np.average(task_completed[tmp_fp][tmp_nr][0]))
                fp_mean.append(np.average(task_completed[tmp_fp][tmp_nr][1]))
                
                # normalize to 1
                total = tp_mean[-1] + fp_mean[-1]
                tp_mean[-1] = tp_mean[-1]/total
                fp_mean[-1] = fp_mean[-1]/total
                
                num_robots.append(tmp_nr)
            

            x_pos = [i for i,_ in enumerate(num_robots)]

            fig = plt.figure(fig_num)
            plt.bar(x_pos, tp_mean, label='True Positives', color='blue')
            plt.bar(x_pos, fp_mean, label='False Positives', color='orange', bottom=tp_mean)
            plt.legend(fontsize=25)
            plt.xlabel('Number of Robots',fontsize=25)
            plt.ylabel('Fraction of Tasks Completed by $R_1$',fontsize=25)
            plt.xticks(x_pos, num_robots, fontsize=20)
            plt.yticks(fontsize=20)
            plt.savefig('plots/one_faulty_robot_fp{}.png'.format(tmp_fp), bbox_inches='tight')

            fig_num += 1

        # plotting gains
        gain_2r = []
        gain_5r = []

        for tmp_fp in fp_list:
            tp_mean = []
            #print(tmp_fp)
            #print(task_completed[tmp_fp])
            for tmp_nr in sorted(task_completed[tmp_fp].keys()):
                tp_mean.append(np.average(task_completed[tmp_fp][tmp_nr][0]))
                fp_mean.append(np.average(task_completed[tmp_fp][tmp_nr][1]))
                
                # normalize to 1
                total = tp_mean[-1] + fp_mean[-1]
                tp_mean[-1] = tp_mean[-1]/total

            # gain(r) = tp(r) - tp(r1)
            gain_2r.append(tp_mean[1] - tp_mean[0])
            gain_5r.append(tp_mean[2] - tp_mean[0])

        
        fig = plt.figure(fig_num)
        plt.plot(fp_list, gain_2r, label='$n_R = 2$', color='orange')
        plt.plot(fp_list, gain_5r, label='$n_R = 5$', color='blue')
        plt.legend(fontsize=25)
        plt.xlabel('False Positive Probability',fontsize=25)
        plt.ylabel('Gain($R_1$, $n_R$)',fontsize=25)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        plt.savefig('plots/one_faulty_robot_gain.png', bbox_inches='tight')
        fig_num += 1

        # plotting bar charts as plots

        tp_2r = []
        tp_5r = []
        tp_1r = []

        for tmp_fp in fp_list:
            tp_mean = []
            #print(tmp_fp)
            #print(task_completed[tmp_fp])
            for tmp_nr in sorted(task_completed[tmp_fp].keys()):
                tp_mean.append(np.average(task_completed[tmp_fp][tmp_nr][0]))
                fp_mean.append(np.average(task_completed[tmp_fp][tmp_nr][1]))
                
                # normalize to 1
                total = tp_mean[-1] + fp_mean[-1]
                tp_mean[-1] = tp_mean[-1]/total

            tp_1r.append(tp_mean[0])
            tp_2r.append(tp_mean[1])
            tp_5r.append(tp_mean[2])
        
        fig = plt.figure(fig_num)
        plt.plot(fp_list, tp_1r, label='1 Robot', color='black')
        plt.plot(fp_list, tp_2r, label='2 Robots', color='orange')
        plt.plot(fp_list, tp_5r, label='5 Robots', color='blue')
        plt.legend(fontsize=17)
        plt.xlabel('False Positive Probability',fontsize=17)
        plt.ylabel('TP Fraction of Tasks Completed by $R_1$',fontsize=17)
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        plt.savefig('plots/one_faulty_robot_distr.png', bbox_inches='tight')

    elif analysis_type == 'multiple':
        # either give specific filenames here
        #filename_list = ["r2_sl1_fp1_fn1_spi80_comp", ...] # without .bag ending
        
        # or all in /data directory
        filename_list= []
        dirListing = os.listdir('./data')
        for fname in dirListing:
            # remove .bag ending
            filename_list.append(fname[:fname.rindex('.bag')])
        
        # store dictinary of format dict[interval][use_sl] = [avg_time, stdev] in dict list and 
        # the seed with which the test was done in the seed list.
        seed_list = [] 
        dict_list = []
        counter_dict_list = []
        history_dict_list = []

        for filename in filename_list:
            # get test info from name
            seperated_str= re.split('_+', filename)
            use_sl_str = re.split('sl', seperated_str[1])[1]
            spawn_interval = int(re.findall('\d+', seperated_str[4])[0])# _spi# is 5th entry of name 
            
            print(seperated_str)

            seed = int(re.findall('\d+', seperated_str[5])[0]) # _seed# is 6th entry of name 
            fpp0_num = float(re.findall('\d+\.\d+', seperated_str[6])[0])
            fpp1_num = float(re.findall('\d+\.\d+', seperated_str[8])[0])

            # only select the ones with specific fp uncertainties:
            if fpp0_num == 0.4 and fpp1_num == 0.4 and seperated_str[-1] != 'type0.99':
                
                # extract information
                data_analysis = DataAnalysis(bag_path, filename)
                data_analysis.extract_active_tasks()
                #data_analysis.extract_opinion_map()
                #data_analysis.extract_observation_info()
                #data_analysis.extract_robot_pos()
                
                # saving data in pickle file for offline CBF calculations
                #data_analysis.save_data()
                
                # analysis
                print('File tested: SL = ' + use_sl_str + ' seed = {} interval = {}'.format(seed, spawn_interval))
                duration_results = data_analysis.ground_truth_completion_time()
                counter_results = data_analysis.task_type_counter()
                history_results = data_analysis.completion_time_history(False)

                # store duration_results: first check if already dictionary with seed 
                # if exists then add to that.
                if seed in seed_list:
                    index = seed_list.index(seed)
                    
                    existing_dict = dict_list[index]
                    existing_counter_dict = counter_dict_list[index]
                    existing_history_dict = history_dict_list[index]   
                    
                    # check if keys exist
                    if spawn_interval in existing_dict.keys() and use_sl_str in existing_dict[spawn_interval].keys():
                        print('Keys already exist, should not happen, replace previous value with new one')

                    # replace previous value -- maybe should take average instead...
                    existing_dict[spawn_interval][use_sl_str] = duration_results
                    dict_list[index] = existing_dict

                    existing_counter_dict[spawn_interval][use_sl_str] = counter_results
                    counter_dict_list[index] = existing_counter_dict
                    
                    existing_history_dict[spawn_interval][use_sl_str] = history_results
                    history_dict_list[index] = existing_history_dict

                else:
                    seed_list.append(seed)    
                    
                    seed_dict = defaultdict(dict)
                    seed_dict[spawn_interval][use_sl_str] = duration_results
                    dict_list.append(seed_dict)

                    counter_dict = defaultdict(dict)
                    counter_dict[spawn_interval][use_sl_str] = counter_results
                    counter_dict_list.append(counter_dict)

                    history_dict = defaultdict(dict)
                    history_dict[spawn_interval][use_sl_str] = history_results
                    history_dict_list.append(history_dict)
                # duration_results = [avg time, stdev]

                # Printing tasks who are a false positive for both robots
                #for key in data_analysis.task_data.keys():
                #    if len(data_analysis.task_data[key]['false_positive']) == 2:
                #        print(data_analysis.data[key])
                
                
                # Detailed Plotting: belief maps plots aggregation of individual opinions (debugging)
                #                    opinons maps plots belief and expected distribution (prefered)
                #data_analysis.plot_belief_maps()
                #data_analysis.plot_opinion_maps()

        # plotting avg completion times and their stdev. against spi
        fig_counter = 1
        for i in range(len(seed_list)):
            # Plotting durations
            fig = plt.figure(fig_counter)
            seed = seed_list[i]
            seed_dict = dict_list[i]
            intervals = sorted(seed_dict.keys()) # from smallest fo largest
            print([seed_dict[j]['true'][0]  - seed_dict[j]['false'][0]for j in intervals])
            plt.scatter(intervals, [seed_dict[j]['true'][0] for j in intervals],label='avg time with KA')
            plt.scatter(intervals, [seed_dict[j]['false'][0] for j in intervals],label='avg time no KA')
            plt.scatter(intervals, [seed_dict[j]['true'][1] for j in intervals], label='stdev for KA')
            plt.scatter(intervals, [seed_dict[j]['false'][1] for j in intervals], label='stdev no KA')
            plt.legend()

            plt.savefig('plots/cleaning_times_seed{}_45min.png'.format(seed), bbox_inches='tight')
            fig_counter += 1

            # Plotting counters
            fig = plt.figure(fig_counter)
            counter_dict = counter_dict_list[i]
            print(counter_dict)
            plt.scatter(intervals, [counter_dict[j]['true'][0] for j in intervals],label='gt cleaned wth KA')
            plt.scatter(intervals, [counter_dict[j]['false'][0] for j in intervals],label='gt cleaned no KA')
            plt.scatter(intervals, [counter_dict[j]['true'][2] for j in intervals], label='fp cleaned with KA')
            plt.scatter(intervals, [counter_dict[j]['false'][2] for j in intervals], label='fp cleaned no KA')
            plt.legend()

            plt.savefig('plots/cleaning_numbers_seed{}_45min.png'.format(seed), bbox_inches='tight')
            fig_counter += 1

        # plotting the completion times
        for i in range(len(seed_list)):
            seed = seed_list[i]
            seed_dict = history_dict_list[i]
            counter_d = counter_dict_list[i]
            intervals = sorted(seed_dict.keys()) # from smallest fo largest
            for it in intervals:
                # GT time history
                fig = plt.figure(fig_counter)
                plt.plot(seed_dict[it]['true'][0], [j / float(counter_d[it]['true'][1]) for j in range(1, 1 + len(seed_dict[it]['true'][0]))], label='gt cleaned with KA')
                plt.plot(seed_dict[it]['false'][0], [j / float(counter_d[it]['false'][1]) for j in range(1, 1 + len(seed_dict[it]['false'][0]))] , label='gt cleaned no KA')
                plt.legend()
                plt.savefig('plots/gt_history_seed{}_interval{}_45min.png'.format(seed, it), bbox_inches='tight')

                fig_counter += 1
                
                # plotting fp time history
                fig = plt.figure(fig_counter)
                plt.plot(seed_dict[it]['true'][1], [j / float(counter_d[it]['true'][3]) for j in range(1, 1 + len(seed_dict[it]['true'][1]))], label='FP cleaned with KA')
                plt.plot(seed_dict[it]['false'][1],[j/ float(counter_d[it]['false'][3]) for j in range(1, 1 + len(seed_dict[it]['false'][1]))], label='FP cleaned no KA')
                plt.legend()
                plt.savefig('plots/fp_history_seed{}_interval{}_45min.png'.format(seed, it), bbox_inches='tight')

                fig_counter += 1

                # bar plot of completed gt vs fp in total (later make it per robot)
                fig, ax = plt.figure(fig.counter)
                label = ['KA', 'No KA']
                completions = [counter_d[it]['true'][0], counter_d_[it]['false'][2]]
                ax.bar(label, completions)
                plt.savefig('plots/bar_seed{}_interval{}_45min.png'.format(seed, it), bbox_inches='tight')







