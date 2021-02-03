# !/usr/bin/env python3.3 or higher
from matplotlib import pyplot as plt
import pickle
import numpy as np
import jpy
from collections import defaultdict

# File to perform SL calculation from stored opinions offline. This was used to verfiy that the calculation performed at run-time were correct. 

class so_analysis:
    def __init__(self, filename):
        # variables
        self.opinion_map_calc = defaultdict(dict)


        # initiate the subjective library -- change class path.
        self.classpath = '-Djava.class.path=/home/malte/catkin_ws/src/knowledge_aggregation/subjective_logic/build/libs/subjective-logic-java-library-0.1.0.jar'
        self.init_sl_library(self.classpath)

        # extract data from pickle file
        save_file = open('./data/' + filename + ".pkl", "rb")
        [self.observations_map, self.opinion_map] = pickle.load(save_file, encoding='latin1')

    def init_sl_library(self, classpath):
        jpy.create_jvm(['-Xmx512M', classpath])

        self.SubjectiveOpinion = jpy.get_type('de.tum.i4.subjectivelogic.SubjectiveOpinion')
        self.ArrayList = jpy.get_type('java.util.ArrayList')

    def sl_map(self):
        # initiate map of vacuous opinions
        vacuous_op = self.SubjectiveOpinion(0.0, 0.0, 1.0, 0.5)
        for i in range(400):
            self.opinion_map_calc[0][i] = vacuous_op

        #counter_total = 1
        #counter_belief = 1
        for msg_num in range(len(self.observations_map)):
            # copy previous values in new opinion map
            for tmp_ind in self.opinion_map_calc[msg_num].keys():
                self.opinion_map_calc[msg_num + 1][tmp_ind] = self.opinion_map_calc[msg_num][tmp_ind]

            so_collection = self.ArrayList()
            for so in self.observations_map[msg_num]:
                new_op_index = so[0]
                if new_op_index < 400:
                    new_op = self.SubjectiveOpinion(so[1], so[2], so[3], so[4]) 
                    old_op = self.opinion_map_calc[msg_num + 1][new_op_index]
                    
                    so_collection.add(old_op)
                    so_collection.add(new_op)
                
                    aggregated_op = self.SubjectiveOpinion.cumulativeCollectionFuse(so_collection)
                    
                    # limit the disbelief and uncertainty to be <= 99%
                    if (aggregated_op.getDisbelief() <= 0.99 and aggregated_op.getUncertainty() <= 0.99):
                        self.opinion_map_calc[msg_num + 1][new_op_index] = aggregated_op
                    
                else:
                    print(new_op_index)

                so_collection.clear()

    
    def comparison(self):
        """
        Compare the expected knowledge aggregation with what is actually happening
        """
        for msg_num in range(len(self.opinion_map_calc.keys())):
            abs_diff = 0.

            for index in range(len(self.opinion_map[msg_num])): 
                so_expected = self.opinion_map_calc[msg_num][index]
                so_measured = self.opinion_map[msg_num][index]
                
                expected_expectation = so_expected.getBelief() + (so_expected.getUncertainty() 
                                                              * so_expected.getBaseRate())

                measured_expectation = so_measured[1] + so_measured[3] * so_measured[4]
                
                if measured_expectation >= 0:
                    abs_diff += abs(expected_expectation - measured_expectation)
            print(abs_diff)



    def plotting(self):
        for msg_num in range(len(self.opinion_map_calc.keys())):
            
            measured_exp = -1 * np.ones(400)
            calc_exp = -1 * np.ones(400)
            for index in range(len(self.opinion_map[msg_num])): 
                so_expected = self.opinion_map_calc[msg_num][index]
                so_measured = self.opinion_map[msg_num][index]
                
                calc_exp[index] = so_expected.getBelief() + (so_expected.getUncertainty() 
                                                              * so_expected.getBaseRate())

                measured_exp[index] = so_measured[1] + so_measured[3] * so_measured[4]
                
            measured_exp = np.reshape(measured_exp, [20,20])
            calc_exp = np.reshape(calc_exp, [20,20])

            mask = (measured_exp != -1)
            print(np.sum(abs(calc_exp[mask] - measured_exp[mask])))

            fig, (ax0, ax1, ax2) = plt.subplots(1,3)
            im0 = ax0.pcolormesh(calc_exp)
            ax0.set_title('Calculated Expected Distribution')
            fig.colorbar(im0, ax=ax0)

            im1 = ax1.pcolormesh(measured_exp)
            ax1.set_title('Measured Expected Distribution')
            fig.colorbar(im1, ax=ax1)

            im2 = ax2.pcolormesh(calc_exp - measured_exp)
            ax2.set_title('Calc - Measured Distribution')
            fig.colorbar(im2, ax=ax2)
            
            plt.savefig('plots/comparison_{}.png'.format(msg_num))
            plt.close(fig)
            #fig_counter += 1

    
if __name__ == '__main__':
    filename = 'r2_sl1_fp1_fn0_test'

    so_analyse = so_analysis(filename)
    so_analyse.sl_map()
    so_analyse.plotting()
    #so_analyse.comparison()

