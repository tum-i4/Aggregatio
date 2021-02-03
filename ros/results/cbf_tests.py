#/usr/bin/env python3.3 or higher
import jpy
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import rcParams
rcParams['mathtext.fontset'] = 'stix' #latex
rcParams['font.family'] = 'STIXGeneral'

# Create a new JVM with the SL library on its classpath -- change claspath!
jpy.create_jvm(['-Xmx512M', '-Djava.class.path=/home/ge73xus/catkin_ws/src/knowledge_aggregation/subjective_logic/build/libs/subjective-logic-java-library-0.1.0.jar'])

# Get a reference of the SubjectiveOpinion Java class through jpy bridge
SubjectiveOpinion = jpy.get_type('de.tum.i4.subjectivelogic.SubjectiveOpinion')
ArrayList = jpy.get_type('java.util.ArrayList')

######## Compare operators / combination for wave ############
def compare_operators(operator_type):
    vac_opinion = SubjectiveOpinion(0.0, 0.0, 1, 0.50)
    occ_opinion = SubjectiveOpinion(0.7, 0.0, 0.3, 0.50)
    empty_opinion = SubjectiveOpinion(0.0, 0.7, 0.3, 0.50)

    so_list = ArrayList()
    expectation_list = []
    uncertainty_list = []
   
    aggregated_op = vac_opinion
    uncertainty_list.append(aggregated_op.getUncertainty())
    expectation_list.append(aggregated_op.getBelief() + aggregated_op.getUncertainty() * aggregated_op.getBaseRate())

    for p in range(4):
        # first add number of occupied opinions then unoccpied
        for i in range(2):
            op = SubjectiveOpinion(0.0,0.0,1,0.5)
            if i == 0:
                op = occ_opinion
            else:
                op = empty_opinion

            for r in range(30):
                if operator_type == 'CBF':
                    so_list.add(aggregated_op)
                    so_list.add(op)

                    aggregated_op = SubjectiveOpinion.cumulativeCollectionFuse(so_list)
                    belief = aggregated_op.getBelief()
                    uncert = aggregated_op.getUncertainty()
                    br = aggregated_op.getBaseRate()
                    #print(belief, uncert, br)
                    
                    uncertainty_list.append(uncert)
                    expectation_list.append(belief + uncert * br)
                    so_list.clear()

                elif operator_type == 'combination':
                    so_list.add(aggregated_op)
                    so_list.add(op)

                    new_opinion_type = opinion_type(op)
                    old_opinion_type = opinion_type(aggregated_op)

                    # Option 1: Use CBF except when uncertainty < 1e-1 and conflicting then use CCF 
                    # to increase uncertainty.
                    if new_opinion_type != old_opinion_type and aggregated_op.getUncertainty() < 1e-1:
                        # Use CCF to increase uncertainty
                        aggregated_op = SubjectiveOpinion.ccCollectionFuse(so_list)
                        # Check that uncertainty of CCF not < 1e-11 (else CCF unstable, then use average)
                        if aggregated_op.getUncertainty() <= 1e-11:
                            aggregated_op = SubjectiveOpinion.average(so_list)
                    else:
                        # matching types and large enough uncertainty so use CBF
                        aggregated_op = SubjectiveOpinion.cumulativeCollectionFuse(so_list)
                    belief = aggregated_op.getBelief()
                    uncert = aggregated_op.getUncertainty()
                    br = aggregated_op.getBaseRate()

                    uncertainty_list.append(uncert)
                    expectation_list.append(belief + uncert * br)
                    so_list.clear()

    return expectation_list, uncertainty_list

def expectation(op):
    """
    calculates the expected value from an subjective opinion
    """
    belief = op.getBelief()
    uncert = op.getUncertainty()
    br = op.getBaseRate()

    return belief + uncert * br


def recovery(u_level, threshold=0.8):
    """
    For a given uncertainty level, how many opinions does it take to recover the max-opposite
    opinion
    """
    extreme_op = SubjectiveOpinion(0.0, 1 - u_level, u_level, 0.50)

    avg_u = 2 / (3 * 3.5)
    avg_occ_op = SubjectiveOpinion(1 - avg_u, 0.0, avg_u, 0.50)

    so_list = ArrayList()
    expectation_list = []

    aggregated_op = extreme_op
    num_aggregations = 0
    while(expectation(aggregated_op) < threshold):
        so_list.add(aggregated_op)
        so_list.add(avg_occ_op)
        aggregated_op = SubjectiveOpinion.cumulativeCollectionFuse(so_list)
        so_list.clear()

        num_aggregations +=1

    return num_aggregations

def opinion_type(subjective_opinion):
    """ 
    This function outputs 1 if belief >= disblief of subjective opinion and 0 otherwise
    """
    so_type = 0
    if subjective_opinion.getBelief() >= subjective_opinion.getDisbelief():
        so_type = 1
    return so_type

if __name__ == '__main__':

    if False:
        cbf_expct, cbf_unct = compare_operators('CBF')
        comb_expct, comb_unct = compare_operators('combination')

        num_opinions = range(1, len(cbf_expct) + 1)
        fig_num = 1
        fig = plt.figure(fig_num)
        plt.plot(num_opinions, cbf_expct, label='CBF Expectation')
        plt.plot(num_opinions, cbf_unct, label='CBF Uncertainty')
        #plt.plot(num_opinions, comb_expct, label='Comb Expectation')
        #plt.plot(num_opinions, comb_unct, label='Comb Uncertainty')
        plt.legend()
        plt.xlabel('Num of Opinions')
        plt.ylabel('Value')
        plt.savefig('plots/comparing_operators.png')
        fig_num += 1

        fig = plt.figure(fig_num)
        #plt.plot(num_opinions, cbf_expct, label='CBF Expectation')
        #plt.plot(num_opinions, cbf_unct, label='CBF Uncertainty')
        plt.plot(num_opinions, comb_expct, label='Comb Expectation')
        #plt.plot(num_opinions, comb_unct, label='Comb Uncertainty')
        plt.legend()
        plt.xlabel('Num of Opinions')
        plt.ylabel('Value')
        plt.savefig('plots/comparing_operators2.png')
        fig_num += 1

        fig = plt.figure(fig_num)
        #plt.plot(num_opinions, cbf_expct, label='CBF Expectation')
        #plt.plot(num_opinions, cbf_unct, label='CBF Uncertainty')
        #plt.plot(num_opinions, comb_expct, label='Comb Expectation')
        plt.plot(num_opinions, comb_unct, label='Comb Uncertainty')
        plt.legend()
        plt.xlabel('Num of Opinions')
        plt.ylabel('Value')
        plt.savefig('plots/comparing_operators3.png')
        fig_num += 1

        fig = plt.figure(fig_num)
        #plt.plot(num_opinions, cbf_expct, label='CBF Expectation')
        #plt.plot(num_opinions, cbf_unct, label='CBF Uncertainty')
        plt.plot(num_opinions, comb_expct, label='Comb Expectation')
        plt.plot(num_opinions, comb_unct, label='Comb Uncertainty')
        plt.legend()
        plt.xlabel('Num of Opinions')
        plt.ylabel('Value')
        plt.savefig('plots/comparing_operators4.png')
        fig_num += 1

        fig = plt.figure(fig_num)
        plt.plot(num_opinions, cbf_expct, label='CBF')
        #plt.plot(num_opinions, cbf_unct, label='CBF Uncertainty')
        plt.plot(num_opinions, comb_expct, label='Comb.')
        #plt.plot(num_opinions, comb_unct, label='Comb Uncertainty')
        plt.legend(fontsize=15,loc='upper right')
        plt.xlabel('Number of Aggregated Opinions',fontsize=15)
        plt.ylabel('$P(x)$',fontsize=15)
        plt.xticks(fontsize=13)
        plt.yticks(fontsize=13)
        plt.savefig('plots/comparing_operators5.png')
        fig_num += 1
    
    # Recovery test
    #n_3 = recovery(1e-3,0.8)
    #n_2 = recovery(1e-2,0.8)
    #n_1 = recovery(1e-1,0.8)
    #print(n_3, " ", n_2, " ", n_1)

    # SO Table in Background
    so_table = ArrayList()

    op_agree_1 = SubjectiveOpinion(0.85, 0.1, 0.05, 0.5)
    op_agree_2 = SubjectiveOpinion(0.52, 0.18, 0.3, 0.5)


    print("Agree 1: ", op_agree_1.toString())
    print("Agree 2: ", op_agree_2.toString())

    so_table.add(op_agree_1)
    so_table.add(op_agree_2)

    agg_op_cbf = SubjectiveOpinion.cumulativeCollectionFuse(so_table)
    agg_op_ccf = SubjectiveOpinion.ccCollectionFuse(so_table)
    agg_op_avg = SubjectiveOpinion.average(so_table)
    agg_op_wcf = SubjectiveOpinion.weightedCollectionFuse(so_table)
    
    print("CBF: ", agg_op_cbf.toString())
    print("CCF: ", agg_op_ccf.toString())
    print("AVG: ", agg_op_avg.toString())
    print("WCF: ", agg_op_wcf.toString())

    so_table.clear()

    op_disagree_1 = SubjectiveOpinion(0.85, 0.1, 0.05, 0.5)
    op_disagree_2 = SubjectiveOpinion(0.2, 0.64, 0.16, 0.5)


    print("Agree 1: ", op_disagree_1.toString())
    print("Agree 2: ", op_disagree_2.toString())

    so_table.add(op_disagree_1)
    so_table.add(op_disagree_2)

    agg_op_cbf = SubjectiveOpinion.cumulativeCollectionFuse(so_table)
    agg_op_ccf = SubjectiveOpinion.ccCollectionFuse(so_table)
    agg_op_avg = SubjectiveOpinion.average(so_table)
    agg_op_wcf = SubjectiveOpinion.weightedCollectionFuse(so_table)
    
    print("CBF: ", agg_op_cbf.toString())
    print("CCF: ", agg_op_ccf.toString())
    print("AVG: ", agg_op_avg.toString())
    print("WCF: ", agg_op_wcf.toString())



