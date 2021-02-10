import jpy

# Create a new JVM with the SL library on its classpath
jpy.create_jvm(['-Xmx512M', '-Djava.class.path=/home/ubuntu/Desktop/Aggregatio/subjective_logic/build/libs/subjective-logic-java-library-0.1.0.jar'])

# Get a reference of the SubjectiveOpinion Java class through jpy bridge
SubjectiveOpinion = jpy.get_type('de.tum.i4.subjectivelogic.SubjectiveOpinion')

# Create two subjective opinions
so1 = SubjectiveOpinion(0.7, 0.00, 0.3, 0.50)
so2 = SubjectiveOpinion(0.55-1e-2, 0.45, 1e-2, 0.50)

# Get a reference to Java ArrayList through jpy bridge
ArrayList = jpy.get_type('java.util.ArrayList')
olist = ArrayList()

# Add the subjective opinions to the array list
olist.add(so1)
olist.add(so2)

print ("SO 1: " + so1.toString())
print ("SO 2: " + so2.toString())

# Perform Consensus&Compromise Fusion of the opinions on the list
ccf = SubjectiveOpinion.ccCollectionFuse(olist)
print ("CCF: " + ccf.toString())

# Perform Cumulative Fusion of the opinions in the list
cbf = SubjectiveOpinion.cumulativeCollectionFuse(olist)
print ("CBF: " + cbf.toString())

