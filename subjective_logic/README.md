# Subjective Logic Java library

This directory contains the source code implementing the Subjective Logic (SL) formalisms to perform Knowledge Aggregation in our ROS-based testbed. This source code is based on the one provided in https://github.com/vs-uulm/subjective-logic-java.

At the time the testbed was implemented, Java support for ROS Melodic was not available. Additionally, the nodes included in our testbed are implemented either in Python or C++. Thus, we needed a way to consume the subjective logic library API in our ROS nodes. To this end, we use jpy, a bi-directional Python-Java bridge that can be used to embed Java code in Python programs and the other way round. In our case, we only need the features that enable us to load Java classes, access Java class fields, and call class constructors and methods from our SL API.

In the next sections, we provide the basic instructions to install jpy and to build the SL library that we need in our ROS-based testbed.

## Installing jpy
To install jpy it is necessary to execute the full build process from sources. Since the testbed is intended to be used in Ubuntu, we provide the instructions to build and install jpy in Linux.

Before proceeding, make sure that your system has:
* A working Python 3.6 environment
* A Java SDK installed. Use Oracle JDK 11 or higher-you can follow the [Java installation](../subjective_logic/Installing Java.md) instructions for setting up Java in Linux.
* [Maven 3](maven.apache.org) or higher. Install maven with the command:
    ```
    sudo apt install maven
    ```
* gcc

### Building and installing jpy
1. Use git to clone the repository:
    ```
    git clone https://github.com/bcdev/jpy.git
    ```
    Alternatively, you can download the jpy sources from the [jpy releases page](https://github.com/bcdev/jpy/releases)

2. Specify the current JDK home with the following environment variables:
    ```
    export JDK_HOME=<path to the JDK installation directory>
    export JAVA_HOME=$JDK_HOME
    ```
    These variables are going to be used by Maven and Python in the next step.

3. Change into the checkout directory and compile the sources. Change the python command to the Python version with which you want jpy to be built. In our case that would be: 
    ```
    python3 setup.py build maven bdist_wheel
    ```
On success, you will find the binaries under the `build` directory and a Python wheel under `dist` directory.

4. Install the jpy wheel as any other Python package:
    ```
    pip3 install path/to/wheel
    ```
    By installing jpy using the wheel, jpy is installed system-wide and no further configuration is necessary.

5. Build the SL Library as described in the next section.

6. The jpy installation can be tested sing the file [`testjpy.py`](../subjective_logic/testjpy.py). Within this file one must change line 4 to the path of your SL Library (the.jar file that is located in the build/libs/ folder) and then one can test the jpy installation by executing:
    ```
    python3 ./testjpy.py 
    ```
   After executing the script, you should see the aggregated results of two subjective opinions, using Cumulative Belief Fusion and Consensus&Compromise Fusion.

7. To execute the actual ROS code one must pass the previously mentioned path towards the SL Library towards ROS. This is accomplished in the `.launch` files that are conatined within the `/ros/test/`  directory. In each of these files one must alter the following line of code:
    ```
    <arg name="sl_classpath" default="-Djava.class.path=/path/to/SL_Library"/>
    ```
	with the path to the SL Library. 

To install jpy in non-Linux systems and for troubleshooting, we recommend to proceed as described in the README file distributed with the source code. Although the online documentation is not updated, you can also read the [official jpy installation](https://jpy.readthedocs.io/en/latest/install.html) guide.

## Building the SL library
To build the subjective logic library we need to install gradle:
    ```
    sudo apt install gradle
    ```

The provided code for our SL library is a java project for Gradle. The build process amounts to run the command `gradle build` in your terminal or through your IDE of preference in the `knowledege_aggregation/subjective_logic` folder. On success, you may find the respective library JAR file under the `build` directory. This JAR file is the one to be used in our testbed; for more details read the [ROS implementation](../ros/README.md) release notes.

Additionally, it is possible to generate the library's API documentation by running the command `gradle build javadoc`. This documentation describe the different methods available to perform knowledge aggregation using the SL library.

## SL Library usage in Python

Python programs that import the `jpy` module can load Java classes, access Java class fields, and call class constructors and methods:

```
import jpy

File = jpy.get_type('java.io.File')

file = File('test/it')
name = file.getName()
```

Concretely, in our use case, the file [`testjpy.py`](../subjective_logic/testjpy.py) illustrates the usage of our SL library using jpy.
