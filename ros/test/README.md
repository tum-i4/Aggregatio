# Tests
The files within this folder are used to start one or multiple ros-simulations and save certain topics of these simulations. There are two types of files within this folder. The files ending with `.launch` are the ros-launch files. There exists one launch-files for 1, 2, ..., 5  robots. Please note, that the launch file for the two-robot case is called `master.launch`. 

The second type of file within the folder are the bash-scripts which provide launch files with the specific parameters of a simulation and save desired topics. There are again two types of bash-scripts. The first type only starts a single experiments with the desired parameters and saves certain topics of this experiment. These files are called `launch_#robots.sh`. The second type of bash-script is called `launch_#r_tests.sh` which are scripts that execute multiple tests consecutively. These scripts use mutex to create virtual tabs, which allows the execution of these tests via SSH. If one prefers to open actual tabs on their local computer the file `launch_2r_tests.sh` shows how this is done. Please note, that there might be different naming conventions for the saved data for different files due to convenience. 

Lastly, please make sure that the classpath of the SL library has been varied in the `.launch` file that you are using. 

