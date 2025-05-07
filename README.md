*** INTRODUCTION ***

This project impelments a custom controller for foraging in swarm robotics. Created for RBE 511 - Swarm Intelligence.

*** COMPILATION ***

To compile the project, first start by creating a build folder in the main directory.

$ mkdir build
$ cd build

To setup the compiler, run the following command in the new build folder:

$ cmake -DCMAKE_BUILD_TYPE=Release ..

Finally, launch the compilation with the command:

$ make

To run the simulation, use the following command:

$ argos3 -c foraging.argos

This ARGoS file offers a host of parameters that can be tuned, but to tune our custom hyperparameters change:

`float alpha = #;`

`float beta = #;`

within the *novelAlgorithm()* function in `footbot_foraging.cpp`.

To change the food selection algorithm, the method can be changed within the *Rest()* function in `footbot_foraging.cpp`.

*** RESULTS ***

The results for these algorithms can be found in the `/final_runs` directory.