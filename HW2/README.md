# Homework 2 - Laser Mapper
The goal of this homework is to modify the files provided in order to color the pixel for both the points perceived by the scanner and the points corresponding to the positions of the robot. 

### Needed stuff

I suggest you to configure the entire workspace, as described [here](https://sites.google.com/studenti.uniroma1.it/ing-informatica-sapienza/labiagi) (currently only in Italian), to get all the necessary for this homework and for the following ones. You will also need a bag (the willow-erratic bag would be nice, but you can use the one provided [here](https://gitlab.com/tizianoGuadagnino/lab_ai_gi/tree/master/datasets)).

### To run the exercise
- Download the HW2 folder containing all the required files and put it in a convenient location.
- In a terminal go to the src folder of the labaigi workspace and run `ln -s [directory]` where [directory] is the path on your PC of the HW2 folder (ex. /home/username/Desktop/HW2)
- Run `catkin build` in the labaigi workspace to build all packages
- Run `roscore`, then in another terminal `source devel/setup.bash` and `rosrun laser_mapper laser_mapper_node`
