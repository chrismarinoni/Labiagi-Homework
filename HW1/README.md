# Homework 1 - Orazio Wrapper
The goal of this homework is to realize a C++ wrapper class for the Orazio Client. The task can be easily accomplished by looking to the C file orazio_client.c (in the "orazio_host" directory).

### Needed stuff

You need to download the Orazio files from the official [repository](https://gitlab.com/srrg-software/srrg2_orazio). I suggest you to configure the entire workspace, as described [here](https://sites.google.com/studenti.uniroma1.it/ing-informatica-sapienza/labiagi) (currently only in Italian), to be ready for the following homework. You will also need an Arduino Mega to flash the Orazio firmware and test the code.

### How to run this code 
1) Move the folder that contains the wrapper files in srrg2_orazio/srrg2_orazio
2) Connect your Arduino Mega and run `make` in srrg2_orazio/srrg2_orazio/host_build
3) Compile using `make` in wrapper folder and check the solution with `./orazio_wrapper_test`
