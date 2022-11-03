# MAC Bandit exercise

DroNET is a Python simulator for experimenting routing algorithms on unmanned aerial vehicle 
networks.

## Execution

In order to start the simulator from the terminal, clone the git project and place yourself in ``DroNETworkSimulator`` directory, then and run:

```bash
python -m src.main
```
You can also use the terminal from PyCharm using the same command. 
In order to get all the requirements properly installed you can use the virtual environment placed inside the main directory.
To activate the environment in **windows** use the command below. REMEMBER, you must be inside the DroNETworkSimulator folder.
```windows
.\venv\Scripts\activate
```
To activate the environment in **linux** use the command below. 
```bash
source .\venv\Scripts\activate
```
After the environment activation you can use pip to install all the requirements needed

```bash
pip install -r requirements.txt
```

The simulation will start in a new window, the parameters of the simulation are set in ``src.utilities.config``, 
 have a look at the simulation setup in the configuration file to understand what is going on in the 
 simulation. 

## Known Issues

**PLEASE** use python 3.7, other python versions have compatibility issues with some libraries.

- PyGame could not work with python versions > 3.7.
- To install PyGame you may need to use pip  

**IMPORTANT**: if you find other issues please drop a mail to us describing what is 
the issue including also screens or message errors flavio.giorgi[AT]uniroma1.it or giulio.attenni[AT]uniroma1.it
 
**PLEASE: USE [AUTONOMOUS NETWORKING] as email object and send the email always to both of us adding one of us in CC** 

## Project structure 
The project has the following structure:

The entry point of the project is the ``src.main`` file, from there you can run simulations and extensive
 experimental campaigns, by setting up an appropriate ``src.simulator.Simulator`` object. 
 
On a high level, the two main directories are ``data`` and ``src``. The directory ``data`` must contain all the 
data of the project, like drones tours, and other input and output of the project. The directory ``src`` 
contains the source code, organized in several packages. 

* ``src.drawing`` it contains all the classes needed for drawing the simulation on screen. Typically, you may 
want to get your hands in this directory if you want to change the aspect of the simulation, display a new 
object, or label on the area.

* ``src.entites`` it contains all the classes that define the behaviour and the structure of the main
 entities of the project like: Drone, Depot, Environment, Packet, Event classes.

* ``src.experiments`` it contains classes that handle experimental campaigns.

* ``src.routing_algorithms`` it contains all the classes modelling the several routing algorithms, 
**every** routing algorithm should have its own class.

* ``src.simulation`` it contains all the classes to handle a simulation and its metrics. 

* ``src.utilities`` it contains all the utilities and the configuration parameters. In particular use ``src.utilities.config`` file to 
specify all the constants and parameters for a one-shot simulation, ideal when one wants to evaluate
the quality of a routing algorithm making frequent executions. Constants and parameters should **always** be added here
and never be hard-coded.

## The assignment: BANDIT BASED MAC PROTOCOL
In order to build a proper Bandit based MAC protocol you should go to ``src.mac_protocol.ai_depot_mac.py`` and implement 
the ``allocate_resource_to_drone()`` function. You can use any kind of libraries, support classes or methods.

Inside the ``src.utilities.config.py`` file you can find all the simulation parameters. You should modify only the following 
parameters:


```python
MAC_ALGORITHM = MACAlgorithm.RND
MAC_PRINT_STATS = False
PLOT_HISTOGRAMS = True
MATPLOTLIB_TERMINAL = True
PLOT_SIM = False
```
When you implement the bandit mac protocol, in order to use it, you should select ``MAC_ALGORITHM = MACAlgorithm.AI`` 
We give you as examples two other naive mac protocols namely: random and round-robin.
For more details see the config file.

At the end of the simulation some metrics will be printed along with a terminal plot (if activated from config file) 


## Contacts
For further information contact Flavio Giorgi  **flavio.giorgi[AT]uniroma1.it**  or Giulio Attenni **giulio.attenni[AT]uniroma1.it**

## Thanks and License
The current version of the simulator is free for non-commercial use.
The simulator was done by Andrea Coletta in collaboration with Matteo Prata, PhD Student at La Sapienza  **coletta[AT]di.uniroma1.it**, **prata[AT]di.uniroma1.it**.


