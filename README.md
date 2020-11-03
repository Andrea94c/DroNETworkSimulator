# UAV-Networks Simulator - Autonomous Networking - A.A. 20/21

UAV-Networks-Routing is a Python simulator for experimenting routing algorithms and mac protocols on unmanned aerial vehicle 
networks. The project requires Python 3, and several dependencies.
This code is released for the course of Autonomous Networking - A.A. 2020-2021, to develop and test AI based protocols. 

## Execution

In order to execute UAV-Networks-Routing project from the terminal, clone
the git project and place yourself in ``UAV-Networks-Routing`` directory, then and run:

```bash
python -m src.main
```

The simulation will start in a new window, the parameters of the simulation are set in ``src.utilities.config``, 
 have a look at the simulation setup in the configuration file to understand what is going on in the 
 simulation. 

## Project structure 
The project has the following structure:
```bash
.
├── README.md
├── data
│   └── tours
│       ├── RANDOM_missions1.json
│       ├── ...
│       └── RANDOM_missions90.json
└── src
    ├── main.py
    ├── drawing
    │   ├── color.py
    │   ├── picture.py
    │   ├── pp_draw.py
    │   └── stddraw.py
    ├── entities
    │   └── uav_entities.py
    ├── experiments
    ├── routing_algorithms
    │   └── georouting.py
    ├── simulation
    │   ├── metrics.py
    │   └── simulator.py
    └── utilities
        ├── config.py
        └── utilities.py
```

The entry point of the project is the ``src.main`` file, from there you can run simulations and extensive
 experimental campaigns, by setting up an appropriate ``src.simulator.Simulator`` object. 
 
On a high level, the two main directories are ``data`` and ``src``. The directory ``data`` must contain all the 
data of the project, like drones tours, and other input and output of the project. The directory ``src`` 
contains the source code, organized in several packages. 

* ``src.drawing`` it contains all the classes needed for drawing the simulation on screen. Typically you may 
want to get your hands in this directory if you want to change the aspect of the simulation, display a new 
object, or label on the area.

* ``src.entites`` it contains all the classes that define the behaviour and the structure of the main
 entities of the project like: Drone, Depot, Environment, Packet, Event classes.

* ``src.experiments`` it contains classes that handle experimental campaigns.

* ``src.routing_algorithms`` it contains all the classes modelling the several routing algorithms, 
**every** routing algorithm should have its own class, see section [Adding routing algorithms](#adding-routing-algorithms) below. 

* ``src.simulation`` it contains all the classes to handle a simulation and its metrics. 

* ``src.utilities`` it contains all the utilities and the configuration parameters. In particular use ``src.utilities.config`` file to 
specify all the constants and parameters for a one-shot simulation, ideal when one wants to evaluate
the quality of a routing algorithm making frequent executions. Constants and parameters should **always** be added here
and never be hard-coded.

## Understand the project
In this section it will be given a high level overview of the project. Before adding any new
file to the project, as a contribute, you may want to run some simulations, understand the idea
 behind the simulator, and the routing algorithm available. 

#### Make some simulations 
Run a simulation from ``src.main``, on a new window it will be displayed a live simulation. At the end of 
the simulation some metrics will be printed. In the main function, a ``Simulator`` object is instantiated
with default parameters coming from the ``src.utilities.config``. In order to make different executions 
and simulations, you may want to let the parameters in the config file vary appropriately. 

Let us make an example with an excerpt of the configuration file:

```python
SIM_DURATION = 7000   # int: steps of simulation. # ***
TS_DURATION = 0.150   # float: seconds duration of a step in seconds.
SEED = 1              # int: seed of this simulation.

N_DRONES = 5          # int: number of drones. # ***
ENV_WIDTH = 1500      # float: meters, width of environment.
ENV_HEIGHT = 1500     # float: meters, height of environment.

# events
EVENTS_DURATION = SIM_DURATION   # int: steps, number of time steps that an event lasts.
D_FEEL_EVENT = 500               # int: steps, a new packet is felt (generated on the drone) every 'D_FEEL_EVENT' steps. # ***
P_FEEL_EVENT = .25               # float: probability that the drones feels the event generated on the drone. # ***
```

From this excerpt, one expects a simulation that lasts for 7000 steps of 0.150 seconds each. The executions 
will run with seed 1, with 5 drones flying over an area of 1500m * 1500m. The events on the map last for the entire duration 
of the simulation. The drones are set to feel an event every 500 steps, but they feel it with probability 
0.25.

#### Simulator and K-Routing algorithm  
In the simulator, time is simulated. A simulation lasts for ``SIM_DURATION`` steps, lasting ``TS_DURATION`` 
seconds each. During a single step, as one can see from ``src.simulator.Simulator.run()``, essentially 4 
things happen, for every drone:

1.  it feels an event, if it's the right moment and if it is lucky enough to grasp it from the environment.

2.  it updates the packets in its buffer, deleting all the packets that are expired.

3. it routes its buffer to its neighbours, if it has any.
4. it sets its next waypoint and moves towards it, it can be either a point in the map, or the depot, 
depending on what the routing algorithm decides for it.

The UAVs can have any possible path/tour given by a json file (a dict id_drone : list of waypoints).
Notice that a waypoint is a 2-tuple (x, y), the coordinate of the point. Events are generated right on the
drone. If an event is successfully "felt", the drone generates a packet out of it and it is responsible to
bring it to the depot according to the routing algorithm currently running. Packets can expire and have a 
TTL to avoid infinite pin-pongs, _that are seen to be rare_.

The routing algorithms in the project go under the directory``src.routing_algorithms`` . 

#### Adding routing algorithms
Routing algorithms should be implemented as a class, extending the ``src.routing_algorithms.BASE_routing`` 
class. This will need the definition of required methods, such as: ``routing()``. 

Once created, the class should be declared in the configuration file, specifically in the ``RoutingAlgorithm`` enumeration, in 
which it suffices to give a name to the enumeration variable and associate it to the class name. For instance
if the created routing algorithm class is named ``MyRouting``, then add in ``src.utilities.config`` to the 
``RoutingAlgorithm`` enumeration the enumeration variable ``MY_ROUTING = MyRouting``.

To run a simulation with your new routing algorithms, just set the attribute ``ROUTING_ALGORITHM`` in the config file 
with the enumeration variable of your choice.

## Contacts
For further information contact Andrea Coletta at **coletta[AT]di.uniroma1.it**.

## Thanks and License
The current version of the simulator is free for non-commercial use.
The simulator was done in collaboration with Matteo Prata, PhD Student at La Sapienza **prata[AT]di.uniroma1.it**.


