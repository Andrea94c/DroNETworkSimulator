import numpy as np
import pickle
import pandas as pd
import seaborn as sb
import json
import matplotlib.pyplot as plt
from dataclasses import dataclass, fields

""" Metrics class keeps track of all the metrics during all the simulation. """


@dataclass
class Metrics:

    simulation_id = None
    mean_number_of_relays: float = 0.0
    number_of_generated_events: int = 0
    number_of_detected_events: int = 0
    all_control_packets_in_simulation: int = 0
    all_data_packets_in_simulation: int = 0
    number_of_events_to_depot: int = 0
    number_of_packets_to_depot: int = 0
    packet_mean_delivery_time: float = 0.0
    event_mean_delivery_time: float = 0.0
    mean_numbers_of_possible_relays = 0.0
    events: int = 0
    events_not_listened: int = 0
    all_packets_correctly_sent_by_drones: int = 0
    drones_packets_to_depot: int = 0
    time_on_mission: int = 0
    time_on_active_routing: int = 0

    def save_as_json(self, filename):
        """ save all the metrics into a json file """
        out = str(self)
        js = json.dumps(out)
        f = open(filename, "w")
        f.write(js)
        f.close()

    def __str__(self):
        return self.__repr__()

    def __repr__(self):

        cls = self.__class__
        cls_name = cls.__name__
        indent = ' ' * 4
        res = [f'Simulation {self.simulation_id} {cls_name}(']

        for f in fields(cls):

            if f == self.simulation_id:

                continue

            value = getattr(self, f.name)
            res.append(f'{indent}{f.name} = {value!r},')

        res.append(')')
        return '\n'.join(res)

