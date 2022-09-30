from src.entities.generic.entity import Entity
from src.entities.packets.packets import DataPacket


class Event(Entity):
    """ An event is any kind of event that the drone detects on the aoi. It is an Entity. """
    def __init__(self, coords: tuple, current_time: int, simulator, deadline=None):

        super().__init__(id(self), coords, simulator)
        self.current_time = current_time

        # One can specify the deadline or just consider as deadline now + EVENTS_DURATION
        # The deadline of an event represents the estimate of the drone that the event will be no more
        # interesting to monitor.
        self.deadline = current_time + self.simulator.event_duration if deadline is None else deadline

        # add metrics: all the events generated during the simulation
        # GENERATED_EVENTS
        if not coords == (-1, -1) and not current_time == -1:
            self.simulator.metrics.events.add(self)

    def to_json(self):
        """ return the json repr of the obj """
        return {"coord": self.coords,
                "i_gen": self.current_time,
                "i_dead": self.deadline,
                "id": self.identifier
                }

    def is_expired(self, cur_step):
        """ return true if the deadline expired """
        return cur_step > self.deadline

    def as_packet(self, time_step_creation, drone):
        """ build a packet out of the event, by default the packet has deadline set to that of the event
            so the packet dies at the same time of the event, then add the input drone as first hop
        """
        # Notice: called only when a packet is created

        pck = DataPacket(time_step_creation, self.simulator, event_ref=self)
        # if config.DEBUG_PRINT_PACKETS: print("data", pck, pck.src_drone, pck.dst_drone, self.current_time)
        pck.add_hop(drone)
        return pck

    def __repr__(self):
        return "Ev id:" + str(self.identifier) + " c:" + str(self.coords)
