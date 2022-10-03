from src.entities.generic.entity import Entity
from src.entities.packets.packets import DataPacket


class Event(Entity):
    """
    An Event object is any kind of event that the drone detects in the Area-of-Interest
    """
    def __init__(self, simulator, coords: tuple, current_time: int, deadline=None):
        """

        @param coords:
        @param current_time:
        @param simulator:
        @param deadline:
        """

        super().__init__(simulator, id(self), coords)
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

    @property
    def is_expired(self):
        """
        Check if the Event is expired
        @return: True if the event is expired False otherwise
        """
        return self.simulator.cur_step > self.deadline

    def as_packet(self, drone):
        """
        Build a Packet out of the Event, by default the packet has the same deadline of the event
        so the packet expire at the same time of the event.
        """
        # Notice: called only when a packet is created

        packet = DataPacket(self.simulator, event_ref=self)
        packet.add_hop(drone)

        return packet

    def __repr__(self):

        return f"Event id:{str(self.identifier)} coordinates:{str(self.coords)}"

