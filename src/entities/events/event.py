from src.entities.generic.entity import Entity
from src.entities.packets.packets import DataPacket


class Event(Entity):
    """
    An Event object is any kind of event that the drone detects in the Area-of-Interest
    """
    def __init__(self, simulator, coordinates: tuple, current_time: int, deadline=None):
        """

        @param coordinates:
        @param current_time:
        @param simulator:
        @param deadline:
        """

        super().__init__(simulator, id(self), coordinates)
        self.current_time = current_time

        # One can specify the deadline or just consider as deadline now + EVENTS_DURATION
        # The deadline of an event represents the estimate of the drone that the event will be no more
        # interesting to monitor.
        self.deadline = current_time + self.simulator.event_duration if deadline is None else deadline

        # add metrics: all the events generated during the simulation
        # GENERATED_EVENTS
        if not coordinates == (-1, -1) and not current_time == -1:
            self.simulator.metrics.events.add(self)

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
        @param drone:
        @return:
        """

        # Notice: called only when a packet is created

        packet = DataPacket(simulator=self.simulator, event_ref=self)
        packet.add_hop(drone)

        return packet

    def __repr__(self):
        """

        @return:
        """

        return f"Event id:{str(self.identifier)} coordinates:{str(self.coordinates)}"

    def to_json(self):
        """

        @return:
        """

        event_dict = {
            "Event_identifier": self.identifier,
            "Event_coordinates": self.coordinates,
            "Event_generation_time": self.current_time,
            "Event_deadline": self.deadline
            }

        return event_dict
