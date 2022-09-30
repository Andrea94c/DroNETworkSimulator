class SimulatedEntity:
    """
    A simulatedEntity keeps track of the simulation object, where you can access all the parameters
    of the simulation. No class of this type is directly instantiable.
    """
    def __init__(self, simulator):
        self.simulator = simulator


class Entity(SimulatedEntity):
    """
    The class Entity represents everything in the environment, for instance
    Drones, Events, Packets, etc... It extends SimulatedEntity.
    """

    def __init__(self, identifier: int, coords: tuple, simulator):
        super().__init__(simulator)
        self.identifier = identifier  # the id of the entity
        self.coords = coords          # the coordinates of the entity on the map

    def __eq__(self, other):
        """
        Entity objects are uniquely identified by their identifier
        """
        if not isinstance(other, Entity):
            return False
        else:
            return other.identifier == self.identifier

    def __hash__(self):
        """
        The hash function is applied to the identifier and the current coordinates
        """
        return hash((self.identifier, self.coords))
