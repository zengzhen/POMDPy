from __future__ import print_function
from builtins import range
from pomdpy.discrete_pomdp import DiscreteState


class MultiObjectState(DiscreteState):
    """
    Need to difine what information should be contained in the state
    Here lists some essential infromation:
    The current room that the robot is current in. (Will be used by action move(r))
    The current robot position. Should be in metric value. 
    The scene graph.
    The object position. (This may include the robot position, the robot position should also include the semantic postion
    i.e. the room it is currently in.)
    """

    def __init__(self, object_position, scene_graph):
        self.position = object_position
        self.scene_graph = scene_graph

    def distance_to(self, other_rock_state):
        # This function defines the difference between two different states, currently not useful
        pass

    def __eq__(self, other_rock_state):
        return self.position == other_rock_state.position and self.scene_graph is other_rock_state.scene_graph

    def copy(self):
        return MultiObjectState(self.position, self.scene_graph)

    def __hash__(self):
        """
        Returns a decimal value representing the binary state string
        :return:
        """
        return int(self.to_string(), 2)

    def to_string(self):
        # Need to be designed to transfer the state to string
        state_string = "Needed to be implemented"
        return state_string

    def print_state(self):
        # Need to be designed to visualize the state
        pass

    def as_list(self):
        # Seems to be currently useless for our model
        pass

    def separate_rocks(self):
        # Seems to be currently useless for our model
        pass