# Necessary imports

from constants import INF


# Cell class
class Cell:
    # Initialise variables for the cell class
    def __init__(self):
        self.g = INF
        self.h = INF
        self.f = INF

        self.is_blocked = False
        self.four_neighbors = list()
        self.eight_neighbors = list()

        # newly added properties
        self.previous_examinations = 0
        self.previous_visits = 0
        self.max_threshold_of_examinations = 0

    # Reset attributes of this class
    def reset_except_h(self):
        self.g = INF
        self.f = INF

        self.is_blocked = False

        self.previous_examinations = 0
        self.previous_visits = 0
        self.max_threshold_of_examinations = 0

    def reset(self):
        self.reset_except_h()
        self.h = INF
