
def grid_plot():
    for x, y in (
            (0, 0),
            (20, 20),
            (-20, 20),
            (20, -20),
            (-20, -20),

            (0, 20),
            (0, 15),
            (0, 10),
            (0, 5),
            (0, -5),
            (0, -10),
            (0, -15),
            (0, -20),

            (20, 0),
            (15, 0),
            (10, 0),
            (5, 0),
            (-5, 0),
            (-10, 0),
            (-15, 0),
            (-20, 0),
        ):

        for z in (64, 46.5, 64):
            yield x, y, z
