"""Path planners generate an iterable of points between two points that, when
when traversed, will result in the desired movement path.
"""

from math import sqrt


class XYPathPlanner():
    @staticmethod
    def get_xy_path(Ax, Ay, Bx, By):
        """Given the points (Ax, Ay) and (Bx, By), return an iterable of
        points to visit in order to effect the desired path (e.g. line, arc)
        while moving from A to B. The same plan can be used to move in the
        opposite direction by simply reversing the iterable.
        """
        raise NotImplementedError


class NoPlanPathPlanner(XYPathPlanner):
    """This planner has no plan; it just moves directly from A to B.
    """
    @staticmethod
    def get_xy_path(Ax, Ay, Bx, By):
        return ((Bx, By),)


class LinearPathPlanner(XYPathPlanner):
    """This planner moves in a straight line from A to B.
    """
    @staticmethod
    def get_xy_path(Ax, Ay, Bx, By):
        while True:
            # Calc the vector.
            vec = (Bx - Ax), (By - Ay)
            # Calc vector length.
            Vab = sqrt(vec[0] ** 2 + vec[1] ** 2)

            if Vab < .25:
                yield Bx, By
                break

            # Calc the unit vector.
            V1 = vec[0] / Vab / 4, vec[1] / Vab / 4

            # Yield the point.
            _Ax, _Ay = Ax + V1[0], Ay + V1[1]
            print(_Ax, _Ay)
            yield _Ax, _Ay

            Ax = _Ax
            Ay = _Ay
