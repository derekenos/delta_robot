"""
From: http://fab.cba.mit.edu/classes/863.15/section.CBA/people/Spielberg/Rostock_Delta_Kinematics_3.pdf

S = distance between tower midpoints
DELTA_SMOOTH_ROD_OFFSET = distance from tower midpoint to center
L = DELTA_DIAGONAL_ROD = length of arm
E = DELTA_EFFECTOR_OFFSET = distance from effector center to connecting rod midpoint

Acz = the height of carriage A above the effector platform
Bcz = the height of carriage B above the effector platform
Ccz = the height of carriage C above the effector platform

Az = the height of carriage A above the bed
Bz = the height of carriage B above the bed
Cz = the height of carriage C above the bed

Hez = distance that the head extends beloew the effector

Z = the height of the head above the bed

So...
Xz (carriage X height) = Z (head above bed) + Xcz (carriage above effector) +
                         Hez (head beyond effector)

Az = Z + Acz + Hez
Bz = Z + Bcz + Hez
Cz = Z + Ccz + Hez

We can simplify the calculations by deriving formulas for the X, Y plane and
remove Z from the equation, because we can translate the X, Y plane to any Z
location by moving all the carriages the same amount up or down.

All we need to concern ourselves with is Acz, Bcz, and Ccz


Terminology for next section:

L = DELTA_DIAGONAL_ROD = length of arm
Aco, Bco, Cco = DELTA_CARRIAGE_OFFSET = distance from tower midpoint to joint
DR, Ad, Bd, Cd = DELTA_RADIUS = horizontal distance from joint to joint of arm
R = DELTA_SMOOTH_ROD_OFFSET = distance from tower midpoint to center


"""

from math import (
    sqrt,
)

L = 122
L_SQUARED = pow(122, 2)


DR = 85.767
Avx, Avy = 0, DR
Bvx, Bvy = DR * (sqrt(3) / 2), -DR / 2
Cvx, Cvy = -DR * (sqrt(3) / 2), -DR / 2


def calc_carriage_z_for_point(x, y, z):
    Acz = sqrt(L_SQUARED - pow(x - Avx, 2) - pow(y - Avy, 2))
    Bcz = sqrt(L_SQUARED - pow(x - Bvx, 2) - pow(y - Bvy, 2))
    Ccz = sqrt(L_SQUARED - pow(x - Cvx, 2) - pow(y - Cvy, 2))

    Acz, Bcz, Ccz = [x + z for x in (Acz, Bcz, Ccz)]

    return Acz, Bcz, Ccz


CARRIAGE_HEIGHT = 194
def invert_for_tower(Acz, Bcz, Ccz):
    return [CARRIAGE_HEIGHT - x for x in (Acz, Bcz, Ccz)]
