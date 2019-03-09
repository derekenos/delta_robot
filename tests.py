
from sys import stdout

from delta_robot import delta_robot


def assert_equal(x, y):
    if x != y:
        raise AssertionError('{} != {}'.format(x, y))


def assert_close(x, y, threshold=0.000001):
    """Assert that the values are within some threshold of each other.
    """
    if abs(x - y) > threshold:
        raise AssertionError('{} and {} are not within {}'.format(
            x, y, threshold))


def assert_lt(x, y):
    if x >= y:
        raise AssertionError('{} not less than {}'.format(x, y))



class DeltaRobotTester():
    @staticmethod
    def test_carriage_get_z_delta():
        carriage = delta_robot.carriages['A']
        MM_PER_STEP = 0.0184082
        carriage.stepper.MM_PER_STEP = MM_PER_STEP

        for self_z, z, expected_z_delta, expected_error  in (
                (0, 0, 0, 0),
                (0, 1, 1, 0.005957200000000024),
                (0, 2, 2, 0.011914400000000047),
                (0, 3, 3, 0.01787160000000007),
                (0, 4, 4, 0.005420600000000095),
                (4, 0, -4, -0.005420600000000095),
            ):

            carriage.z = self_z
            z_delta, error = carriage.get_z_delta(z)

            assert_equal(z_delta, expected_z_delta)
            assert_close(error, expected_error)
            assert_lt(error, MM_PER_STEP)


if __name__ == '__main__':
    # Replace all Stepper.step() functions with a dummy lambda to disable them.
    for carriage in delta_robot.carriages.values():
        carriage.stepper.step = lambda *args, **kwargs: None

    print('Running all tests...')
    for k in dir(DeltaRobotTester):
        if k.startswith('test'):
            stdout.write('{}() : '.format(k))
            getattr(DeltaRobotTester, k)()
            stdout.write('pass\n')
