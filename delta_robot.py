"""
Stepper Toothed Idler Pulley

teeth diameter: 12 mm
outer belt diameter: 13.8 mm

circumference = 2 * 3.1415926 * (12 / 2) = 37.7 mm


"""

from math import copysign

from micropython import alloc_emergency_exception_buf
from machine import (
    Pin,
    Timer,
)
from utime import (
    sleep_ms,
    ticks_ms,
)


OFF_COIL_STATES = (0, 0, 0, 0)

HOME = -1
A, B, C = 'A', 'B', 'C'

UP = FORWARD = 0
DOWN = REVERSE = 1

FULL_STEP_COIL_STATE_SEQUENCE = (
    (1, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 1),
    (1, 0, 0, 1),
)


DELTA_ROBOT_MOTION_TIMER = Timer(0)
CARRIAGE_LIMIT_SWITCH_DEBOUNCE_TIMER = Timer(1)


# Allocate a Micropython exception buffer to ensure that ISR-related errorr are
# reported. See: https://docs.micropython.org/en/latest/reference/isr_rules.html#the-emergency-exception-buffer
alloc_emergency_exception_buf(100)


class Stepper(object):

    COIL_STATE_SEQUENCE = FULL_STEP_COIL_STATE_SEQUENCE
    MIN_INTRA_STATE_DELAY_MS = 2

    IDLER_CIRCUMFERENCE_MM = 37.7
    STEPS_PER_REVOLUTION = (360 / 11.25) * 64
    MM_PER_STEP = IDLER_CIRCUMFERENCE_MM / STEPS_PER_REVOLUTION


    def __init__(self, coil_pin_nums):
        # Create the Pin objects for the specified coil output pins.
        self.coil_pins = [Pin(n, Pin.OUT) for n in coil_pin_nums]
        self.off()

        self.current_coil_state_sequence_index = 0

        # Init the last state change timestamp to (<now> - <minDelay>)
        self.last_state_change_ms = ticks_ms() - self.MIN_INTRA_STATE_DELAY_MS


    def set_coil_states(self, coil_0, coil_1, coil_2, coil_3):
        coil_pins = self.coil_pins
        coil_pins[0].value(coil_0)
        coil_pins[1].value(coil_1)
        coil_pins[2].value(coil_2)
        coil_pins[3].value(coil_3)


    def off(self):
        self.set_coil_states(0, 0, 0, 0)


    def get_next_sequence_index(self, direction):
        """ Return the next coil state sequence index in the specified
        direction.
        """
        if direction == FORWARD:
            next_index = self.current_coil_state_sequence_index + 1
            if next_index == len(self.COIL_STATE_SEQUENCE):
                next_index = 0
        else:
            next_index = self.current_coil_state_sequence_index - 1
            if next_index == -1:
                next_index = len(self.COIL_STATE_SEQUENCE) - 1
        return next_index


    def step(self, direction):
        # Get the next sequence index.
        next_index = self.get_next_sequence_index(direction)
        self.current_coil_state_sequence_index = next_index

        # If there hasn't elapsed at least MIN_INTRA_STATE_DELAY_MS
        # milliseconds between the last state change and now, wait it out.
        ms_since_last_step = ticks_ms() - self.last_state_change_ms
        ms_to_wait = self.MIN_INTRA_STATE_DELAY_MS - ms_since_last_step
        if ms_to_wait > 0:
            sleep_ms(ms_to_wait)
        self.last_state_change_ms = ticks_ms()

        # Update the coil state.
        self.set_coil_states(*self.COIL_STATE_SEQUENCE[next_index])


class CarriageError(Exception): pass

class UnknownCarriagePosition(CarriageError): pass


class Carriage(object):

    LIMIT_SWITCH_ACTIVE_VALUE = 0
    LIMIT_SWITCH_DEBOUNCE_TICKS = 30
    LIMIT_SWITCH_DEBOUNCE_TICK_PERIOD = 1

    debounce_timer = CARRIAGE_LIMIT_SWITCH_DEBOUNCE_TIMER
    debounce_name_carriage_map = {}


    @classmethod
    def tick_debounce(cls, timer):
        for name, carriage in list(cls.debounce_name_carriage_map.items()):
            if carriage.limit_switch_debounce_ticks_remaining > 0:
                carriage.limit_switch_debounce_ticks_remaining -= 1
                if carriage.limit_switch_debounce_ticks_remaining == 0:
                    del cls.debounce_name_carriage_map[carriage.name]
        if not cls.debounce_name_carriage_map:
            timer.deinit()


    @classmethod
    def start_debouncing(cls, carriage):
        debounce_name_carriage_map = cls.debounce_name_carriage_map
        debounce_name_carriage_map[carriage.name] = carriage
        if len(debounce_name_carriage_map) == 1:
            cls.debounce_timer.init(period=1, mode=Timer.PERIODIC,
                                    callback=cls.tick_debounce)


    def __init__(self, stepper, limit_sw_pin_num, name):
        self.stepper = stepper
        self.limit_switch_pin = Pin(limit_sw_pin_num, Pin.IN, Pin.PULL_UP)
        self.name = name

        self.at_limit = self.limit_switch_pin.value() == \
                        self.LIMIT_SWITCH_ACTIVE_VALUE
        self.limit_switch_debounce_ticks_remaining = 0
        self.enable_limit_switch_interrupts()

        self.z = 0 if self.at_limit else None
        self.z_error = 0


    def enable_limit_switch_interrupts(self):
        self.limit_switch_pin.irq(
            trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
            handler=self.limit_switch_irq_handler,
        )


    def limit_switch_irq_handler(self, limit_switch_pin):
        if self.limit_switch_debounce_ticks_remaining > 0:
            return

        self.at_limit = not self.at_limit

        self.limit_switch_debounce_ticks_remaining = \
            self.LIMIT_SWITCH_DEBOUNCE_TICKS

        Carriage.start_debouncing(self)

        print('Carriage: {}, at_limit: {}'.format(self.name, self.at_limit))


    def step_stepper(self, direction):
        if direction == UP and self.at_limit:
            self.z = 0
            self.z_error = 0
            return False

        self.stepper.step(direction)

        if self.z is not None:
            if direction == UP:
                self.z -= self.stepper.MM_PER_STEP
            else:
                self.z += self.stepper.MM_PER_STEP

        return True


    def move_mm(self, mm, direction):
        num_steps = int(mm / self.stepper.MM_PER_STEP)
        while num_steps:
            # Silently break loop if stepper can't move as requested.
            if not self.step_stepper(direction):
                break
            num_steps -= 1
        self.stepper.off()


    def get_z_delta(self, z):
        """Return the delta to the specified z and the error in precision
        given the minimum stepper step distance.
        """
        if self.z is None:
            raise UnknownCarriagePosition

        z_delta = z - self.z
        error = z_delta % copysign(self.stepper.MM_PER_STEP, z_delta)
        return z_delta, error


    def step_toward_home(self):
        return not self.step_stepper(UP)


    def home(self):
        while not self.step_toward_home():
            pass
        self.stepper.off()


    def step_toward_z(self, z):
        """Take one step toward the specified z value and return a boolean
        indicating whether we've reached it.
        """
        z_delta, error = self.get_z_delta(z)
        # TODO - accumulate the error value to use for compensation.
        if z_delta == 0:
            return True

        direction = DOWN if z_delta > 0 else UP

        if direction == UP and self.at_limit:
            raise AssertionError('Carriage {} was about to move UP but limit '\
                                 'switch is active. self.z={}'.format(
                                     self.name, self.z))

        self.step_stepper(direction)
        return abs(self.z - z) < abs(error)


class DeltaRobot(object):
    def __init__(self, carriage_A, carriage_B, carriage_C):
        self.carriages = {
            A: carriage_A,
            B: carriage_B,
            C: carriage_C,
        }
        self.carriage_targets = {
            A: None,
            B: None,
            C: None,
        }
        self.xyz = (None, None, None)
        self.motion_timer = DELTA_ROBOT_MOTION_TIMER
        self.moving = False


    def step_toward_targets(self):
        carriage_targets = self.carriage_targets

        num_stepped = 0
        for name, carriage in self.carriages.items():
            target = carriage_targets[name]
            if target is None:
                continue

            if target == HOME:
                done = carriage.step_toward_home()
            else:
                done = carriage.step_toward_z(target)

            if done:
                carriage_targets[name] = None
                carriage.stepper.off()

            num_stepped += 1
        return num_stepped


    def start_moving(self):
        def isr(timer):
            num_stepped = self.step_toward_targets()
            if num_stepped == 0:
                timer.deinit()
                self.moving = False

        self.motion_timer.init(period=1, mode=Timer.PERIODIC, callback=isr)
        self.moving = True


    def move_to_targets(self, A_target, B_target, C_target):
        self.carriage_targets['A'] = A_target
        self.carriage_targets['B'] = B_target
        self.carriage_targets['C'] = C_target
        self.start_moving()


    def home(self):
        self.move_to_targets(HOME, HOME, HOME)


    def move_to_point(self, x, y, z):
        from kinematics import (
            calc_carriage_z_for_point,
            invert_for_tower
        )

        Az, Bz, Cz = invert_for_tower(*calc_carriage_z_for_point(x, y, z))

        self.move_to_targets(Az, Bz, Cz)
        self.xyz = (x, y, z)


    def move_to_sequence(self, coords):
        for coord in coords:
            self.move_to_point(*coord)

            while self.moving:
                sleep_ms(1)


    def off(self):
        self.motion_timer.deinit()
        for carriage in self.carriages.values():
            carriage.stepper.off()


###############################################################################


delta_robot = DeltaRobot(
    carriage_A=Carriage(Stepper((16, 17, 18, 19)), 23, 'A'),
    carriage_B=Carriage(Stepper((2, 4, 5, 21)), 26, 'B'),
    carriage_C=Carriage(Stepper((13, 12, 14, 27)), 15, 'C'),
)
