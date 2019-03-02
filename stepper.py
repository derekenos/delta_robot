"""
Stepper Toothed Idler Pulley

teeth diameter: 12 mm
outer belt diameter: 13.8 mm

circumference = 2 * 3.1415926 * (12 / 2) = 37.7 mm


"""


from machine import Pin
from time import (
    ticks_ms,
    sleep_ms,
)


UP = FORWARD = 0
DOWN = REVERSE = 1

LIMIT_SWITCH_INACTIVE = 1

OFF_COIL_STATES = (0, 0, 0, 0)

FULL_STEP_COIL_STATE_SEQUENCE = (
    (1, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 1),
    (1, 0, 0, 1),
)


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


class Carriage(object):

    def __init__(self, stepper, limit_sw_pin_num):
        self.stepper = stepper
        self.limit_switch_pin = Pin(limit_sw_pin_num, Pin.IN, Pin.PULL_UP)


    def move_mm(self, mm, direction):
        num_steps = int(mm / self.stepper.MM_PER_STEP)

        print('num_steps: {}'.format(num_steps))

        while num_steps:
            self.stepper.step(direction)
            num_steps -= 1
        self.stepper.off()


    def go_home(self):
        while self.limit_switch_pin.value() == LIMIT_SWITCH_INACTIVE:
            self.stepper.step(UP)
        self.stepper.off()



###############################################################################

def demo():
    carriageA = Carriage(Stepper((16, 17, 18, 19)), limit_sw_pin_num=23)
    carriageB = Carriage(Stepper((21, 2, 4, 5)), limit_sw_pin_num=26)
    carriageC = Carriage(Stepper((13, 12, 14, 27)), limit_sw_pin_num=15)

    carriageA.go_home()
    carriageB.go_home()
    carriageC.go_home()

    n = 40
    while n:
        for c in (carriageA, carriageB, carriageC):
            c.move_mm(2, DOWN)
        n -= 1
