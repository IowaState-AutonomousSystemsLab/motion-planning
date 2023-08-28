#! /usr/bin/python3

class PIDF:
    """

    """
    def __init__(self, kP, kI, kD, kF):
        # Initialize the PID controller with provided constants
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF

        # Initialize control parameters
        self.setpoint = 0
        self.prevError = 0
        self.integral = 0
        self.clampingLowerLimit = -1
        self.clampingUpperLimit = 1

    def set(self, setpoint):
        # Set the desired setpoint for the controller
        self.setpoint = setpoint

    def clamp(self, output):
        # Ensure the output value is within the defined limits
        if output >= 0:
            return min(output, self.clampingUpperLimit)
        else:
            return max(output, self.clampingLowerLimit)

    def checkSaturation(self, output):
        # Check if the output value is saturated
        if self.clamp(output) != output:
            return 1  # Output is saturated
        else:
            return 0

    def checkSign(self, error, output):
        # Check if error and output have the same sign
        if (error > 0 and output > 0) or (error < 0 and output < 0):
            return 1 #Integral is adding to the output
        else:
            return 0

    def checkError(self, current):
        # Adjust error under specific conditions to reduce it to zero
        error = self.setpoint - current
        if (self.setpoint == 0.5) and (0.48 < current < 0.52):
                error = 0
        elif (self.setpoint == 0.4) and (0.38 < current < 0.42):
                error = 0
        elif (self.setpoint == 0.3) and (0.28 < current < 0.32):
                error = 0
        elif (self.setpoint == 0.2) and (0.18 < current < 0.22):
                error = 0 
        elif (self.setpoint == 0.1) and (0.08 < current < 0.12):
                error = 0
        elif (self.setpoint == 0.0) and (-0.05 < current < 0.05):
                error = 0

        return error 

    def update(self, current, dt):
        # Update the controller with current value and time step
        error = self.checkError(current)

        derivative = (error - self.prevError) / dt
        self.integral += error 
        power = self.kP * error + self.kI * self.integral * dt + self.kD * derivative

        if self.checkSaturation(power) == 1 and self.checkSign(error, power) == 1:
            self.integral = 0

        newPower = self.kP * error + self.kI * self.integral * dt + self.kD * derivative
        self.prevError = error

        return newPower