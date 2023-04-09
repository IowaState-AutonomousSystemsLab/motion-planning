class Controller:
    """
    Controller interface for purepursuit. Pure Pursuit needs a controller
    that will move the robot in a way that will minimize the error.
    """
    def minimize(error: float):
        raise NotImplementedError()