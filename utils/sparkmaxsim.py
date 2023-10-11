import rev
from wpilib.simulation import SimDeviceSim


class SparkMaxSim:
    def __init__(self, spark_max: rev.CANSparkMax):
        self.sim_device = SimDeviceSim(f"SPARK MAX [{spark_max.getDeviceId()}]")
        self._position = self.sim_device.getDouble("Position")
        self._velocity = self.sim_device.getDouble("Velocity")

    def getPosition(self) -> float:
        return self._position.get()

    def setPosition(self, value: float):
        self._position.set(value)

    def getVelocity(self) -> float:
        return self._velocity.get()

    def setVelocity(self, value: float):
        self._velocity.set(value)