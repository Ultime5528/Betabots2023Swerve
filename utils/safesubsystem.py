import commands2


class SafeSubsystem(commands2.SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName(self.__class__.__name__)
        self.setSubsystem(self.__class__.__name__)