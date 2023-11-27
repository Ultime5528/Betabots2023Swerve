from subsystems.loader import Loader
from utils.safecommand import SafeCommand


class Load(SafeCommand):
    def __init__(self, loader: Loader):
        super().__init__()
        self.addRequirements(loader)
        self.loader = loader

    def execute(self) -> None:
        self.loader.revolve()

    def end(self, interrupted: bool) -> None:
        self.loader.stop()
