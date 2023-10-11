import commands2
import wpilib


def putCommandOnDashboard(sub_table: str, cmd: commands2.CommandBase, name=None):
    if sub_table:
        sub_table += "/"
    else:
        sub_table = ""

    if name is None:
        name = cmd.getName()

    print(sub_table + name)

    wpilib.SmartDashboard.putData(sub_table + name, cmd)

    return cmd