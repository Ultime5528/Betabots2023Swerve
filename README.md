# Betabots 2023
### Writing Conventions 
* All code must be written in the English language
* Follow PyCharm style recommendations
* Commit names must be clear and informative
* Progress must be tracked with GitHub Projects (https://github.com/orgs/Ultime5528/projects/4)

* File names use lowercase without spaces
* Class names use PascalCase
* Function names use camelCase
* Variable names use snake_case
* Function and command names start with an action verb (get, set, move, start, stop...)
* Commands and subsystems inherit from SafeCommand and SafeSubsystem
* Ports  
    * Must be added to ports.py
    * Respect the naming convention : "subsystem" _ "component type"  _ "precision"
    * Example : drivetrain_motor_left
* Properties 
  * Must be added to properties.py 
  * Respect the naming convention : "subsystem/command" \_ "variable type" _ "precision"
  * Example : intake_speed_slow, climber_height_max
  * ntproperty strings are the same as their variables, ex:
    * **shooter_speed** = ntproperty("/Properties/**shooter_speed**", 1500, ...

### Environment setup
* Download the latest Miniconda version on your computer with the following link (https://docs.conda.io/en/latest/miniconda.html)
* Open Anaconda Prompt
* Run the following commands to make sure everything is up to date:
```commandline
  conda config --add channels conda-forge
  conda config --set channel_priority strict
  conda update conda
  conda update python
```
* Run the following command to create an environment named "frc2023":
```commandline
  conda create -n frc2023 python=3.11
```
* Add the environment to the interpreter on PyCharm
* Run the following command on the PyCharm terminal to add the requirements
```commandline
  pip install robotpy[commands2,navx,rev,sim,apriltag,photonvision] numpy
```

* To update robotpy :
```commandline
  pip install --upgrade robotpy[commands2,navx,rev,sim,apriltag,photonvision]
```
