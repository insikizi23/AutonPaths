# Going Over Auton Paths

## Drivetrain Subsystem

### Motors and motor getters / setters
- right master, follower, left master, ollower
-  regular config
- Voltage Compensation saturation voltage
- get left/right distance
- setOutput
- setOutputVoltage
- get motor speeds
- [link for talons](http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json)
### Gyro
- create gryo from AHRS
- getter for robot heading
- [link for AHRS](https://www.kauailabs.com/dist/frc/2021/navx_frc.json)
### Other classes needed, just initialize and add getter
- DifferentialDriveKinematics
- DifferentialDriveOdometry (no need forgetter for this)
- Pose2d (used to store current position)
- SimpleMotorFeedforward
- PIDController (left and right)
- TrajectoryConfig
### Other stuff
- update current in periodic
- log motor outputs and direction to smart dashboard

## FRC Characterization

### Installation
Have pip installed and execute the following command
```
pip install frc-characterization
```


### Using it
Run 
```
frc-characterization drive new
```
- Select Project Location
- Change control type to CTRE
- Change unit type to meters and set units per rotation to meters per rotation (0.496371639267 for 2020 practice bot)
- Edit config file
- Save Config -> Read Config -> Generate Project
- Connect to robot
- Deploy project -> Launch Data Logger -> Run tests -> Save data
- Launch Data Analyzer -> select data file -> get SVA constants :)

## PathWeaver
- Open PathWeaver
- Create Project
    - Project directory: root directory of robot project
    - Output directory: same as ^
    - Max Velocity: 1.5 m/s (for this at least)
    - Max Acceleration 2 m/s^2
- Plus button under paths to create a new path
    - Mess around with path
    - Build paths to save

## Auton Command

- Create a sequential command group
- Pass in drivetrain subsystem
- create getTrajectory method
    - Create trajectory using list of Poses with x, y, and Rotation2d(tangent x, tangent y) using x, y, tangetn x, and tangent y from PathWeaver
    - create the ramsete command (I'm not gonna explain it here, its really big)
- pass in the command returned from getTrajectory into addCommands

## Robot Container
- Create a drivetrain subsytem
- in getAutonomousCommand return an instance of the sequential command group you just created