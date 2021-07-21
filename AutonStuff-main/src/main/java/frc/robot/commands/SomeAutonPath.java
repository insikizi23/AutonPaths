// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SomeAutonPath extends SequentialCommandGroup {
  Drivetrain drivetrain;

  /** Creates a new SomeAutonPath. */
  public SomeAutonPath(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      getTrajectory()
    );
    this.drivetrain = drivetrain;
  }

  public Command getTrajectory() {

    // create a trajectory, takes in list of poses
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        // poses take in a position in (x, y) and a rotation 2d that is created with (x-tangent, y-tangent)
        new Pose2d(0.0,0.0, new Rotation2d(-3.1196117442565523,-0.47717175301615633)),
        new Pose2d(2.9906033556662894, -2.059560042847262, new Rotation2d(-0.501628527419868, -1.6846488418902912)),
        new Pose2d(3.4605597416883156, -4.780360172448468, new Rotation2d(1.0141164119422688, -1.4098691580660807))
      ), 
      drivetrain.getTrajectoryConfig()
    );

    /**
     * Arguments to this giant constructor
     * 
     * trajectory that you want to set to this command
     * getter method for current position (:: syntax just means passing a reference to the method)
     * ramsete controller with 2 and 0.7, idk what it does and why 2 and 0.7, but yes
     * feed forward variable from drivetrain
     * kinematics variale from drivetrain
     * getter method for current speeds fo the robot
     * left and right PID controllers
     * setter method to set speeds of robot motors
     */
    RamseteCommand command = new RamseteCommand(
      trajectory,
      drivetrain::getCurrentPosition,
      new RamseteController(2, 0.7),
      drivetrain.getFeedforward(),
      drivetrain.getKinematics(),
      drivetrain::getSpeeds,
      drivetrain.getLeftController(),
      drivetrain.getRightController(),
      drivetrain::setOutputVoltage
    );

    return command;
  }
}
