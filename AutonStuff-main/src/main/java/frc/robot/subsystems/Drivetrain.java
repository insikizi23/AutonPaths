// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public static final double WHEEL_DIAMETER_METERS = .158;
  public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
  public static final double TRACK_WIDTH_METERS = 0.5883;
  public static final double TICKS_PER_ROTATION = 2048;
  public static final double GEAR_RAIO = 11.2444;
  public static final double DRIVE_S = 0.166;
  public static final double DRIVE_V = 2.41;
  public static final double DRIVE_A = 0.25;
  // we won't really use them because SVA is good enough, but good to have in case
  // we need further accuracy due to external factors
  public static final double DRIVE_P = 0;
  public static final double DRIVE_I = 0;
  public static final double DRIVE_D = 0;
  public static final double MAX_VELOCITY_METERS = 1.5;
  public static final double MAX_ACCELERATION_METERS = 2.0;
  public static final int TIMEOUT_MS = 10;
  public static final int MAX_VOLTAGE = 10;

  // motors, kinda obvious
  TalonFX rightMaster;
  TalonFX rightFollower;

  TalonFX leftMaster;
  TalonFX leftFollower;

  // gyro, also obvious
  AHRS gyro;

  // kinematics converts needed linear + angular velocity to velocities for left
  // and right side
  DifferentialDriveKinematics kinematics;

  // estimates position and the direction it is facing (ex. 8, 5, 45 degrees)
  DifferentialDriveOdometry odometry;

  // stores the current position of the robot
  Pose2d currentPosition;

  // does math with SVA constants
  SimpleMotorFeedforward feedforward;

  // PID Controllers
  PIDController rightController;
  PIDController leftController;

  TrajectoryConfig trajectoryConfig;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    rightMaster = new TalonFX(0);
    rightFollower = new TalonFX(1);

    leftMaster = new TalonFX(2);
    leftFollower = new TalonFX(3);

    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();

    // makes sure that the maximum voltage that the motor can pull
    leftMaster.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
		leftFollower.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
		rightMaster.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
		rightFollower.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);

		leftMaster.enableVoltageCompensation(true);
		leftFollower.enableVoltageCompensation(true);
		rightMaster.enableVoltageCompensation(true);
		rightFollower.enableVoltageCompensation(true);

    rightMaster.setInverted(false);
    rightFollower.setInverted(false);
    leftMaster.setInverted(true);
    leftFollower.setInverted(true);

    rightFollower.follow(rightMaster);
    leftFollower.follow(leftMaster);

    gyro = new AHRS(SPI.Port.kMXP);

    // kinematics takes in track width of robot (distance between wheels)
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // odometry takes in the current direction the robot is facing using the
    // Rotation2d class
    odometry = new DifferentialDriveOdometry(getDirection());

    // Voltage = kS * sgn(v) + kV * v + kA * a
    // S term overcomes static friction in the direction of velocity
    // V term is multiplied by velocity
    // A term is multiplied by acceleration
    // added together they magically get us the voltage required to get to the
    // target velocity
    // good to know, but we won't be implementing the math because WPILib makes it
    // easier

    // to get SVA constants use frc characterization
    // install: pip install frc-characterization
    // run: frc-characterization drive new
    // give it some constants, save config, read config, generate project, launch
    // data logger, run routines, launch data analyzer, get SVA

    feedforward = new SimpleMotorFeedforward(DRIVE_S, DRIVE_V, DRIVE_A);

    rightController = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);
    leftController = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);

    trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY_METERS, MAX_ACCELERATION_METERS);

    // last generate trajectory with pathweaver
    // maxVelocity: 1.5
    // maxAcceleration: 2.0
    // wheelBase: 0.5883
    // gameName: Infinite Recharge
  }

  public TrajectoryConfig getTrajectoryConfig() {
    return trajectoryConfig;
  }

  /**
   * Sets the voltage to each motor in volts
   * @param right voltage to the right motor
   * @param left  voltage to the left motor
   */
  public void setOutputVoltage(double right, double left) {
    rightMaster.set(ControlMode.PercentOutput, right / MAX_VOLTAGE);
    leftMaster.set(ControlMode.PercentOutput, left / MAX_VOLTAGE);
  }

  /**
   * @return the direction of the robot using the Rotation2d class
   */
  public Rotation2d getDirection() {
    // gyro angle is negative because in AHRS, clockwise is positive and in WPILib's
    // math stuff counterclockwise is positive
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * @return the distance that the left side of the robot has travelled in meters
   */
  public double getLeftDistance() {
    // first calculate the amount of motor ticks that the left side has travelled by
    // taking the average of the two left encoder values
    double ticksTravelled = (leftMaster.getSelectedSensorPosition() + leftFollower.getSelectedSensorPosition()) / 2;

    // then calculate the meters travelled by converting ticks to rotations and then
    // multiplying by the wheel circumfrence
    double metersTravelled = ticksToMeters(ticksTravelled);

    return metersTravelled;

  }

  /**
   * @return the distance that the right side of the robot has travelled in meters
   */
  public double getRightDistance() {
    // first calculate the amount of motor ticks that the right side has travelled
    // by taking the average of the two left encoder values
    double ticksTravelled = (rightMaster.getSelectedSensorPosition() + rightFollower.getSelectedSensorPosition()) / 2;

    double metersTravelled = ticksToMeters(ticksTravelled);

    return metersTravelled;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getRightController() {
    return rightController;
  }

  public PIDController getLeftController() {
    return leftController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getCurrentPosition() {
    return currentPosition;
  }

  @Override
  public void periodic() {
    currentPosition = odometry.update(getDirection(), getLeftDistance(), getRightDistance());
  }

  public double ticksToMeters(double ticks) {
    // convert ticks to meters by converting ticks to rotations and multiplying by circumfrence
    double meters = ticks / GEAR_RAIO / TICKS_PER_ROTATION * WHEEL_CIRCUMFRENCE_METERS;

    return meters;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      ticksToMeters(leftMaster.getSelectedSensorVelocity()), 
      ticksToMeters(rightMaster.getSelectedSensorVelocity())
    );
  }

  /**
   * logs data about subsystem to SmartDashboard
   */
  public void log() {
    SmartDashboard.putNumber("Left Ouput", leftMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Ouput", rightMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Direction", -gyro.getAngle());
  }
}
