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
  public static final double DRIVE_S = 2.19;
  public static final double DRIVE_V = -0.00218;
  public static final double DRIVE_A = 0.061;
  public static final double DRIVE_P = 0;
  public static final double DRIVE_I = 0;
  public static final double DRIVE_D = 0;
  public static final double MAX_VELOCITY_METERS = 1.5;
  public static final double MAX_ACCELERATION_METERS = 2.0;
  public static final int TIMEOUT_MS = 10;
  public static final int MAX_VOLTAGE = 10;

  // motors
  TalonFX rightFront;
  TalonFX rightBack;

  TalonFX leftFront;
  TalonFX leftBack;

  // gyro
  AHRS gyro;

  // kinematics 
  DifferentialDriveKinematics kinematics;

  // odometry
  DifferentialDriveOdometry odometry;

  // stores current position of  robot
  Pose2d currentPosition;

  // does math with SVA constants
  SimpleMotorFeedforward feedforward;

  // PID Controllers
  PIDController rightController;
  PIDController leftController;

  TrajectoryConfig trajectoryConfig;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    rightFront = new TalonFX(3);
    rightBack = new TalonFX(5);

    leftFront = new TalonFX(2);
    leftBack = new TalonFX(1);

    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();

    // makes sure that the maximum voltage that the motor can pull
    leftFront.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
    leftFront.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
    rightFront.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
    rightBack.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);

		leftFront.enableVoltageCompensation(true);
		leftBack.enableVoltageCompensation(true);
		rightFront.enableVoltageCompensation(true);
		rightBack.enableVoltageCompensation(true);

    rightFront.setInverted(false);
    rightBack.setInverted(false);
    leftFront.setInverted(true);
    leftBack.setInverted(true);

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

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
    rightFront.set(ControlMode.PercentOutput, right / MAX_VOLTAGE);
    leftFront.set(ControlMode.PercentOutput, left / MAX_VOLTAGE);
  }

  /**
   * @return the direction of the robot using the Rotation2d class
   */
  public Rotation2d getDirection() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * @return the distance that the left side of the robot has travelled in meters
   */
  public double getLeftDistance() {
    double ticksTravel = (leftFront.getSelectedSensorPosition()+leftBack.getSelectedSensorPosition())/2;
    double metersTravel = ticksToMeters(ticksTravel);
    return metersTravel;
  }

  /**
   * @return the distance that the right side of the robot has travelled in meters
   */
  public double getRightDistance() {
    double ticksTravel = (rightFront.getSelectedSensorPosition()+rightBack.getSelectedSensorPosition())/2;
    double metersTravel = ticksToMeters(ticksTravel);
    return metersTravel;
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
    double meters = ticks / GEAR_RAIO / TICKS_PER_ROTATION * WHEEL_CIRCUMFRENCE_METERS;
    return meters;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      ticksToMeters(leftFront.getSelectedSensorVelocity()), 
      ticksToMeters(rightFront.getSelectedSensorVelocity())
    );
  }

  /* logs data about subsystem to SmartDashboard */
  public void log() {
    SmartDashboard.putNumber("Left Ouput", leftFront.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Ouput", rightFront.getMotorOutputPercent());
    SmartDashboard.putNumber("Direction", -gyro.getAngle());
  }
}
