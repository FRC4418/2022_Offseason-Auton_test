// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Ports;
import frc.robot.constants.Settings;
import frc.robot.utills.Conversions;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.stuypulse.stuylib.math.SLMath;

public class Drivetrain extends SubsystemBase {

  final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(Ports.Drivetrain.LEFT_FRONT);
  final WPI_TalonFX leftBackMotor = new WPI_TalonFX(Ports.Drivetrain.LEFT_BACK);
  MotorControllerGroup leftGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(Ports.Drivetrain.RIGHT_FRONT);
  final WPI_TalonFX rightBackMotor = new WPI_TalonFX(Ports.Drivetrain.RIGHT_BACK);
  MotorControllerGroup rightGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

  public DifferentialDriveOdometry odometry;
  public DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(Settings.Drivetrain.TRACK_WIDTH);

  public ADIS16448_IMU imu = new ADIS16448_IMU();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFrontMotor.configFactoryDefault();
		leftBackMotor.configFactoryDefault();
		rightFrontMotor.configFactoryDefault();
		rightBackMotor.configFactoryDefault();

		leftBackMotor.follow(leftFrontMotor);
		rightBackMotor.follow(rightFrontMotor);

    // Config closed-loop constants
    leftFrontMotor.config_kF(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kF);
    leftFrontMotor.config_kP(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kP);
    leftFrontMotor.config_kI(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kI);
    leftFrontMotor.config_kD(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kD);

    rightFrontMotor.config_kF(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kF);
		rightFrontMotor.config_kP(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kP);
		rightFrontMotor.config_kI(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kI);
    rightFrontMotor.config_kD(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kD);

    
    // Config integrated sensors (built-in encoders)
		leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		resetEncoders();
    
    // Set one side of the robot to have inverted motors
    leftGroup.setInverted(true);
		rightGroup.setInverted(false);

    // Set Status Frame Period for lead motors
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Settings.Drivetrain.STATUS_FRAME_PERIOD, Settings.Drivetrain.STATUS_FRAME_TIMEOUT);
    rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Settings.Drivetrain.STATUS_FRAME_PERIOD, Settings.Drivetrain.STATUS_FRAME_TIMEOUT);

    // Set the motors to brake while idle
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    leftBackMotor.setNeutralMode(NeutralMode.Brake);
    rightBackMotor.setNeutralMode(NeutralMode.Brake);

    // Create odometry object to keep track of position
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }


	// Encoder methods
  public double getLeftDistance() {
		return leftFrontMotor.getSelectedSensorPosition() * Settings.Drivetrain.Encoders.ENCODER_DISTANCE_PER_PULSE;
	}

	public double getRightDistance() {
		return rightFrontMotor.getSelectedSensorPosition() * Settings.Drivetrain.Encoders.ENCODER_DISTANCE_PER_PULSE;
	}

	public double getAverageDistance() {
		return (getRightDistance() + getLeftDistance()) / 2.0d;
	}

	public void resetEncoders() {
		leftFrontMotor.setSelectedSensorPosition(0.d);
		rightFrontMotor.setSelectedSensorPosition(0.d);
	}


  // Trajectory methods
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void zeroHeading() {
    imu.reset();
  }

  public double getHeading(){
    //return Math.IEEEremainder(gyro.getAngle(), 360);
    return -1 * Math.IEEEremainder(imu.getAngle(),360);
  }

  public double getTurnRate(){
    return imu.getRate();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
                  Conversions.convertTalonSRXNativeUnitsToWPILibTrajecoryUnits(
                    this.leftFrontMotor.getSelectedSensorVelocity(), 
                    Settings.Drivetrain.Encoders.WHEEL_DIAMETER, 
                    true, 
                    Settings.Drivetrain.Encoders.ENCODER_PULSES_PER_REVOLUTION), 
                  Conversions.convertTalonSRXNativeUnitsToWPILibTrajecoryUnits(
                    this.rightFrontMotor.getSelectedSensorVelocity(), 
                    Settings.Drivetrain.Encoders.WHEEL_DIAMETER, 
                    true, 
                    Settings.Drivetrain.Encoders.ENCODER_PULSES_PER_REVOLUTION));
  }


  // Motor control methods
  // Stops drivetrain from moving
  public void stop() {
    differentialDrive.stopMotor();
  }

  // Drives using arcade drive
  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(speed, rotation, false);
  }

  // Drives using curvature drive algorithm
  public void curvatureDrive(double xSpeed, double zRotation, double baseTS) {
      // Clamp all inputs to valid values
      xSpeed = SLMath.clamp(xSpeed, -1.0, 1.0);
      zRotation = SLMath.clamp(zRotation, -1.0, 1.0);
      baseTS = SLMath.clamp(baseTS, 0.0, 1.0);

      // Find the amount to slow down turning by.
      // This is proportional to the speed but has a base value
      // that it starts from (allows turning in place)
      double turnAdj = Math.max(baseTS, Math.abs(xSpeed));

      // Find the speeds of the left and right wheels
      double lSpeed = xSpeed + zRotation * turnAdj;
      double rSpeed = xSpeed - zRotation * turnAdj;

      // Find the maximum output of the wheels, so that if a wheel tries to go > 1.0
      // it will be scaled down proportionally with the other wheels.
      double scale = Math.max(1.0, Math.max(Math.abs(lSpeed), Math.abs(rSpeed)));

      lSpeed /= scale;
      rSpeed /= scale;

      // Feed the inputs to the drivetrain
      differentialDrive.tankDrive(lSpeed, rSpeed, false);
  }

  // Drives using curvature drive algorithm with automatic quick turn
  public void curvatureDrive(double xSpeed, double zRotation) {
      this.curvatureDrive(xSpeed, zRotation, Settings.Drivetrain.BASE_TURNING_SPEED.get());
  }

  // Drives using WPILib Trajectory Velocity Units
  public void tankDriveVelocity(double leftVel, double rightVel){
    // System.out.println(leftVel + ","+ rightVel);  

    double leftFrontNativeVelocity = Conversions.convertWPILibTrajectoryUnitsToTalonSRXNativeUnits(
                                                  leftVel, 
                                                  Settings.Drivetrain.Encoders.WHEEL_DIAMETER, 
                                                  true, 
                                                  Settings.Drivetrain.Encoders.ENCODER_PULSES_PER_REVOLUTION);
    double rightFrontNativeVelocity = Conversions.convertWPILibTrajectoryUnitsToTalonSRXNativeUnits(
                                                    rightVel, 
                                                    Settings.Drivetrain.Encoders.WHEEL_DIAMETER, 
                                                    true, 
                                                    Settings.Drivetrain.Encoders.ENCODER_PULSES_PER_REVOLUTION);

    // Drive the motors (and their followers) to the velocity
    this.leftFrontMotor.set(ControlMode.Velocity, leftFrontNativeVelocity);
    this.rightFrontMotor.set(ControlMode.Velocity, rightFrontNativeVelocity);

    // Print the setpoint and error to the dashboard
    SmartDashboard.putNumber("Auton/Left Velocity Setpoint", leftFrontNativeVelocity);
    SmartDashboard.putNumber("Auton/Left Velocity Error", rightFrontNativeVelocity-this.leftFrontMotor.getSelectedSensorVelocity());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double leftDistance = Conversions.convertTalonEncoderTicksToMeters(leftFrontMotor.getSelectedSensorPosition(), 
                                                                              Settings.Drivetrain.Encoders.WHEEL_DIAMETER, 
                                                                              Settings.Drivetrain.Encoders.ENCODER_PULSES_PER_REVOLUTION, 
                                                                              true);
    double rightDistance = Conversions.convertTalonEncoderTicksToMeters(rightFrontMotor.getSelectedSensorPosition(), 
                                                                              Settings.Drivetrain.Encoders.WHEEL_DIAMETER, 
                                                                              Settings.Drivetrain.Encoders.ENCODER_PULSES_PER_REVOLUTION, 
                                                                              true);
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftDistance, rightDistance);

    // Create and then set current limits for motors if enabled
    if(Settings.Drivetrain.CURRENT_LIMIT.get()){
      SupplyCurrentLimitConfiguration current_limit = 
        new SupplyCurrentLimitConfiguration(
          true, 
          Settings.Drivetrain.MAX_AMP.get(), 
          Settings.Drivetrain.MAX_AMP.get() + 5, 
          5.0);

      leftFrontMotor.configSupplyCurrentLimit(current_limit);
      leftBackMotor.configSupplyCurrentLimit(current_limit);
      rightFrontMotor.configSupplyCurrentLimit(current_limit);
      rightBackMotor.configSupplyCurrentLimit(current_limit);
    }
  }

}