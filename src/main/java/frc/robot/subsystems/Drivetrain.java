// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Settings;
import frc.robot.constants.Settings.Drivetrain.Encoders;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFrontMotor.configFactoryDefault();
		leftBackMotor.configFactoryDefault();
		rightFrontMotor.configFactoryDefault();
		rightBackMotor.configFactoryDefault();

		leftBackMotor.follow(leftFrontMotor);
		rightBackMotor.follow(rightFrontMotor);

    // Config closed-loop controls
    /*
    leftFrontMotor.config_kF(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kF, 
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);
		leftFrontMotor.config_kP(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kP,
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);
		leftFrontMotor.config_kI(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kI, 
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);
    leftFrontMotor.config_kD(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kD, 
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);

    rightFrontMotor.config_kF(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kF, 
                                Settings.Drivetrain.Motion.PID.kTimeoutMs);
		rightFrontMotor.config_kP(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kP, 
                                Settings.Drivetrain.Motion.PID.kTimeoutMs);
		rightFrontMotor.config_kI(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kI, 
                                Settings.Drivetrain.Motion.PID.kTimeoutMs);
    rightFrontMotor.config_kD(Settings.Drivetrain.Motion.PID.kSlot, 
                                Settings.Drivetrain.Motion.PID.kD, 
                                Settings.Drivetrain.Motion.PID.kTimeoutMs);
		*/
    
    // Config integrated sensors (built-in encoders)
		leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		resetEncoders();

    leftGroup.setInverted(true);
		rightGroup.setInverted(false);
  }


	// Encoder methods
  public double getLeftDistance() {
		return leftFrontMotor.getSelectedSensorPosition() * Encoders.ENCODER_DISTANCE_PER_PULSE;
	}

	public double getRightDistance() {
		return rightFrontMotor.getSelectedSensorPosition() * Encoders.ENCODER_DISTANCE_PER_PULSE;
	}

	public double getAverageDistance() {
		return (getRightDistance() + getLeftDistance()) / 2.0d;
	}

	public void resetEncoders() {
		leftFrontMotor.setSelectedSensorPosition(0.d);
		rightFrontMotor.setSelectedSensorPosition(0.d);
	}


  // Stops drivetrain from moving
  public void stop() {
    differentialDrive.stopMotor();
  }

  // Drives using tank drive
  public void tankDrive(double left, double right) {
    differentialDrive.tankDrive(left, right, false);
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
      tankDrive(lSpeed, rSpeed);
  }

  // Drives using curvature drive algorithm with automatic quick turn
  public void curvatureDrive(double xSpeed, double zRotation) {
      this.curvatureDrive(xSpeed, zRotation, Settings.Drivetrain.BASE_TURNING_SPEED.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}