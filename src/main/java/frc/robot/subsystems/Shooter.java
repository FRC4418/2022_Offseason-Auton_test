// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

public class Shooter extends SubsystemBase {

  final WPI_TalonFX topMotor = new WPI_TalonFX(Ports.Shooter.TOP);
  final WPI_TalonFX bottomMotor = new WPI_TalonFX(Ports.Shooter.BOTTOM);


  /** Creates a new Shooter. */
  public Shooter() {
    topMotor.configFactoryDefault();
		bottomMotor.configFactoryDefault();


    // Config closed-loop controls
    /*
    topMotor.config_kF(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kF, 
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);
		topMotor.config_kP(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kP,
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);
		topMotor.config_kI(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kI, 
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);
    topMotor.config_kD(Settings.Drivetrain.Motion.PID.kSlot, 
                               Settings.Drivetrain.Motion.PID.kD, 
                               Settings.Drivetrain.Motion.PID.kTimeoutMs);
    */
    topMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    topMotor.setInverted(false);
    bottomMotor.setInverted(true);
		resetEncoders();
  }

  public void setRPM(Number speed){
    topMotor.set(ControlMode.PercentOutput, (double) speed);
		bottomMotor.set(ControlMode.PercentOutput, (double) speed);
  }

  public void setRPM(Number topSpeed, Number bottomSpeed){
    topMotor.set(ControlMode.PercentOutput, (double) topSpeed);
		bottomMotor.set(ControlMode.PercentOutput, (double) bottomSpeed);
  }



  public double getTopVelocity() {
		return topMotor.getSelectedSensorVelocity();
	}

	public double getBottomVelocity() {
		return bottomMotor.getSelectedSensorVelocity();
	}

  public void resetEncoders() {
		topMotor.setSelectedSensorPosition(0.d);
		bottomMotor.setSelectedSensorPosition(0.d);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
