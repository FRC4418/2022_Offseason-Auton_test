// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;

public class Feeder extends SubsystemBase {

  final WPI_TalonFX feederMotor = new WPI_TalonFX(Ports.Shooter.FEEDER);
  /** Creates a new Feeder. */
  public Feeder() {
    feederMotor.configFactoryDefault();

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
    feederMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    feederMotor.setInverted(true);
    feederMotor.setNeutralMode(NeutralMode.Brake);
    resetEncoders();
  }

  
  public void setRPM(Number speed){
    feederMotor.set(ControlMode.PercentOutput, (double) speed);

  }

  public void resetEncoders() {
    feederMotor.setSelectedSensorPosition(0.d);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
