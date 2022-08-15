// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Settings;
import frc.robot.subsystems.Feeder;

public class FeederControl extends CommandBase {

  private final Feeder feeder;
  private final Gamepad driver;
  private final IStream commandedSpeed;
  SmartNumber CURRENT_COMMAND_VALUE = new SmartNumber("Feeder current command value", 0);

  /** Creates a new FeederControl. */
  public FeederControl(Feeder feeder, Gamepad driver) {
    this.feeder = feeder;
    this.driver = driver;

    this.commandedSpeed =
        IStream.create(() -> driver.getRightTrigger())
                .filtered(
                        x -> SLMath.map(x, 0, 1, Settings.Feeder.MIN_SPEED.get(), 
                                        Settings.Feeder.MAX_SPEED.get()
                                        ),
                        x -> SLMath.spow(x, Settings.Feeder.SPEED_POWER.get()),
                        new LowPassFilter(Settings.Feeder.SPEED_FILTER)
                        );
    
    addRequirements(feeder);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("feeder initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.setRPM(commandedSpeed.get());
    CURRENT_COMMAND_VALUE.set(commandedSpeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
