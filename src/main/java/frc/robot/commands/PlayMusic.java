// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class PlayMusic extends Command {

    private Drivebase drivebase;

    private Orchestra orchestra = new Orchestra();
    private AudioConfigs audioConfigs = new AudioConfigs();
  /** Creates a new Drive. */
  public PlayMusic(Drivebase drivebase) {
    this.drivebase = drivebase;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    List<TalonFX> motors = drivebase.getInstruments();
    int[] tracks = {1, 2, 3, 4, 5, 6, 18, 17};
    for (int i = 0; i < tracks.length; i++)
    {
      orchestra.addInstrument(motors.get(i), tracks[i]);
    }
    
    SmartDashboard.putBoolean("load chrp file", orchestra.loadMusic("starwars3.chrp").isOK());
    SmartDashboard.putBoolean("audio enabled", audioConfigs.withAllowMusicDurDisable(true).AllowMusicDurDisable);
    SmartDashboard.putBoolean("orhcestra play no errors", orchestra.play().isOK());
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("orhcestra playing", orchestra.isPlaying());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orchestra.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
