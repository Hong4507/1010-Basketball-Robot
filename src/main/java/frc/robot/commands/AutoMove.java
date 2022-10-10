// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutoMove extends CommandBase {

  private final Drive m_drive;

  private double initTime;
  private double elapsedTime;
  private double Time1;
  private double Time2;
  
  /** Creates a new AutoMove. */
  public AutoMove(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.motorSet2Zero();
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // 20ms -> 1s = 50tick
  @Override
  public void execute() {
    elapsedTime = Timer.getFPGATimestamp();

    // Go straight for 3s, stop, Turn for 1s.
    for (int i = 0 ; i < 151 ; i++) {
      m_drive.arcadeDrive(0.3, 0);
    }

    m_drive.motorSet2Zero();

    for (int i = 0 ; i < 51 ; i++) {
      m_drive.arcadeDrive(0, 0.5);
    }

    m_drive.motorSet2Zero();
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
