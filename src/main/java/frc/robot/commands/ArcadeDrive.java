// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends Command {

  DoubleSupplier speedSupplier;
  DoubleSupplier angleSUpplier;
  Drivetrain m_drivetrain;
  
  /** Creates a new ArcadeDrive. 
   *
   * @param speed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param angle The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   * @param drivetrain the subsystem used.
   */
  public ArcadeDrive(DoubleSupplier speed, DoubleSupplier angle, Drivetrain drivetrain) {

    speedSupplier = speed;
    angleSUpplier = angle;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrain.arcadeDrive(speedSupplier.getAsDouble(), angleSUpplier.getAsDouble());
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
