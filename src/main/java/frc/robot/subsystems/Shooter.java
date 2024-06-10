// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The elevator subsystem uses PID to go to a given height. Unfortunately, in
 * it's current state PID
 * values for simulation are different than in the real world do to minor
 * differences.
 */
public class Shooter extends SubsystemBase {
  private CANSparkMax m_flywheel = new CANSparkMax(DriveConstants.kFlywheelMotorPort, MotorType.kBrushless);
  private RelativeEncoder m_flywheelEncoder = m_flywheel.getEncoder();
  private CANSparkMax m_triggerMotor = new CANSparkMax(DriveConstants.kShooterMotorPort, MotorType.kBrushless);
  private RelativeEncoder m_triggerEncoder = m_flywheel.getEncoder();
  private double m_flywheelMotorTopSpeed; //Converts the inputted percentage
  private double m_triggerMotorTopSpeed; //Converts the inputted percentage


  public Shooter() {
    m_flywheel.restoreFactoryDefaults();
    m_flywheel.setSmartCurrentLimit(60);
    m_flywheel.setIdleMode(IdleMode.kBrake);
    m_flywheel.setInverted(false);
    m_flywheel.setOpenLoopRampRate(0.1);
    m_triggerMotor.restoreFactoryDefaults();
    m_triggerMotor.setSmartCurrentLimit(60);
    m_triggerMotor.setIdleMode(IdleMode.kBrake);
    m_triggerMotor.setInverted(false);
    m_flywheelMotorTopSpeed = ShooterConstants.kFlywheelMotorShootSpeed; 
    m_triggerMotorTopSpeed = ShooterConstants.kTriggerMotorShootSpeed; //Converts the inputted percentage

    // m_triggerMotor.setOpenLoopRampRate(0.1);
    SmartDashboard.putNumber("FW-Encoder/speed", m_flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("FW-Encoder/distance", m_flywheelEncoder.getPosition());
    SmartDashboard.putNumber("Trigger-Encoder/speed", m_flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Trigger-Encoder/distance", m_flywheelEncoder.getPosition());

    this.stop();
  }

   /**
   * Get the shooter Warmup Command
   * @return
   */
  public Command getShooterWarmupCommand(){
    return new RunCommand(this::warmup, this).withName("Warmup");
  }

  /**
   * Get the Shoot Command
   * @return
   */
  public Command getShootCommand(){
    return new RunCommand(
      this::shoot,
      this
      ).withName("Shoot");
  }

  /**
   * Get the ShootWhenReady Command
   * @return
   */
  public SequentialCommandGroup getShootWhenReadyCommands(){
    return new SequentialCommandGroup(
      this.getShooterWarmupCommand().until(this.shooterReadyTrigger()),
      this.getShootCommand()
    );
  }

    

  public Trigger shooterReadyTrigger(){
    return new Trigger(this::isFlywheelReady);
  }
  /**
   * get the Intake command
   * @return
   */
  public Command getIntakeCommand(){
    return this.startEnd(
      this::intake,
      this::stop
      ).withName("Intake");
  }

    /**
   * get the Stop command
   * @return
   */
  public Command getStopCommand(){
    return this.startEnd(
      this::stop,
      this::stop
      ).withName("Stop");
  }
  private void set(Double speed) {
    m_triggerMotor.set(speed);
    m_flywheel.set(speed);
  }

  private boolean isFlywheelReady(){
    return (Math.abs(m_flywheelEncoder.getVelocity()) >= ShooterConstants.kShooterVelocity);
  }
  private void warmup() {
    m_flywheel.set(ShooterConstants.kFlywheelMotorShootSpeed);
  }
  public double getFlywheelMotorSpeed() {
    return m_flywheel.getEncoder().getVelocity();
  }

  public double getTriggerMotorSpeed() {
    return m_triggerMotor.getEncoder().getVelocity();
  }

  private void stop() {
    m_triggerMotor.stopMotor();
    m_flywheel.stopMotor();
  }

  private void intake() {
    m_triggerMotor.set(ShooterConstants.kTriggerMotorIntakeSpeed);
    m_flywheel.set(ShooterConstants.kFlywheelMotorIntakeSpeed);
  }

  private void shoot() {
    this.setMotorSpeed(ShooterConstants.kFlywheelMotorShootSpeed, ShooterConstants.kTriggerMotorShootSpeed);
  }

  private void setMotorSpeed(double topSpeed, double bottomSpeed) {
    m_flywheel.set(topSpeed);
    m_triggerMotor.set(bottomSpeed);
  }


  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    // SmartDashboard.putData("Shooter Speed", m_pot);
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
    SmartDashboard.putNumber("FlywheelMotorSpeed", getFlywheelMotorSpeed());
    SmartDashboard.putNumber("TriggerMotorSpeed", getTriggerMotorSpeed());
  }
}
