// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  // private final Joystick m_joystick = new Joystick(0);
  private final CommandXboxController m_driver = new CommandXboxController(0);
 // private final CommandXboxController m_operator = new CommandXboxController(1);

  private final Command m_autonomousCommand;


  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Put Some buttons on the SmartDashboard

    // Assign default commands
    // m_drivetrain.setDefaultCommand(
    // new TankDrive(() -> -m_driver.getLeftY(), () -> -m_driver.getRightY(),
    // m_drivetrain));

    /**
     * Decide if you want to use Arcade drive
     */
    m_drivetrain.setDefaultCommand(this.getDefaultDriveCommand());

       
    m_shooter.setDefaultCommand(m_shooter.getStopCommand());
 
    m_autonomousCommand =  new WaitCommand(1);//m_shooter.getShootCommand().withTimeout(2).andThen( new DriveForTime(m_drivetrain, -.5, 2));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Show what command your subsystem is running on the SmartDashboard
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_shooter);

    // Configure the button bindings
    configureButtonBindings();

        // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));
  }

  private Command getDefaultDriveCommand() {
    //return m_drivetrain.getDriveCommand(m_driver::getLeftY,m_driver::getRightX);

    return new TankDrive(m_driver::getLeftY, m_driver::getRightX, m_drivetrain);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    initializeShooterControls();

  }

  /**
   * Configures the Note shooter controls, according to the shooter buttons
   * defined in the class.
   */
  private void initializeShooterControls() {
    // Connect the buttons to commands
    m_driver.start().onTrue(m_drivetrain.getInvertControlsCommand());
    m_driver.x().whileTrue(m_shooter.getShootWhenReadyCommands());
   // m_driver.x().whileTrue(new RunCommand(() -> m_shooter.shoot(),m_shooter));
    m_driver.y().whileTrue(m_shooter.getIntakeCommand());
    m_driver.b().onTrue(m_shooter.getStopCommand());
   // m_driver.a().whileTrue(new AmpShoot(m_shooter).withTimeout(5));
    SmartDashboard.putNumber("TopShooterMotor", 100.0);
    SmartDashboard.putNumber("BottomShooterMotor", 100.0);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  // return m_autonomousCommand;
   return autoChooser.getSelected();
    // return new PathPlannerAuto("New New Auto");
  }

  public void namedCommand() {
    NamedCommands.registerCommand("Shoot",
        new SequentialCommandGroup(
            m_shooter.getShootCommand(),
            new WaitCommand(1.2),
            m_shooter.getStopCommand()));
  }

}
