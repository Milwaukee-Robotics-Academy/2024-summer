// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // The Drivetrain subsystem incorporates the sensors and actuators attached to
  // the robots chassis.
  // These include four drive motors, a left and right encoder and a gyro.

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = Units.inchesToMeters(21.75); // meters
  private static final double kWheelRadius = 0.0508; // meters

  private final CANSparkMax m_leftLeader = new CANSparkMax(DriveConstants.kLeftMotorPort1, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftMotorPort2, MotorType.kBrushless);
  private final CANSparkMax m_rightLeader = new CANSparkMax(DriveConstants.kRightMotorPort1, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightMotorPort2, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  private AHRS m_gyro;
  private Rotation2d m_gyroOffset = new Rotation2d();
  private final Field2d m_field = new Field2d();
  private boolean invertDriverControls = false;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      Constants.DriveConstants.kTrackWidthMeters);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
      Constants.DriveConstants.kS,
      Constants.DriveConstants.kV,
      Constants.DriveConstants.kA);
  private final SlewRateLimiter m_ForwardBackLimiter = new SlewRateLimiter(
      Constants.DriveConstants.kForwardBackSlewRate);
  private final SlewRateLimiter m_TurnLimiter = new SlewRateLimiter(Constants.DriveConstants.kTurnSlewRate);

  /** Create a new drivetrain subsystem. */
  public Drivetrain() {
    super();

    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    m_leftLeader.restoreFactoryDefaults();
    m_leftLeader.setSmartCurrentLimit(80);
    m_rightLeader.restoreFactoryDefaults();
    m_rightLeader.setSmartCurrentLimit(80);
    m_leftFollower.restoreFactoryDefaults();
    m_leftFollower.setSmartCurrentLimit(80);
    m_rightFollower.restoreFactoryDefaults();
    m_rightFollower.setSmartCurrentLimit(80);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);
    m_leftLeader.setInverted(false);
    m_gyro = new AHRS(SPI.Port.kMXP);

    m_rightLeader.setOpenLoopRampRate(0.2);
    m_leftLeader.setOpenLoopRampRate(0.2);

    double conversionFactor = 0.1524 * Math.PI / 10.71;
    //
    double velocityConversionFactor = (1 / 60.0) * (0.1524 * 2 * Math.PI) / 10.71;
    m_leftEncoder = m_leftLeader.getEncoder();
    m_leftEncoder.setPositionConversionFactor(conversionFactor);
    m_leftEncoder.setVelocityConversionFactor(velocityConversionFactor);
    m_rightEncoder = m_rightLeader.getEncoder();
    m_rightEncoder.setPositionConversionFactor(conversionFactor);
    m_rightEncoder.setVelocityConversionFactor(velocityConversionFactor);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    // Let's name the sensors on the LiveWindow
    addChild("Drive", m_drive);
    addChild("Gyro", m_gyro);
    Shuffleboard.getTab("Field").add("Field1", m_field);

    SmartDashboard.putBoolean("Invert Driver Controls", invertDriverControls);

    // Configure AutoBuilder last
    AutoBuilder.configureRamsete(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        this::tankDriveVolts, // Method that will drive the robot given ChassisSpeeds
        new ReplanningConfig(), // Default path replanning config. See the API for the options here
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Default Drive command to Drive the robot with Arcade controls
   * 
   * @param speed
   * @param rotation
   * @return
   */
  public Command getDriveCommand(DoubleSupplier speed, DoubleSupplier rotation) {
    return new RunCommand(
        () -> this.arcadeDrive(
            m_ForwardBackLimiter.calculate(-speed.getAsDouble()),
            m_TurnLimiter.calculate(-rotation.getAsDouble())),
        this).withName("ArcadeDrive");
  }

  /**
   * Invert Drivetrain command to Drive the robot with Arcade controls
   * 
   * @param speed
   * @param rotation
   * @return
   */
  public Command getInvertControlsCommand() {
    return new InstantCommand(this::invertControls).withName("InvertDriverControls");
  }

  /** The log method puts interesting information to the SmartDashboard. */
  public void log() {
    SmartDashboard.putNumber("Left Distance", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Distance", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Speed", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Gyro2", getGyroRotation2d().getDegrees());
  }

  /**
   * Tank style driving for the Drivetrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void tankDrive(double left, double right) {
    // m_drive.tankDrive(left, right);
    // Example differential drive wheel speeds: 2 meters per second
    // for the left side, 3 meters per second for the right side.
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(left * kMaxSpeed, right * kMaxSpeed);

    if (invertDriverControls){
       wheelSpeeds = new DifferentialDriveWheelSpeeds(-left * kMaxSpeed, -right * kMaxSpeed);
    }

    // Convert to chassis speeds.
    tankDriveVolts(m_kinematics.toChassisSpeeds(wheelSpeeds));
  }

  /**
   * Method to control the drivetrain using arcade drive. Arcade drive takes a
   * speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses
   * these to control the drivetrain motors
   *    
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.

   */
  public void arcadeDrive(double speed, double rotation) {

    if (invertDriverControls) {
      m_drive.arcadeDrive(-speed, -rotation);
    } else {
      m_drive.arcadeDrive(speed, rotation);
    }
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(ChassisSpeeds chassisSpeeds) {

    m_leftLeader.setVoltage(
        m_feedforward.calculate(m_kinematics.toWheelSpeeds(chassisSpeeds).leftMetersPerSecond));

    m_rightLeader.setVoltage(
        m_feedforward.calculate(m_kinematics.toWheelSpeeds(chassisSpeeds).rightMetersPerSecond));
    m_drive.feed();

  }

  private void stop() {
    this.tankDrive(0.0, 0.0);
  }

  /**
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /** Reset the robots sensors to the zero states. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading(Rotation2d rotation) {
    m_gyro.zeroYaw();
    m_gyroOffset = rotation;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading
   */
  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).minus(m_gyroOffset);
    // return m_odometry.getPoseMeters().getRotation();
  }

  public double getGyroRotation2dDegrees() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading(pose.getRotation());
    m_odometry.resetPosition(
        getGyroRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /**
   * Get the average distance of the encoders since the last reset.
   *
   * @return The distance driven (average of left and right encoders).
   */
  public double getDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2;
  }

  /**
   * Returns the ChassisSpeeds of the robot.
   *
   * @return The ChassisSpeeds.
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return m_kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Invert controls. change the forward anb back driving direction per driver
   * preference
   */
  public void invertControls() {
    invertDriverControls = !invertDriverControls;
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Invert Driver Controls", invertDriverControls);
    m_odometry.update(
        getGyroRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    log();
    m_field.setRobotPose(m_odometry.getPoseMeters());

  }
}
