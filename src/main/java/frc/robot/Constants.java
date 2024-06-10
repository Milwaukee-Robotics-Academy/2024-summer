// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static final int kLeftClimberMotorPort = 9;
  public static final int kRightClimberMotorPort = 10;

  public static final class DriveConstants {
    public static final int kLeftMotorPort1 = 1;
    public static final int kLeftMotorPort2 = 2;

    public static final int kRightMotorPort1 = 3;
    public static final int kRightMotorPort2 = 4;

    public static final int kFlywheelMotorPort = 6;
    public static final int kShooterMotorPort = 5;

    public static final int kFlywheelLeftMotorPort = 8 ;
    public static final int kShooterLeftMotorPort = 7;

    public static final int[] kLeftEncoderPorts = {0, 1};
    public static final int[] kRightEncoderPorts = {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = false;

    public static final int kRangeFinderPort = 6;
    public static final int kAnalogGyroPort = 1;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final int kForwardBackSlewRate = 3;
    public static final int kTurnSlewRate = 2;
    public static final double kTrackWidthMeters = 0.53;

    public static final double kS = 0.11673; // copied from the features/pathplanner branch
    public static final double kV = 1.3815;
    public static final double kA = 0.12011;
    public static final double kP =0.7;
    public static final double kD =0.0;
  }

   public static final class ShooterConstants {

    public static double kFlywheelMotorShootSpeed = 1.0;
    public static double kTriggerMotorShootSpeed = 1.0;
    public static double kFlywheelMotorIntakeSpeed = -0.25;
    public static double kTriggerMotorIntakeSpeed = -0.25;
    public static double kShooterVelocity = 5000;

   }
}
