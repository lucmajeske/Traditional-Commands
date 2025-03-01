// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Class to drive the robot over CAN
public class CANArmSubsystem extends SubsystemBase {
  private final SparkMax leftFront;
  private final SparkMax leftBack;
  private final SparkMax rightFront;
  private final SparkMax rightBack;

  private final DifferentialDrive drive;

  public CANArmSubsystem() {
    // create brushless motors for drive
    leftFront = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    leftBack = new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightFront = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
    rightBack = new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftFront, leftBack);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftFront.setCANTimeout(250);
    rightFront.setCANTimeout(250);
    leftBack.setCANTimeout(250);
    rightBack.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftFront);
    rightFront.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(leftBack);
    rightBack.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to left leader
    config.disableFollowerMode();
    leftBack.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to right leader. Set right side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    rightFront.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightBack.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
