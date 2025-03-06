// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import org.ejml.dense.row.misc.RrefGaussJordanRowPivot_DDRM;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Class to drive the robot over CAN
public class CANDriveSubsystem extends SubsystemBase {
  private final Pigeon2 gyro;
  private final SparkFlex leftLeader;
  private final SparkFlex leftFollower;
  private final SparkFlex rightLeader;
  private final SparkFlex rightFollower;

  public DifferentialDriveOdometry odometry;
  public DifferentialDriveWheelPositions positions;
  public DifferentialDriveWheelSpeeds speeds;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    gyro = new Pigeon2(DriveConstants.GYRO_ID);

    // create brushless motors for drive
    leftLeader = new SparkFlex(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkFlex(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkFlex(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkFlex(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkFlexConfig config = new SparkFlexConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to left leader
    config.disableFollowerMode();
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to right leader. Set right side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
  }

  @Override
  public void periodic() {
    positions = new DifferentialDriveWheelPositions(
      leftLeader.getAbsoluteEncoder().getPosition() * DriveConstants.COUNTS_PER_REVOLUTION,
      rightLeader.getAbsoluteEncoder().getPosition() * DriveConstants.COUNTS_PER_REVOLUTION
    );
    speeds = new DifferentialDriveWheelSpeeds(
      leftLeader.getAbsoluteEncoder().getVelocity() * DriveConstants.COUNTS_PER_REVOLUTION,
      rightLeader.getAbsoluteEncoder().getVelocity() * DriveConstants.COUNTS_PER_REVOLUTION
    );
    odometry.update(gyro.getRotation2d(), positions);
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
