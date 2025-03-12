// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Class to drive the robot over CAN
public class CANDriveSubsystem extends SubsystemBase {
    private final Pigeon2 gyro;
    private final SparkFlex leftLeader, rightLeader;
    private final SparkFlex leftFollower, rightFollower;
    private final RelativeEncoder leftEncoder, rightEncoder;

    public DifferentialDriveOdometry odometry;
    public DifferentialDriveWheelPositions positions;
    public DifferentialDriveWheelSpeeds speeds;

    private final DifferentialDrive drive;
    private double gyroOffset;

    public CANDriveSubsystem() {
        gyro = new Pigeon2(DriveConstants.GYRO_ID);
        gyroOffset = -gyro.getRotation2d().getRadians();

        // create brushless motors for drive
        leftLeader = new SparkFlex(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
        leftFollower = new SparkFlex(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushless);
        rightLeader = new SparkFlex(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
        rightFollower = new SparkFlex(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();

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
        // battery). The current limit helps prevent tripping breakers.
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
        // Set config to inverted and then apply to right leader. Set right side inverted
        // so that postive values drive both sides forward
        config.inverted(true);
        rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
        positions = new DifferentialDriveWheelPositions(0, 0);
        speeds = new DifferentialDriveWheelSpeeds();
  }

    @Override
    public void periodic() {
        positions.leftMeters = leftEncoder.getPosition() * DriveConstants.METERS_PER_PULSE;
        positions.rightMeters = rightEncoder.getPosition() * DriveConstants.METERS_PER_PULSE;
        speeds.leftMetersPerSecond = leftEncoder.getVelocity() * DriveConstants.METERS_PER_PULSE;
        speeds.rightMetersPerSecond = rightEncoder.getVelocity() * DriveConstants.METERS_PER_PULSE;
        odometry.update(new Rotation2d(gyro.getRotation2d().getRadians() + gyroOffset), positions);

        SmartDashboard.putData("Gyroscope", gyro);
        SmartDashboard.putData("DriveTrain", drive);
        SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
    }

    // sets the speed of the drive motors
    public void driveArcade(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }

    public void driveTank(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed, true);
    }
}
