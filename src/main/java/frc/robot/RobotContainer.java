// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToLocationCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;
import frc.robot.subsystems.CANWristSubsystem;
import frc.robot.subsystems.CANArmSubsystem;

public class RobotContainer {
    // The robot's subsystems
    public final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
    public final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();
    public final CANArmSubsystem armSubsystem = new CANArmSubsystem();
    public final CANWristSubsystem wristSubsystem = new CANWristSubsystem();

    // The driver's controller
    private final CommandJoystick driverLeftController = new CommandJoystick(OperatorConstants.DRIVER_LEFT_CONTROLLER_PORT);
    private final CommandJoystick driverRightController = new CommandJoystick(OperatorConstants.DRIVER_RIGHT_CONTROLLER_PORT);

    // The operator's controller
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    // The autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up command bindings
        configureBindings();

        // Set the options to show up in the Dashboard for selecting auto modes. If you
        // add additional auto modes you can add additional lines here with
        // autoChooser.addOption
        autoChooser.setDefaultOption("Go Forward X", new AutoCommand(driveSubsystem)); {
            driveSubsystem.driveTank(0.25,0.25);

        };
      
        autoChooser.addOption("Autonomous", new AutoCommand(driveSubsystem));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Set the default command for the drive subsystem to an instance of the
        // DriveCommand with the values provided by the joystick axes on the driver
        // controller. The Y axis of the controller is inverted so that pushing the
        // stick away from you (a negative value) drives the robot forwards (a positive
        // value). Similarly for the X axis where we need to flip the value so the
        // joystick matches the WPILib convention of counter-clockwise positive
        driveSubsystem.setDefaultCommand(new DriveCommand(
            () -> -driverLeftController.getY(),
            () -> -driverRightController.getY(),
            driveSubsystem));

        // Set the default command for the roller subsystem to an instance of
        // RollerCommand with the values provided by the triggers on the operator
        // controller
        rollerSubsystem.setDefaultCommand(new RollerCommand(
            () -> 0.25 * (operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()),
            rollerSubsystem));

        //set the armSubsystem to move on a normalized value of the joysticks
        // armSubsystem.setDefaultCommand(new ArmCommand(
        // () -> operatorController.getRightTriggerAxis(),
        // () -> operatorController.getLeftTriggerAxis(),
        // armSubsystem));

        // Bind commands to buttons
        operatorController.button(1).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_ONE_HOME), armSubsystem));
        operatorController.button(2).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_TWO_INTAKE), armSubsystem));
        operatorController.button(3).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_THREE_ALGAE_DEPOSIT), armSubsystem));
        operatorController.button(4).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_FOUR_CORAL_LOW), armSubsystem));
        operatorController.button(5).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_FIVE_CORAL_HIGH), armSubsystem));
        operatorController.button(6).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_SIX_ALGAE_LOW), armSubsystem));
        operatorController.button(7).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_SEVEN_ALGAE_HIGH), armSubsystem));
        operatorController.button(8).onTrue(Commands.runOnce(() -> armSubsystem.setArmGoal(ArmConstants.GOAL_EIGHT_BARGE_SHOOT), armSubsystem));
      
        operatorController.button(1).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_ONE_HOME), wristSubsystem));
        operatorController.button(2).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_TWO_INTAKE), wristSubsystem));
        operatorController.button(3).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_THREE_ALGAE_DEPOSIT), wristSubsystem));
        operatorController.button(4).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_FOUR_CORAL_LOW), wristSubsystem));
        operatorController.button(5).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_FIVE_CORAL_HIGH), wristSubsystem));
        operatorController.button(6).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_SIX_ALGAE_LOW), wristSubsystem));
        operatorController.button(7).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_SEVEN_ALGAE_HIGH), wristSubsystem));
        operatorController.button(8).onTrue(Commands.runOnce(() -> wristSubsystem.setWristGoal(WristConstants.GOAL_EIGHT_BARGE_SHOOT), wristSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
