package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class DriveToLocationCommand extends Command {
    private CANDriveSubsystem driveSubsystem;
    private Pose2d destination;
    private boolean done;

    public DriveToLocationCommand(CANDriveSubsystem driveSubsystem, Pose2d destination) {
        addRequirements(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.destination = destination;
    }

    public void initialize() {
        done = false;
        driveSubsystem.driveArcade(5,0);
        Pose2d rob = driveSubsystem.odometry.getPoseMeters();
        double pdeg = MathUtil.angleModulus(Math.atan2(
            rob.getY() - destination.getY(),
            rob.getX() - destination.getX()
        ) + rob.getRotation().getRadians());
        double toff = (pdeg / Math.abs(pdeg));
        pdeg = MathUtil.angleModulus(pdeg - Math.PI);
        driveSubsystem.driveArcade(0, toff * .4);
        while (driveSubsystem.odometry.getPoseMeters().getRotation().getRadians() * toff >= pdeg) {}
    }

    public void execute() {
        if (driveSubsystem.odometry.getPoseMeters().getTranslation().getDistance(destination.getTranslation()) <= 0)
            done = true;
    }

    public boolean isFinished() {
        return done;
    }
}
