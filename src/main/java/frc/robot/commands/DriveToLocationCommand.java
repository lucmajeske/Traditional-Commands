package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class DriveToLocationCommand extends Command {
    private CANDriveSubsystem driveSubsystem;
    private Translation2d destination;
    private Pose2d origin;
    private double doff, pdeg, driv;
    private boolean done;

    public DriveToLocationCommand(CANDriveSubsystem driveSubsystem, Translation2d destination) {
        addRequirements(driveSubsystem);
        this.driveSubsystem = driveSubsystem;
        this.destination = destination;
    }

    /**
     * Convert from range of (-PI, PI) to actual radians
     * @param rad The input in range from (-PI, PI)
     * @return The output in radians
     */
    private double toRadians(double rad) {
        if (rad < 0)
            rad += (2 * Math.PI);
        return rad;
    }

    public void initialize() {
        done = false;

        // Get our initial position
        origin = driveSubsystem.odometry.getPoseMeters();
        pdeg = toRadians(Math.atan2(
            origin.getY() - destination.getY(),
            origin.getX() - destination.getX()
        ));
        doff = (pdeg - MathUtil.angleModulus(origin.getRotation().getRadians())) > Math.PI ? -1 : 1;
        driv = origin.getTranslation().getDistance(destination);
    }

    public void execute() {
        if (doff != 0) {
            if (!MathUtil.isNear(pdeg, toRadians(MathUtil.angleModulus(driveSubsystem.odometry.getPoseMeters().getRotation().getRadians())), .3)) {
                driveSubsystem.driveArcade(0, doff * .3);
                return;
            }
            doff = 0;
        }
        if (driveSubsystem.odometry.getPoseMeters().getTranslation().getDistance(origin.getTranslation()) >= driv) {
            driveSubsystem.driveArcade(0, 0);
            done = true;
        } else
            driveSubsystem.driveArcade(.3, 0);
    }

    public boolean isFinished() {
        return done;
    }
}
