package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.Path;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class PathCommand extends CommandBase {

    private final Path path;

    private final Robot robot = Robot.getInstance();

    private final double speed;

    public PathCommand(Path path) {
        this.path = path;
        this.speed = 1;
    }

    public PathCommand(Path path, double speed) {
        this.path = path;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        robot.follower.setMaxPower(speed);
        robot.follower.followPath(path, false);
    }

    @Override
    public void execute() {
        robot.follower.update();
    }

    @Override
    public boolean isFinished() {
//        double xError = Math.abs(robot.getPose().getX() - path.getPath(0).getLastControlPoint().getX());
//        double yError = Math.abs(robot.getPose().getY() - path.getPath(0).getLastControlPoint().getY());
//        double headingError = Math.abs(robot.getPose().getHeading() - path.getPath(0).getHeadingGoal(1));

//        return xError < Constants.pathEndXTolerance && yError < Constants.pathEndYTolerance && headingError < Constants.pathEndHeadingTolerance;
        return !robot.follower.isBusy();
    }

}
