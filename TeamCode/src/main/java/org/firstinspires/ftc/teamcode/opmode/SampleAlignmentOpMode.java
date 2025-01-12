package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

@Config
@TeleOp
public class SampleAlignmentOpMode extends OpMode {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID, movePID;

    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(new Pose(0, 0, 0));
        anglePID = new PIDFController(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));
        robot.follower.startTeleopDrive();
        robot.startCamera();
    }

    @Override
    public void loop() {
        double angle = robot.sampleAlignmentProcessor.getAngleToRotate();

        robot.follower.update();

        anglePID.updatePosition(0);
        anglePID.setTargetPosition(angle);
        anglePID.setCoefficients(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));


        robot.follower.setTeleOpMovementVectors(0, 0, -anglePID.runPIDF(), true);

        telemetry.addData("angle", angle);
        telemetry.addData("pid error", anglePID.getError());
        telemetry.addData("pid val", anglePID.runPIDF());

        telemetry.update();
    }
}
