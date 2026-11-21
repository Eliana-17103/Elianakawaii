package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;



@Autonomous
public class BioBuzzRedPoop2tip extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(7.1696, 32.2833, 90);
    private final Pose path1 = poseFactory.of(57.71, 20, 90);
    private final Pose path1Segment1Start = poseFactory.of(57.71, 20, 0);
    private final Pose path1Segment1End = poseFactory.of(57.71, 20, -45);
    private final Pose path1Segment2Start = poseFactory.of(57.71, 20, -45);
    private final Pose path1Segment2End = poseFactory.of(57.71, 20, 90);
    private final Pose point2 = poseFactory.of(10.3671, 8.1149, -0.2196);
    private final Pose point2Control1 = poseFactory.of(54.3244, 7.9464, 0);
    private final Pose point3 = poseFactory.of(58.6863, 123.2096, -90);
    private final Pose point3Control1 = poseFactory.of(35.7009, 40.4108, 0);
    private final Pose point3Control2 = poseFactory.of(40.6449, 72.5182, 0);
    private final Pose point3Control3 = poseFactory.of(13.1611, 108.247, 0);
    private final Pose point3Segment1Start = poseFactory.of(58.6863, 123.2096, 0);
    private final Pose point3Segment1End = poseFactory.of(58.6863, 123.2096, 0);
    private final Pose point3Segment3Start = poseFactory.of(58.6863, 123.2096, 78.1108);
    private final Pose point3Segment3End = poseFactory.of(58.6863, 123.2096, -90);
    private final Pose point4Start = poseFactory.of(58.6863, 123.2096, -90);
    private final Pose point4 = poseFactory.of(46.8299, 123.1719, -90);
    private final Pose point5Start = poseFactory.of(46.8299, 123.1719, -90);
    private final Pose point5 = poseFactory.of(46.9343, 129.6751, -90);
    private final Pose point6Start = poseFactory.of(46.9343, 129.6751, -90);
    private final Pose point6 = poseFactory.of(58.2862, 123.3466, -90);
    private final Pose point7 = poseFactory.of(12.2414, 123.617, 179.6635);

    //The things we need to shoot and intake
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private Servo flower_intake;
    private int lvelo = 1200;

    private DcMotor transfer;
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                startlauncher,
                follow(follower, path1()),
                shootAtTarget(),
                activateflower,
                follow(follower, path2()),
                intakeballs,
                waitMs(3500),
                follow(follower, path3()),
                stopintake,
                dieflower,
                shootAtTarget(),
                follow(follower, path4()),
                activateflower,
                follow(follower, path5()),
                intakeballs,
                waitMs(3500),
                follow(follower, path6()),
                stopintake,
                dieflower,
                shootAtTarget(),
                follow(follower, path7()),
                stoplauncher
        );
    }
    // Spin up launcher
    private Command startlauncher = instant(() -> {
        launcher.setVelocity(lvelo);
    });
    private Command stoplauncher = instant(() -> {
        launcher.setVelocity(0);
    });
    private Command activateflower = instant(() -> {

        flower_intake.setPosition((double) 143 /300);
    });
    private Command dieflower = instant(() -> {

        flower_intake.setPosition((double) 17 /300);
    });

    // Feed balls
    private Command feedballs = instant(() -> {
        transfer.setPower(-1.0);
    });

    private Command intakeballs = instant(() -> {
        intake.setPower(-1.0);
        leftIntakeServo.setPower(-1.0);
        rightIntakeServo.setPower(-1.0);
    });
    private Command stopintake = instant(() -> {
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    });


    // Stop launcher and windmill
    private Command stoptransfer = instant(() -> {
        transfer.setPower(0);
    });

    // Complete shooting sequence
    private Command shootAtTarget() {
        return sequential(
                feedballs,
                waitMs(2500),
                stoptransfer
        );
    }
    @Override
    public void init(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        flower_intake = hardwareMap.get(Servo.class, "flower_intake");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
    public Path path1() {
        return Paths.line(start, path1).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(path1Segment1Start, path1Segment1End)).until(1, Interpolator.linear(path1Segment2Start, path1Segment2End)));
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3Control3, point3).heading(Interpolator.piecewise().until(0.25, Interpolator.linear(point3Segment1Start, point3Segment1End)).until(0.625, Interpolator.tangent).until(1, Interpolator.linear(point3Segment3Start, point3Segment3End)));
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }
}

