package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import com.pedropathing.ivy.Command;

@Autonomous
public class BioBuzzPoopDookieMonsterRed extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    //poses
    //p.of(x, y, direction);
    private final Pose startPose = p.of(56, 9, 90);
    private final Pose shootPose = p.of(56, 20, 90);
    private final Pose chillp = p.of(17.5, 28.4, 0);
    private final Pose park = p.of(12, 106, 0);

    private final Pose park2 = p.of(12, 102, 0);

    //poses from before
    //Imports for servo and DCmotors

    //DC motor EXTENDED
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private int lvelo = 1300;

    private CRServo windmill;
    private Path shoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }
    private Path park2() {
        return line(park, park2).linear(park, park2);
    }
    private Path park() {
        return curve(shootPose, chillp, park).linear(shootPose, park).tangent();
    }
    private Command poopRoutine() {
        return sequential(
                follow(follower, shoot()),
                shootAtTarget(),
                // Add mechanism commands here.
                follow(follower, park()),
                follow(follower, park2())
        );
    }
    // Spin up launcher
    private Command startlauncher = instant(() -> {
        launcher.setVelocity(lvelo);
    });


    // Feed balls
    private Command feedballs() {
        return sequential(
                windmillCycle(), windmillCycle(), waitMs(100), windmillCycle(), windmillCycle(), windmillCycle(), windmillCycleSlow()
        );
    }
    private Command windmillCycle() {
        return sequential(
                instant(() -> windmill.setPower(-0.5)),
                waitMs(620), // Replace with calibrated 2-rotation time

                instant(() -> windmill.setPower(0)),
                waitMs(500)   // Pause between cycles
        );
    }
    private Command windmillCycleSlow() {
        return sequential(
                waitMs(1000),
                instant(() -> windmill.setPower(-0.3)),
                waitMs(1200), // Replace with calibrated 2-rotation time

                instant(() -> windmill.setPower(0))
                // Pause between cycles
        );
    }

    // Stop launcher and windmill
    private Command stopboth = instant(() -> {
        windmill.setPower(0);
        launcher.setVelocity(0);
    });

    // Complete shooting sequence
    private Command shootAtTarget() {
        return sequential(
                startlauncher,
                waitMs(2000),
                feedballs(),
                waitMs(3000),
                stopboth
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        windmill = hardwareMap.get(CRServo.class, "windmill");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    public void start() {
        schedule(poopRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
