package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;

@Autonomous
public class Straff extends OpMode {
    Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(131.1297, 9.3476, 180);
    private final Pose path1 = poseFactory.of(130.9625, 22.1869, 180);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }
    Path thisIs;

    @Override
    public void init() {
       follower = Constants.create(hardwareMap);
       follower.setPose(start);

       thisIs =  path(line(start, path1).linear(start, path1));
    }
    @Override
    public void start() {

    }

    int step = 0;
    @Override
    public void loop() {
        follower.update();
        switch(step) {
            case -1:

            case 0:
                follower.follow(thisIs);
                step++;
                break;
            case 1:
                if(!follower.isBusy()) {
                    step = -1;
                }
        }
    }
}