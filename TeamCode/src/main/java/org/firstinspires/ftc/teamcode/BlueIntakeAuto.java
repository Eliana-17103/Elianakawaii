
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Blue Intake Auto", group = "Autonomous")
public class BlueIntakeAuto extends OpMode {

    // ==========================================
    // 1. PEDROPATHING SETUP
    // ==========================================

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();


    // ==========================================
    // 2. POSES
    // ==========================================

    private final Pose startPose =
            poseFactory.of(9.1702, 131.9809, 0);

    private final Pose intakePose =
            poseFactory.of(46.3994, 129.1542, 90);


    // ==========================================
    // 3. HARDWARE
    // ==========================================

    private DcMotor intake;

    // VARIABLES
    protected int lvelo = 1250;


    // ==========================================
    // 4. PATHS
    // ==========================================

    private Path path1() {
        return line(startPose, intakePose)
                .linear(startPose, intakePose);
    }


    // ==========================================
    // 5. CUSTOM COMMANDS
    // ==========================================

    // Start intake motor
    private Command startIntake() {
        return instant(() -> {
            intake.setPower(1.0);
        });
    }

    // Stop intake motor
    private Command stopIntake() {
        return instant(() -> {
            intake.setPower(0);
        });
    }
    private Command stupidahh() {
        return instant(() -> {
            lvelo = 1300;
        });
    }


    // ==========================================
    // 6. AUTONOMOUS ROUTINE
    // ==========================================

    private Command autonomousRoutine() {
        return sequential(

                // Follow path to intake position
                follow(follower, path1()),

                // Activate intake
                startIntake()

        );
    }


    // ==========================================
    // 7. INITIALIZATION
    // ==========================================

    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        intake.setPower(0);
    }


    // ==========================================
    // 8. START AUTONOMOUS
    // ==========================================

    @Override
    public void start() {
        schedule(autonomousRoutine());
    }


    // ==========================================
    // 9. MAIN LOOP
    // ==========================================

    @Override
    public void loop() {

        follower.update();

        Scheduler.execute();
    }


    // ==========================================
    // 10. CLEANUP
    // ==========================================

    @Override
    public void stop() {

        intake.setPower(0);
    }
}
