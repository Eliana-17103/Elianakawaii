package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;

/**
 * ===================== CONTROLS =====================
 *  GAMEPAD 1 (driver)
 *    Left stick ........ drive (field-centric)
 *    Right stick X ..... turn
 *    Right trigger ..... intake IN        Left trigger ..... intake OUT
 *    Right bumper ...... launcher (hold)  Left bumper ...... transfer (hold, only when launcher is at speed)
 *    D-pad up / down ... launcher faster / slower
 *    A ................. defense arm up/down
 *    B ................. slow mode on/off
 *    X ................. turn to face the hive
 *    Options ........... "robot is facing auto-start direction" (re-zero field-centric)
 *
 *  GAMEPAD 2 (operator)
 *    Y ................. switch Hive 1 / Hive 2
 *    Left bumper ....... transfer override (runs transfer even if launcher isn't "at speed")
 *    During INIT:  B = switch alliance,  X = use/ignore auto data
 * ====================================================
 */
@TeleOp(name = "BioBuzz Mecanum TeleOp 1 FC v1.1", group = "TeleOp")
public class BioBuzzStarterBotTeleopFieldCv11 extends LinearOpMode {

    @Override
    public void runOpMode() {

        Hubs.enableBulkReads(hardwareMap);   // faster loops (see Hubs.java)

        // ---------------- robot parts ----------------
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Defense defense = new Defense(hardwareMap);
        HiveTargeting hive = new HiveTargeting();

        Servo flowerIntake = hardwareMap.get(Servo.class, "flower_intake");

        IMU imu = GlobalState.initImu(hardwareMap);
        HeadingFrame frame = new HeadingFrame(imu);

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));   // replaced after START if auto ran

        // ---------------- one Button per physical button ----------------
        Button allianceButton = new Button(), autoDataButton = new Button();
        Button hiveButton = new Button(), faceHiveButton = new Button();
        Button slowButton = new Button(), rezeroButton = new Button(), defenseButton = new Button();
        Button speedUpButton = new Button(), speedDownButton = new Button();

        boolean useAutoData = true;

        // ---------------- INIT loop ----------------
        while (!isStarted() && !isStopRequested()) {
            if (allianceButton.pressed(gamepad2.b)) hive.toggleAlliance();
            if (autoDataButton.pressed(gamepad2.x)) useAutoData = !useAutoData;

            telemetry.addData("Alliance (G2 B)", hive.isRed() ? "RED" : "BLUE");
            telemetry.addData("Auto data available", GlobalState.autoStarted);
            telemetry.addData("Use auto data (G2 X)", useAutoData && GlobalState.autoStarted);
            if (GlobalState.autoStarted) {
                telemetry.addData("Auto turned the robot", "%.1f deg", Math.toDegrees(GlobalState.rotationDuringAuto()));
            }
            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- set up field-centric + pose ----------------
        frame.establish(useAutoData);
        if (frame.isUsingAutoData()) follower.setPose(GlobalState.endPose);

        boolean flowerIsUp = true;

        // ======================= MAIN LOOP =======================
        while (opModeIsActive()) {

            // ---- BUTTON MAP: every control is read here, once. Change a button on ONE line. ----
            double stickX = gamepad1.left_stick_x, stickY = gamepad1.left_stick_y, turnX = gamepad1.right_stick_x;
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            boolean launcherHeld = gamepad1.right_bumper;
            boolean transferHeld = gamepad1.left_bumper;
            boolean transferOverride = gamepad2.left_bumper;
            boolean flowerButton = gamepad1.cross;

            follower.update();
            launcher.update();   // battery-sag correction
            double push = FieldCentricDrive.stickPush(stickX, stickY, turnX);

            // ---- hive ----
            if (hiveButton.pressed(gamepad2.y)) hive.toggleHive();
            if (faceHiveButton.pressed(gamepad1.x)) hive.startTurn(follower);
            hive.update(follower, push);

            // ---- drive (Pedro controls the wheels while turning to the hive) ----
            if (slowButton.pressed(gamepad1.b)) drive.toggleSlowMode();
            if (rezeroButton.pressed(gamepad1.options)) frame.rezero();
            if (!hive.isTurning()) drive.drive(stickX, stickY, turnX, frame.driveHeading());

            // ---- flower mechanism ----
            /* work later
            if (flowerButton && !previousFlowerButton) {

                // Flip the state (if false -> becomes true; if true -> becomes false)
                flowerIsUp = !flowerIsUp;
                double flowerDownAngle;
              double flowerUpAngle;
                if (flowerIsUp) {
                    flowerIntake.setPosition(flowerDownAngle/300); // Move arm UP and hold
                } else {
                    flowerIntake.setPosition(flowerUpAngle/300); // Move arm DOWN and hold
                }
            }
            */

            // ---- intake ----
            intake.setServosEnabled(flowerIsUp);
            intake.setPower(intakePower);

            // ---- launcher ----
            if (speedUpButton.pressed(gamepad1.dpad_up))     launcher.bumpSetting(+1);
            if (speedDownButton.pressed(gamepad1.dpad_down)) launcher.bumpSetting(-1);
            if (launcherHeld) launcher.runAtSetting(); else launcher.halt();

            // ---- defense ----
            if (defenseButton.pressed(gamepad1.a)) defense.toggle();

            // ---- transfer: only feeds when the launcher is ready ----
            boolean launcherReady = launcher.atSpeed() || (transferOverride && launcher.isSpinning());
            if (transferHeld && launcherReady) transfer.run(); else transfer.halt();

            // ---- telemetry ----
            telemetry.addData("Field-centric from auto", frame.isUsingAutoData());
            telemetry.addData("Heading (vs auto start)", "%.1f deg", Math.toDegrees(frame.heading()));
            telemetry.addData("Pose", "%.1f, %.1f, %.0f deg", follower.pose().x(), follower.pose().y(),
                    Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Hive", hive.status());
            telemetry.addData("Slow mode", drive.isSlowMode() ? "ON" : "OFF");
            telemetry.addData("Intake", intake.status());
            telemetry.addData("Launcher", launcher.status());
            telemetry.addData("Transfer", transfer.status());
            telemetry.addData("Defense", defense.status());
            telemetry.update();
        }

        // ---------------- shutdown ----------------
        follower.stop();
        drive.halt();
        intake.halt();
        transfer.halt();
        launcher.halt();
    }
}
