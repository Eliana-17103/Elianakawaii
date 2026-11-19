package org.firstinspires.ftc.teamcode.biobuzz_modular;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;

/**
 * Controls (gamepad1): sticks = field-centric drive | B = slow mode | X = face hive |
 *   triggers = intake/outtake | right bumper = launcher | left bumper = transfer |
 *   A = defense | D-pad up/down = launcher speed | Options = re-zero heading
 * Gamepad2: B = switch alliance (init) | X = use/ignore auto data (init) | Y = switch hive |
 *   left bumper = transfer override
 */
@TeleOp(name = "BioBuzz Mecanum TeleOp 1 FC v1.1", group = "TeleOp")
public class BioBuzzStarterBotTeleopFieldCv11 extends LinearOpMode {

    // ---- settings ----
    // false: hold left bumper to run transfer (only while launcher is at speed)
    // true : tap left bumper once the launcher is at speed to run transfer for Transfer.RUN_MS
    private static final boolean TIMED_TRANSFER = false;

    @Override
    public void runOpMode() {

        // ---------------- subsystems ----------------
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
        follower.setPose(new Pose(0, 0, 0));   // replaced below once we know whether auto ran

        // ---------------- edge detectors ----------------
        Edge eAlliance = new Edge(), eCarry = new Edge(), eHive = new Edge(), eFace = new Edge(),
                eSlow = new Edge(), eOptions = new Edge(), eDefense = new Edge(),
                eUp = new Edge(), eDown = new Edge(), eTransfer = new Edge();

        boolean useAutoData = true;

        // ---------------- init loop ----------------
        while (!isStarted() && !isStopRequested()) {
            if (eAlliance.rising(gamepad2.b)) hive.toggleAlliance();
            if (eCarry.rising(gamepad2.x)) useAutoData = !useAutoData;

            boolean dataOk = HeadingFrame.autoDataAvailable();
            telemetry.addData("Alliance", hive.isRed() ? "RED" : "BLUE");
            telemetry.addData("Auto data available", dataOk);
            telemetry.addData("Using auto data (G2 X)", useAutoData && dataOk);
            if (dataOk) {
                telemetry.addData("Auto rotation (IMU)", "%.1f deg", Math.toDegrees(GlobalState.imuRotationDuringAuto()));
                telemetry.addData("Auto rotation (Pedro)", "%.1f deg", Math.toDegrees(GlobalState.pedroRotationDuringAuto()));
            }
            telemetry.addLine("Gamepad 2 B: Switch Alliance | Gamepad 2 X: use/ignore auto data");
            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- establish heading frame + pose ----------------
        frame.establish(useAutoData);
        if (frame.isCarriedFromAuto()) follower.setPose(GlobalState.endPose);
        follower.update();

        boolean flowerIsUp = true;
        boolean wasAtSpeed = false;

        // =====================================================
        // MAIN LOOP
        // =====================================================
        while (opModeIsActive()) {

            follower.update();
            launcher.update();
            transfer.update();
            Pose pose = follower.pose();

            // ---------- hive ----------
            if (eHive.rising(gamepad2.y)) hive.toggleHive();
            if (eFace.rising(gamepad1.x)) hive.startTurn(follower);
            hive.update(follower, FieldCentricDrive.stickMagnitude(gamepad1));

            // ---------- drive (Pedro owns the drivetrain while turning to the hive) ----------
            if (eSlow.rising(gamepad1.b)) drive.toggleSlowMode();
            if (eOptions.rising(gamepad1.options)) frame.rezero();
            if (!hive.isTurning()) drive.drive(gamepad1, frame.driveHeading());

            // ---------- flower mechanism ----------
            boolean flowerButton = gamepad1.cross;
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

            // ---------- intake ----------
            intake.setServosEnabled(flowerIsUp);
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // ---------- launcher ----------
            if (eUp.rising(gamepad1.dpad_up))     launcher.bumpSetting(+1);
            if (eDown.rising(gamepad1.dpad_down)) launcher.bumpSetting(-1);

            if (gamepad1.right_bumper) launcher.runAtSetting(); else launcher.stop();

            boolean atSpeed = gamepad1.right_bumper && launcher.atSpeed();
            if (atSpeed && !wasAtSpeed) gamepad1.rumble(200);   // buzz when ready to feed
            wasAtSpeed = atSpeed;

            // ---------- defense ----------
            if (eDefense.rising(gamepad1.a)) defense.toggle();

            // ---------- transfer ----------
            boolean transferButton = gamepad1.left_bumper;
            boolean ready = launcher.atSpeed() || (gamepad2.left_bumper && launcher.isSpinning());
            if (TIMED_TRANSFER) {
                if (eTransfer.rising(transferButton) && ready) transfer.startTimed();
            } else {
                if (transferButton && ready) transfer.run(); else transfer.stop();
            }

            // ---------- telemetry ----------
            frame.addTelemetry(telemetry);
            telemetry.addData("Pedro X / Y / heading", "%.1f / %.1f / %.1f deg",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            hive.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            telemetry.addLine("===== MECHANISMS =====");
            intake.addTelemetry(telemetry);
            transfer.addTelemetry(telemetry);
            defense.addTelemetry(telemetry);
            launcher.addTelemetry(telemetry);
            telemetry.update();
        }

        // ---------------- shutdown ----------------
        follower.stop();
        drive.stop();
        intake.stop();
        transfer.stop();
        launcher.stop();
    }
}
