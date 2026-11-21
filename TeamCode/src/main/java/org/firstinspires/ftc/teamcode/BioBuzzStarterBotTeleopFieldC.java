
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;


@TeleOp(name = "BioBuzz Mecanum TeleOp 1 FC", group = "TeleOp")
public class BioBuzzStarterBotTeleopFieldC extends LinearOpMode {


    // =========================================================
    // 1. HARDWARE DECLARATIONS
    // =========================================================

    // Four mecanum drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // PedroPathing follower handles localization and autonomous
    // drivetrain commands, including the hive-facing rotation.
    private Follower follower;

    // Stores the robot's current estimated position and heading.
    private Pose currentPose;

    // Mechanism motors and servos
    private DcMotor intake;
    private DcMotorEx launcher;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private Servo defense;
    private Servo flower_intake;

    private DcMotor transfer;

    // IMU used for field-centric driving
    private IMU imu;


    // =========================================================
    // 2. LAUNCHER SETTINGS
    // =========================================================

    // Initial launcher velocity
    private double launcherVelocity = 1210;

    // Minimum and maximum allowed launcher velocities
    private static final double MIN_LAUNCHER_VELOCITY = 0;
    private static final double MAX_LAUNCHER_VELOCITY = 2500;

    // Amount to increase/decrease launcher velocity per D-pad press
    private static final double LAUNCHER_INCREMENT = 10;


    // =========================================================
    // 3. DRIVE SETTINGS
    // =========================================================

    // Ignores small joystick movements to prevent unwanted drifting
    private static final double JOYSTICK_DEADZONE = 0.05;

    // Normal and reduced driving speeds
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;


    // =========================================================
    // 4. ALLIANCE HIVE COORDINATES
    // =========================================================

    // -------------------------
    // BLUE ALLIANCE
    // -------------------------

    // Blue Hive 1
    private static final double BLUEHIVE1_X = 84;
    private static final double BLUEHIVE1_Y = 86;

    // Blue Hive 2
    private static final double BLUEHIVE2_X = 84;
    private static final double BLUEHIVE2_Y = 55;


    // -------------------------
    // RED ALLIANCE
    // -------------------------

    // IMPORTANT:
    // These are placeholder coordinates.
    // Replace them with your actual Red alliance field coordinates.

    // Red Hive 1
    private static final double REDHIVE1_X = 57.5;
    private static final double REDHIVE1_Y = 60;

    // Red Hive 2
    private static final double REDHIVE2_X = 57.5;
    private static final double REDHIVE2_Y = 86;


    // =========================================================
    // 5. ALLIANCE AND HIVE SELECTION VARIABLES
    // =========================================================

    // Determines which alliance is selected.
    // false = Blue
    // true  = Red
    //
    // Blue is selected by default.
    private boolean isRedAlliance = false;

    // Determines which hive is selected.
    // false = Hive 1
    // true  = Hive 2
    private boolean alternateHiveTarget = false;

    // These are the coordinates currently used by faceHive().
    private double hiveX = BLUEHIVE1_X;
    private double hiveY = BLUEHIVE1_Y;


    // =========================================================
    // 6. HIVE TURNING VARIABLES
    // =========================================================

    // True while Pedro is turning the robot toward the hive.
    private boolean turningToHive = false;

    // Desired heading calculated from the robot's position
    // and the selected hive coordinates.
    private double targetHeading = 0;

    // Difference between the target heading and current heading
    private double headingError = 0;

    // The robot must get within 2 degrees of its target heading.
    private static final double HEADING_TOLERANCE =
            Math.toRadians(2);


    // =========================================================
    // 7. HIVE COORDINATE UPDATE METHOD
    // =========================================================

    /**
     * Updates hiveX and hiveY based on the currently selected
     * alliance and hive number.
     *
     * This method ensures that the robot always uses the
     * coordinates belonging to the selected alliance.
     *
     * Examples:
     * Blue + Hive 1 = BLUEHIVE1 coordinates
     * Blue + Hive 2 = BLUEHIVE2 coordinates
     * Red  + Hive 1 = REDHIVE1 coordinates
     * Red  + Hive 2 = REDHIVE2 coordinates
     */
    private void updateHiveCoordinates() {

        // If Red alliance is selected
        if (isRedAlliance) {

            // Select between the two Red hive positions
            if (alternateHiveTarget) {

                hiveX = REDHIVE2_X;
                hiveY = REDHIVE2_Y;

            } else {

                hiveX = REDHIVE1_X;
                hiveY = REDHIVE1_Y;
            }

        } else {

            // Otherwise, Blue alliance is selected.
            // Select between the two Blue hive positions.
            if (alternateHiveTarget) {

                hiveX = BLUEHIVE2_X;
                hiveY = BLUEHIVE2_Y;

            } else {

                hiveX = BLUEHIVE1_X;
                hiveY = BLUEHIVE1_Y;
            }
        }
    }


    // =========================================================
    // 8. HIVE SWITCHING METHOD
    // =========================================================

    /**
     * Switches between Hive 1 and Hive 2.
     *
     * The selected alliance remains unchanged.
     *
     * Gamepad 2 Y calls this method during the match.
     */
    private void toggleHiveCoordinates() {

        // Switch the selected hive
        alternateHiveTarget = !alternateHiveTarget;

        // Load the corresponding coordinates for that hive
        updateHiveCoordinates();
    }


    // =========================================================
    // 9. HIVE-FACING METHOD
    // =========================================================

    /**
     * Calculates the heading from the robot's current
     * PedroPathing position toward the selected hive.
     *
     * Uses:
     * - Current robot X and Y coordinates
     * - Selected hive X and Y coordinates
     * - Math.atan2() to calculate the desired heading
     *
     * Pedro's hold() method then attempts to rotate the
     * robot toward that heading while maintaining position.
     */
    private void faceHive() {

        // Retrieve the latest robot position
        currentPose = follower.pose();

        double x = currentPose.x();
        double y = currentPose.y();

        // Calculate the angle from the robot to the hive
        targetHeading = Math.atan2(
                hiveY - y,
                hiveX - x
        );

        // Command Pedro to hold the current position
        // while rotating toward the calculated heading.
        follower.hold(new Pose(x, y, targetHeading));

        // Mark the robot as actively turning
        turningToHive = true;
    }


    // =========================================================
    // 10. MAIN OP MODE
    // =========================================================

    @Override
    public void runOpMode() {


        // =====================================================
        // A. HARDWARE MAPPING
        // =====================================================

        // Map all four drive motors
        leftFront = hardwareMap.get(
                DcMotor.class, "left_front_drive");

        leftBack = hardwareMap.get(
                DcMotor.class, "left_back_drive");

        rightFront = hardwareMap.get(
                DcMotor.class, "right_front_drive");

        rightBack = hardwareMap.get(
                DcMotor.class, "right_back_drive");


        // Map intake and launcher motors
        intake = hardwareMap.get(
                DcMotor.class, "intake");

        launcher = hardwareMap.get(
                DcMotorEx.class, "launcher");


        // Map continuous rotation intake servos
        leftIntakeServo = hardwareMap.get(
                CRServo.class, "left_intake_servo");

        rightIntakeServo = hardwareMap.get(
                CRServo.class, "right_intake_servo");

        defense = hardwareMap.get(
                Servo.class, "defense");
        flower_intake = hardwareMap.get(
                Servo.class, "flower_intake");


        // Map transfer motor
        transfer = hardwareMap.get(
                DcMotor.class, "transfer");


        // Map IMU
        imu = hardwareMap.get(
                IMU.class, "imu");


        // =====================================================
        // B. PEDROPATHING INITIALIZATION
        // =====================================================

        // Create the Pedro follower using your existing constants
        follower = Constants.create(hardwareMap);

        // Set the robot's initial Pedro position.
        // Make sure this matches your actual starting position.
        follower.setPose(new Pose(0, 0, 0));


        // =====================================================
        // C. IMU INITIALIZATION
        // =====================================================

        // Configure the IMU based on the physical orientation
        // of your Control Hub:
        //
        // REV logo faces backward
        // USB port faces upward

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);


        // =====================================================
        // D. DRIVE MOTOR CONFIGURATION
        // =====================================================

        // Configure motor directions for mecanum driving
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        // Apply braking when no power is being sent
        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);


        // =====================================================
        // E. LAUNCHER CONFIGURATION
        // =====================================================

        // Allow launcher to coast when stopped
        launcher.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        // Enable encoder-based velocity control
        launcher.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure launcher PIDF coefficients
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );


        // =====================================================
        // F. SERVO CONFIGURATION
        // =====================================================

        // Reverse the right intake servo so both intake
        // sides rotate in the correct direction together
        rightIntakeServo.setDirection(
                DcMotor.Direction.REVERSE);


        // =====================================================
        // G. ALLIANCE SELECTION DURING INIT
        // =====================================================

        // Blue alliance is selected by default
        isRedAlliance = false;

        // Start with Hive 1 selected
        alternateHiveTarget = false;

        // Load the initial Blue Hive 1 coordinates
        updateHiveCoordinates();


        // Tracks the previous state of Gamepad 2 B.
        // This prevents holding B from repeatedly switching
        // the selected alliance.
        boolean previousAllianceButton = false;


        // Continue checking alliance selection until START
        // is pressed or the OpMode is stopped.
        //
        // This loop happens BEFORE the actual match loop.

        while (!isStarted() && !isStopRequested()) {

            // Read Gamepad 2 B
            boolean allianceButton = gamepad2.b;

            // Detect a new B-button press
            if (allianceButton && !previousAllianceButton) {

                // Switch between Red and Blue
                isRedAlliance = !isRedAlliance;

                // Reset hive selection to Hive 1
                alternateHiveTarget = false;

                // Load coordinates for the newly selected alliance
                updateHiveCoordinates();
            }

            // Remember the current button state
            previousAllianceButton = allianceButton;


            // Display the current alliance selection
            telemetry.addLine("===== ALLIANCE SELECTION =====");

            telemetry.addData(
                    "Selected Alliance",
                    isRedAlliance ? "RED" : "BLUE"
            );

            telemetry.addData(
                    "Selected Hive",
                    alternateHiveTarget ? "Hive 2" : "Hive 1"
            );

            telemetry.addData(
                    "Hive X",
                    "%.2f",
                    hiveX
            );

            telemetry.addData(
                    "Hive Y",
                    "%.2f",
                    hiveY
            );

            telemetry.addLine("");
            telemetry.addLine("Gamepad 2 B: Switch Alliance");
            telemetry.addLine("Press START when ready");

            telemetry.update();

            // Allow the OpMode to yield while waiting
            idle();
        }


        // =====================================================
        // H. WAIT FOR START
        // =====================================================

        // The initialization loop ends when START is pressed.
        waitForStart();

        if (isStopRequested()) return;

        // Reset IMU yaw at the beginning of the match
        imu.resetYaw();


        // =====================================================
        // I. BUTTON STATE VARIABLES
        // =====================================================

        // These variables store the previous loop's button states.
        // They allow us to detect a new button press rather
        // than repeatedly activating while a button is held.

        boolean previousDpadUp = false;
        boolean previousDpadDown = false;

        boolean previousOptions = false;
        boolean previousBButton = false;

        // Gamepad 1 X: face selected hive
        boolean previousXButton = false;

        // Gamepad 2 Y: switch selected hive
        boolean previousYButton = false;

        boolean armIsUp = false;     // Tracks whether the arm is currently up or down
        boolean flowerIsDown=false;
        boolean previousAButton = false;

        // Slow mode starts disabled
        boolean slowMode = false;


        // =====================================================
        // J. MAIN CONTROL LOOP
        // =====================================================

        while (opModeIsActive()) {


            // -------------------------------------------------
            // 1. PEDRO LOCALIZATION
            // -------------------------------------------------

            // Update Pedro's localization every loop
            follower.update();

            // Retrieve the latest robot pose
            currentPose = follower.pose();


            // -------------------------------------------------
            // 2. HIVE SWITCHING
            // -------------------------------------------------

            // Read Gamepad 2 Y
            boolean yButton = gamepad2.y;

            // Switch between Hive 1 and Hive 2 only when
            // Y is newly pressed.
            //
            // Do not switch targets during an active turn.
            if (yButton && !previousYButton && !turningToHive) {

                toggleHiveCoordinates();
            }

            // Save current button state
            previousYButton = yButton;


            // -------------------------------------------------
            // 3. HIVE-FACING BUTTON
            // -------------------------------------------------

            // Read Gamepad 1 X
            boolean xButton = gamepad1.x;

            // Start turning when X is newly pressed
            // and no other hive turn is active.
            if (xButton && !previousXButton && !turningToHive) {

                faceHive();
            }

            // Save current button state
            previousXButton = xButton;


            // -------------------------------------------------
            // 4. HIVE TURN COMPLETION
            // -------------------------------------------------

            if (turningToHive) {

                // Retrieve the latest Pedro pose
                currentPose = follower.pose();

                // Calculate the shortest angular difference
                // between the robot's current heading and target.
                //
                // atan2(sin, cos) keeps the error between
                // -PI and +PI, preventing unnecessary full turns.

                headingError = Math.atan2(
                        Math.sin(targetHeading - currentPose.heading()),
                        Math.cos(targetHeading - currentPose.heading())
                );

                // Check whether the robot is within the
                // allowed heading tolerance.
                if (Math.abs(headingError) < HEADING_TOLERANCE) {

                    // Mark the turn as complete
                    turningToHive = false;

                    // Stop Pedro's hold operation
                    follower.stop();
                }
            }


            // -------------------------------------------------
            // 5. FIELD-CENTRIC DRIVE INPUT
            // -------------------------------------------------

            // Left joystick controls translation
            double y = -gamepad1.left_stick_y;

            // Slightly increase horizontal movement sensitivity
            double x = gamepad1.left_stick_x * 1.1;

            // Right joystick controls rotation
            double rx = -gamepad1.right_stick_x;


            // Apply joystick deadzones
            if (Math.abs(y) < JOYSTICK_DEADZONE) y = 0;
            if (Math.abs(x) < JOYSTICK_DEADZONE) x = 0;
            if (Math.abs(rx) < JOYSTICK_DEADZONE) rx = 0;


            // -------------------------------------------------
            // 6. FIELD-CENTRIC HEADING TRANSFORMATION
            // -------------------------------------------------

            // Get IMU heading in radians
            double heading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            // Add 180 degrees so the robot's backward side
            // is treated as its front.
            double driveHeading = heading + Math.PI;


            // Rotate joystick inputs based on robot orientation
            double rotX = x * Math.cos(-driveHeading)
                    - y * Math.sin(-driveHeading);

            double rotY = x * Math.sin(-driveHeading)
                    + y * Math.cos(-driveHeading);


            // -------------------------------------------------
            // 7. MECANUM POWER CALCULATIONS
            // -------------------------------------------------

            // Calculate power for each wheel
            double fl = rotY + rotX + rx;
            double bl = rotY - rotX + rx;

            double fr = rotY - rotX - rx;
            double br = rotY + rotX - rx;


            // Find the largest absolute motor power
            // so no motor exceeds the allowed power range.
            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br))
            ));


            // Normalize all motor powers
            fl /= max;
            bl /= max;

            fr /= max;
            br /= max;


            // -------------------------------------------------
            // 8. SLOW MODE
            // -------------------------------------------------

            // Read Gamepad 1 B
            boolean bButton = gamepad1.b;

            // Toggle slow mode on a new button press
            if (bButton && !previousBButton) {

                slowMode = !slowMode;
            }

            previousBButton = bButton;


            // Choose the appropriate speed multiplier
            double speedMultiplier =
                    slowMode ? SLOW_SPEED : NORMAL_SPEED;


            // Apply the speed multiplier to all four wheels
            fl *= speedMultiplier;
            bl *= speedMultiplier;

            fr *= speedMultiplier;
            br *= speedMultiplier;


            // -------------------------------------------------
            // 9. MANUAL DRIVE CONTROL
            // -------------------------------------------------

            // Manual motor commands must be disabled while
            // Pedro is controlling the drivetrain.
            //
            // Otherwise, these commands would overwrite
            // Pedro's hive-facing rotation commands.

            if (!turningToHive) {

                leftFront.setPower(fl);
                leftBack.setPower(bl);

                rightFront.setPower(fr);
                rightBack.setPower(br);
            }


            // -------------------------------------------------
            // 10. IMU HEADING RESET
            // -------------------------------------------------

            // Read Gamepad 1 Options
            boolean optionsPressed = gamepad1.options;

            // Reset yaw only on a new button press
            if (optionsPressed && !previousOptions) {

                imu.resetYaw();
            }

            previousOptions = optionsPressed;


            // -------------------------------------------------
            // 11. INTAKE CONTROL
            // -------------------------------------------------

            // Right trigger runs intake forward.
            // Left trigger runs intake backward.
            double intakePower =
                    gamepad1.right_trigger - gamepad1.left_trigger;


            // Control the intake motor
            intake.setPower(intakePower);


            // Control both intake servos
            leftIntakeServo.setPower(-intakePower);
            rightIntakeServo.setPower(-intakePower);


            // -------------------------------------------------
            // 12. LAUNCHER CONTROL
            // -------------------------------------------------

            // Hold Gamepad 1 right bumper to run the launcher
            boolean launcherButton = gamepad1.right_bumper;

            if (launcherButton) {

                // Run at the currently selected velocity
                launcher.setVelocity(launcherVelocity);

            } else {

                // Stop launcher when bumper is released
                launcher.setVelocity(0);
            }

            //67. DEFENSE!!

            // Toggle slow mode on a new button press

            boolean defenseButton = gamepad1.a;

            if (defenseButton && !previousAButton) {

                // Flip the state (if false -> becomes true; if true -> becomes false)
                armIsUp = !armIsUp;
                double targetangle = 110;
                double sussy = -30;
                if (armIsUp) {
                    defense.setPosition(sussy/300); // Move arm UP and hold
                } else {
                    defense.setPosition(targetangle/300); // Move arm DOWN and hold
                }
            }

            // Update previousA for the next loop
            previousAButton = defenseButton;

            //68. FLOWER.
            boolean flowerButton = gamepad1.y;

            if (flowerButton && !previousYButton) {

                // Flip the state (if false -> becomes true; if true -> becomes false)
                flowerIsDown = !flowerIsDown;
                double idealangle = 143;
                double awman = 17;
                if (flowerIsDown) {
                    flower_intake.setPosition(idealangle/300); // Move arm UP and hold
                } else {
                    flower_intake.setPosition(awman/300); // Move arm DOWN and hold
                }
            }

            // Update previousA for the next loop
            previousYButton = flowerButton;

            // -------------------------------------------------
            // 13. TRANSFER CONTROL
            // -------------------------------------------------

            // Hold Gamepad 1 left bumper to operate transfer
            boolean transferButton = gamepad1.left_bumper;

            if (transferButton) {

                // Only operate transfer when launcher is spinning
                if (launcher.getVelocity() > 100) {

                    // DcMotor.setPower() uses a range of -1.0 to 1.0.
                    transfer.setPower(-1.0);

                } else {

                    transfer.setPower(0);
                }

            } else {

                transfer.setPower(0);
            }


            // -------------------------------------------------
            // 14. LAUNCHER VELOCITY ADJUSTMENT
            // -------------------------------------------------

            // Read D-pad inputs
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;


            // Increase launcher velocity on a new Up press
            if (dpadUp && !previousDpadUp) {

                launcherVelocity += LAUNCHER_INCREMENT;
            }


            // Decrease launcher velocity on a new Down press
            if (dpadDown && !previousDpadDown) {

                launcherVelocity -= LAUNCHER_INCREMENT;
            }


            // Prevent velocity from going outside allowed limits
            launcherVelocity = Math.max(
                    MIN_LAUNCHER_VELOCITY,
                    Math.min(MAX_LAUNCHER_VELOCITY, launcherVelocity)
            );


            // Save D-pad states
            previousDpadUp = dpadUp;
            previousDpadDown = dpadDown;


            // -------------------------------------------------
            // 15. PEDRO LOCALIZATION TELEMETRY
            // -------------------------------------------------

            telemetry.addLine("===== PEDRO LOCALIZATION =====");

            telemetry.addData(
                    "Pedro X",
                    "%.2f",
                    currentPose.x()
            );

            telemetry.addData(
                    "Pedro Y",
                    "%.2f",
                    currentPose.y()
            );

            telemetry.addData(
                    "Pedro Heading",
                    "%.1f degrees",
                    Math.toDegrees(currentPose.heading())
            );


            // -------------------------------------------------
            // 16. ALLIANCE AND HIVE TELEMETRY
            // -------------------------------------------------

            telemetry.addLine("===== ALLIANCE / HIVE =====");

            telemetry.addData(
                    "Alliance",
                    isRedAlliance ? "RED" : "BLUE"
            );

            telemetry.addData(
                    "Selected Hive",
                    alternateHiveTarget ? "Hive 2" : "Hive 1"
            );

            telemetry.addData("Hive X", "%.2f", hiveX);
            telemetry.addData("Hive Y", "%.2f", hiveY);


            // -------------------------------------------------
            // 17. HIVE TURNING TELEMETRY
            // -------------------------------------------------

            telemetry.addLine("===== HIVE TARGETING =====");

            telemetry.addData(
                    "Target Heading",
                    "%.1f degrees",
                    Math.toDegrees(targetHeading)
            );

            telemetry.addData(
                    "Heading Error",
                    "%.1f degrees",
                    Math.toDegrees(headingError)
            );

            telemetry.addData("Turning To Hive", turningToHive);


            // -------------------------------------------------
            // 18. IMU TELEMETRY
            // -------------------------------------------------

            telemetry.addLine("===== IMU =====");

            telemetry.addData(
                    "IMU Heading",
                    "%.1f degrees",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Drive Heading",
                    "%.1f degrees",
                    Math.toDegrees(driveHeading)
            );


            // -------------------------------------------------
            // 19. DRIVE TELEMETRY
            // -------------------------------------------------

            telemetry.addLine("===== DRIVE =====");

            telemetry.addData(
                    "Drive Mode",
                    "Field-Centric (Back = Front)"
            );

            telemetry.addData(
                    "Slow Mode",
                    slowMode ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Drive Speed",
                    "%.0f%%",
                    speedMultiplier * 100
            );

            telemetry.addData("Joystick Y", "%.2f", y);
            telemetry.addData("Joystick X", "%.2f", x);
            telemetry.addData("Rotation", "%.2f", rx);

            telemetry.addData(
                    "FL / BL",
                    "%.2f / %.2f",
                    fl, bl
            );

            telemetry.addData(
                    "FR / BR",
                    "%.2f / %.2f",
                    fr, br
            );


            // -------------------------------------------------
            // 20. MECHANISM TELEMETRY
            // -------------------------------------------------

            telemetry.addLine("===== MECHANISMS =====");

            telemetry.addData(
                    "Intake Power",
                    "%.2f",
                    intakePower
            );

            telemetry.addData(
                    "Launcher Button",
                    launcherButton
            );

            telemetry.addData(
                    "Launcher Target",
                    "%.0f",
                    launcherVelocity
            );

            telemetry.addData(
                    "Launcher Actual",
                    "%.0f",
                    launcher.getVelocity()
            );

            telemetry.addData(
                    "Transfer Button",
                    transferButton
            );


            // -------------------------------------------------
            // 21. UPDATE DRIVER HUB DISPLAY
            // -------------------------------------------------

            telemetry.update();
        }


        // =====================================================
        // K. SHUTDOWN
        // =====================================================

        // Stop Pedro's drivetrain control
        follower.stop();


        // Stop all four drive motors
        leftFront.setPower(0);
        leftBack.setPower(0);

        rightFront.setPower(0);
        rightBack.setPower(0);


        // Stop intake mechanisms
        intake.setPower(0);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);


        // Stop transfer and launcher
        transfer.setPower(0);
        launcher.setVelocity(0);
    }
}
