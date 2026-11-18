
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BioBuzz Mecanum TeleOp 1RC", group = "TeleOp")
@Disabled
public class BioBuzzStarterBotTeleopRobotC extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor intake;
    private DcMotorEx launcher;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    private IMU imu;

    private static double LAUNCHER_VELOCITY = 1400;
    private static double FEED_THRESHOLD = LAUNCHER_VELOCITY * 0.80;

    @Override

    public void runOpMode() {

        // Hardware mapping: names must match Robot Configuration.
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        leftIntakeServo =
                hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo =
                hardwareMap.get(CRServo.class, "right_intake_servo");
        windmillServo =
                hardwareMap.get(CRServo.class, "windmill");

        // Starting drivetrain directions.
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Stop drivetrain motors when sticks are released.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Retained from the previous merged code.
        rightIntakeServo.setDirection(DcMotor.Direction.REVERSE);
        windmillServo.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Robot-centric teleop initialized");
        telemetry.addLine("Left stick: drive / strafe");
        telemetry.addLine("Right stick X: turn");
        telemetry.addLine("Right bumper: launcher + conditional feed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------------------------
            // FIELD-CENTRIC MECANUM DRIVE at bottom
            // ---------------------------------

            // ---------------------------------
            // INTAKE: TRIGGERS
            // ---------------------------------
            double intakePower =
                    gamepad1.right_trigger - gamepad1.left_trigger;

            // Right trigger forward, left trigger reverse.
            intake.setPower(intakePower);
            leftIntakeServo.setPower(-intakePower);
            rightIntakeServo.setPower(-intakePower);

            // ---------------------------------
            // LAUNCHER + WINDMILL FEED
            // ---------------------------------
            boolean launcherButton = gamepad1.right_bumper;
            boolean windmillButton = gamepad1.left_bumper;

            if (launcherButton) {
                launcher.setVelocity(LAUNCHER_VELOCITY);

                // Feed only after launcher reaches the threshold.
                /*
                if (launcher.getVelocity() > FEED_THRESHOLD) {
                    windmillServo.setPower(1.0);
                } else {
                    windmillServo.setPower(0.0);
                }

                 */
            } else {
                launcher.setVelocity(0);
                windmillServo.setPower(0.0);

            }


            if (windmillButton) {
                windmillServo.setPower(1.0);
            } else {
                windmillServo.setPower(0.0);
            }
            // ---------------------------------
            // Variable Max Speed
            // ---------------------------------
            if (gamepad1.dpad_up) {
                LAUNCHER_VELOCITY += 10;
            }

            if (gamepad1.dpad_down) {
                LAUNCHER_VELOCITY -= 10;
            }

            // ---------------------------------
            // IMU Controls/Config
            // ---------------------------------
            if (gamepad1.a) {
                imu.resetYaw();
            }

            if (gamepad1.x) {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            // ---------------------------------
            // TELEMETRY
            // ---------------------------------
            telemetry.addData("Drive mode", "Robot-centric");
            telemetry.addData("Gamepad 1 left Y", "%.2f",
                    gamepad1.left_stick_y);


            telemetry.addData("Intake power", "%.2f", intakePower);
            telemetry.addData("Launcher button", launcherButton);
            telemetry.addData("Launcher velocity", "%.0f",
                    launcher.getVelocity());
            telemetry.addData("Windmill power", "%.1f",
                    windmillServo.getPower());

            telemetry.update();
        }

        // Stop mechanisms when the OpMode ends.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        launcher.setVelocity(0);
    }
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        leftFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        rightBack.setPower(maxSpeed * (frontRightPower / maxPower));
        leftBack.setPower(maxSpeed * (backLeftPower / maxPower));
        rightBack.setPower(maxSpeed * (backRightPower / maxPower));

        telemetry.addData("FL / BL", "%.2f / %.2f", frontLeftPower, backLeftPower);
        telemetry.addData("FR / BR", "%.2f / %.2f", frontRightPower, backRightPower);
    }
}
