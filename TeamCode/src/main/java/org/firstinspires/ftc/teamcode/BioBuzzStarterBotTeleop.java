
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "BioBuzz Mecanum TeleOp 0", group = "TeleOp")
@Disabled
public class BioBuzzStarterBotTeleop extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor intake;
    private DcMotorEx launcher;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    private static final double LAUNCHER_VELOCITY = 1400;
    private static final double FEED_THRESHOLD = LAUNCHER_VELOCITY*0.92;

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
        rightFront.setDirection(DcMotor.Direction.REVERSE);
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
            // ROBOT-CENTRIC MECANUM DRIVE
            // ---------------------------------
            /*
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            */
            double y = gamepad1.left_stick_y; // Inverted forward/backward
            double x = -gamepad1.left_stick_x; // Restore your original strafe direction
            double rx = -gamepad1.right_stick_x;

            // Wheel powers.
            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            // Normalize powers to the range [-1, 1].
            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br))
            ));

            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;

            leftFront.setPower(fl);
            leftBack.setPower(bl);
            rightFront.setPower(fr);
            rightBack.setPower(br);

            // ---------------------------------
            // INTAKE: TRIGGERS
            // ---------------------------------
            double intakePower =
                    gamepad1.right_trigger - gamepad1.left_trigger;

            // Right trigger forward, left trigger reverse.
            intake.setPower(intakePower);
            leftIntakeServo.setPower(intakePower);
            rightIntakeServo.setPower(intakePower);

            // ---------------------------------
            // LAUNCHER + WINDMILL FEED
            // ---------------------------------
            boolean launcherButton = gamepad1.right_bumper;

            if (launcherButton) {
                launcher.setVelocity(LAUNCHER_VELOCITY);

                // Feed only after launcher reaches the threshold.
                if (launcher.getVelocity() > FEED_THRESHOLD) {
                    windmillServo.setPower(1.0);
                } else {
                    windmillServo.setPower(0.0);
                }
            } else {
                launcher.setVelocity(0);
                windmillServo.setPower(0.0);
            }

            // ---------------------------------
            // TELEMETRY
            // ---------------------------------
            telemetry.addData("Drive mode", "Robot-centric");
            telemetry.addData("Gamepad 1 left Y", "%.2f",
                    gamepad1.left_stick_y);
            telemetry.addData("FL / BL", "%.2f / %.2f", fl, bl);
            telemetry.addData("FR / BR", "%.2f / %.2f", fr, br);

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
}
