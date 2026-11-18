
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gamepad Input Test", group = "Test")
public class GamepadInputTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Gamepad test initialized");
        telemetry.addLine("Move either joystick after pressing START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;

            boolean inputDetected =
                    Math.abs(lx) > 0.1 ||
                            Math.abs(ly) > 0.1 ||
                            Math.abs(rx) > 0.1 ||
                            Math.abs(ry) > 0.1;

            telemetry.addData("Left stick X / Y", "%.2f / %.2f", lx, ly);
            telemetry.addData("Right stick X / Y", "%.2f / %.2f", rx, ry);
            telemetry.addData("A button", gamepad1.a);
            telemetry.addData("B button", gamepad1.b);
            telemetry.addData("Input detected", inputDetected);
            telemetry.update();
        }
    }
}
