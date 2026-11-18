package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Turret Gear Tester", group = "Test")
public class ServoTest extends LinearOpMode {

    private Servo turretGear;

    @Override
    public void runOpMode() {

        turretGear = hardwareMap.get(Servo.class, "turret_gear");

        telemetry.addLine("Turret Gear Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            turretGear.setPosition(1.0);

            telemetry.addLine("Servo running for 10 seconds");
            telemetry.update();

            sleep(10000);

            turretGear.setPosition(0.5);

            telemetry.addLine("Test complete!");
            telemetry.update();
        }
    }
}