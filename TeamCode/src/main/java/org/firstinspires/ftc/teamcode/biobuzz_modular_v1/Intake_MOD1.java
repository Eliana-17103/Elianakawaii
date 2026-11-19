package org.firstinspires.ftc.teamcode.biobuzz_modular_v1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Intake motor + two continuous-rotation servos (right one reversed). */
public class Intake_MOD1 {
    private final DcMotor motor;
    private final CRServo leftServo, rightServo;
    private boolean servosEnabled = true;   // false when the flower mechanism is down
    private double power = 0;

    public Intake_MOD1(HardwareMap hw) {
        motor      = hw.get(DcMotor.class, "intake");
        leftServo  = hw.get(CRServo.class, "left_intake_servo");
        rightServo = hw.get(CRServo.class, "right_intake_servo");
        rightServo.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setServosEnabled(boolean enabled) { servosEnabled = enabled; }

    /** +1 = intake, -1 = outtake. */
    public void setPower(double p) {
        power = p;
        motor.setPower(p);
        if (servosEnabled) {
            leftServo.setPower(-p);
            rightServo.setPower(-p);
        }
    }

    public void stop() {
        motor.setPower(0);
        leftServo.setPower(0);
        rightServo.setPower(0);
        power = 0;
    }

    public void addTelemetry(Telemetry t) {
        t.addData("Intake", "%.2f", power);
    }
}
