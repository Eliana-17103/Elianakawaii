package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Mecanum drive, field-centric. Pass the joystick values and the driver-frame heading each loop. */
public class FieldCentricDrive_MOD2 {

    public static double DEADZONE = 0.05;
    public static double NORMAL_SPEED = 1.0, SLOW_SPEED = 0.35;
    public static double STRAFE_BOOST = 1.1;

    private final DcMotor leftFront, leftBack, rightFront, rightBack;
    private boolean slowMode = false;

    public FieldCentricDrive_MOD2(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void toggleSlowMode() { slowMode = !slowMode; }
    public boolean isSlowMode()  { return slowMode; }

    /** Biggest joystick push (0 to 1). Used to know when the driver wants control back. */
    public static double stickPush(double leftX, double leftY, double rightX) {
        return Math.max(Math.abs(leftX), Math.max(Math.abs(leftY), Math.abs(rightX)));
    }

    /**
     * @param leftX        gamepad left_stick_x
     * @param leftY        gamepad left_stick_y
     * @param rightX       gamepad right_stick_x
     * @param driveHeading robot heading (radians) in the driver's frame
     */
    public void drive(double leftX, double leftY, double rightX, double driveHeading) {
        double y  = -leftY;
        double x  = leftX * STRAFE_BOOST;
        double rx = -rightX;
        if (Math.abs(y)  < DEADZONE) y = 0;
        if (Math.abs(x)  < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        // Rotate the joystick direction by the robot's heading
        double rotX = x * Math.cos(-driveHeading) - y * Math.sin(-driveHeading);
        double rotY = x * Math.sin(-driveHeading) + y * Math.cos(-driveHeading);

        double fl = rotY + rotX + rx;
        double bl = rotY - rotX + rx;
        double fr = rotY - rotX - rx;
        double br = rotY + rotX - rx;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                                            Math.max(Math.abs(fr), Math.abs(br))));
        double scale = (slowMode ? SLOW_SPEED : NORMAL_SPEED) / max;

        leftFront.setPower(fl * scale);
        leftBack.setPower(bl * scale);
        rightFront.setPower(fr * scale);
        rightBack.setPower(br * scale);
    }

    public void halt() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
