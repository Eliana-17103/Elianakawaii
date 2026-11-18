package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front_drive");
        c.frontRightName.set("right_front_drive");
        c.backLeftName.set("left_back_drive");
        c.backRightName.set("right_back_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("poop");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.9198956827479086);
        c.yPodOffset.set(7.197626068836123);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(1.6537637429951186);
                Controller secondaryTranslationalForward = Controller.proportional(0.6110215136556361);
                Controller primaryTranslationalLateral = Controller.proportional(0.47474767934150924);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17540658202506362);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.020006338368228148));
                c.brake.set(Controller.proportionalFeedforward(0.017005387612993927));

                c.headingFeedback.set(Controller.proportional(4.836966070360826));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.044621106819103556, 0.012063217884836722));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06183048707269619, 0.0948515314248963));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00349007616984236, 0.002258987568770901));

                c.maxAchievableForwardVelocity.set(39.18436965388234);
                c.maxAchievableStrafeVelocity.set(33.06551157878122);
                c.naturalForwardDeceleration.set(15.922546625028172);
                c.naturalStrafeDeceleration.set(20.517348857372404);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}