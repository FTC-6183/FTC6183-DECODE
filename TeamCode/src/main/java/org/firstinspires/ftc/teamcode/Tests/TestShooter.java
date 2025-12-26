package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Config
@TeleOp(name = "TestShooter")
public class TestShooter extends NextFTCOpMode {
    public static double hoodPosition = 0;
    public static double velocity = 0;
    public TestShooter(){
        addComponents(
                new SubsystemComponent(Limelight.INSTANCE,Drivetrain.INSTANCE,Turret.INSTANCE,Transfer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.INSTANCE.startRobotDrive().schedule();
        /*
        Gamepads.gamepad1().leftBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.testOnOneWay)
                .whenBecomesFalse(Turret.INSTANCE.testOff);
        Gamepads.gamepad1().rightBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.testOtherWay)
                .whenBecomesFalse(Turret.INSTANCE.testOff);
         */
        Gamepads.gamepad1().y()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Transfer.INSTANCE.transferUp())
                .whenBecomesFalse(Transfer.INSTANCE.transferDown());
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Turret.INSTANCE.testServoOn)
                .whenBecomesFalse(Turret.INSTANCE.testServoOff);
    }
    @Override
    public void onUpdate(){
        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        Turret.INSTANCE.setVelocity(-velocity).schedule();
        Turret.INSTANCE.followAprilTag().schedule();
        Turret.INSTANCE.periodic();
        BindingManager.update();
        telemetry.addData("Distance From Blue Tag", Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Distance From Red Tag", Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Angle From Blue Tag", Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Angle From Red Tag", Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Pattern of Obelisk",
                Limelight.INSTANCE.patternFromObelisk() == Limelight.GPP_PATTERN_ID ? "GPP":
                        Limelight.INSTANCE.patternFromObelisk() == Limelight.PGP_PATTERN_ID ? "PGP":
                                Limelight.INSTANCE.patternFromObelisk() == Limelight.PPG_PATTERN_ID ? "PPG":
                                        "Can't be found");
//
        //telemetry.addData("Turret Positions", Turret.INSTANCE.servoPosition());
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Turret Positions", Turret.INSTANCE.headingToTurretPosition());
        telemetry.addData("Turret Relative Angle", Turret.INSTANCE.getCurrentAngleFromEncoder());
        telemetry.addData("Turret Absolute Angle", Turret.INSTANCE.getAbsoluteAngleFromEncoder());
        telemetry.addData("Turret Power", Turret.INSTANCE.getTurretPower());
        telemetry.addData("Turret PID Correction", Turret.INSTANCE.turretPIDCorrection());
        telemetry.addData("Shooter One Velocity", Turret.INSTANCE.getVelocityOne());
        telemetry.addData("Shooter Two Velocity", Turret.INSTANCE.getVelocityTwo());
        telemetry.addData("Set Velocity",velocity);
        telemetry.update();
    }
    @Override
    public void onStop(){
    }
}


