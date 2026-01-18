package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Position.POSITION_ONE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Config
@TeleOp(name = "TestShooter")
public class TestShooter extends NextFTCOpMode {
    public static double hoodPosition = 0;
    public static double velocity = 0;
    public static boolean ppLL = true;
    public static Spindexer.Position position = POSITION_ONE;
    public static double spinAngle = 0;
    public TestShooter(){
        addComponents(
                new SubsystemComponent(Limelight.INSTANCE,Drivetrain.INSTANCE,Turret.INSTANCE,Transfer.INSTANCE, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH,72,72, AngleUnit.DEGREES,90));
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH,8.5,8.875, AngleUnit.DEGREES,90));
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

        Gamepads.gamepad1().square()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> ppLL = true)
                .whenBecomesFalse(() -> ppLL = false);
        /*
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Turret.INSTANCE.testServoOn)
                .whenBecomesFalse(Turret.INSTANCE.testServoOff);
                */

    }
    @Override
    public void onUpdate(){
        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        Turret.INSTANCE.setVelocity(-velocity).schedule();
        Turret.INSTANCE.followGoalOdometryPositional().schedule();
        telemetry.addLine("Turret Tracking Odometry");
        telemetry.addData("x", Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y", Pinpoint.INSTANCE.getPosY());
        telemetry.addData("heading", Pinpoint.INSTANCE.getHeading());
        telemetry.addData("360 Heading", (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addData("Position of Turret", (Turret.INSTANCE.positionToAngle(Turret.INSTANCE.turretOnePosition())));
        telemetry.addData("Turret Angle Set", (Turret.INSTANCE.getTurretAngleSet()));
        telemetry.addData("Turret Power Set", (Turret.INSTANCE.getTurretPowerSet()));
        telemetry.addData("Turret One Position", Turret.INSTANCE.turretOnePosition());

        //telemetry.addData("Turret Maximum Voltage",Turret.INSTANCE.getMaxVoltageFromEncoder());
        //telemetry.addData("Turret Current Voltage",Turret.INSTANCE.getVoltageFromEncoder());
        //telemetry.addData("Turret One Position", Turret.INSTANCE.turretTwoPosition());
        Pinpoint.INSTANCE.periodic();
        Turret.INSTANCE.periodic();
        //Spindexer.INSTANCE.setToPosition(position);
        //Spindexer.INSTANCE.setToAngle(spinAngle).schedule();
        ///Spindexer.INSTANCE.periodic();
        BindingManager.update();
        /*
        telemetry.addData("Distance From Blue Tag", Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Distance From Red Tag", Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Angle From Blue Tag", Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Angle From Red Tag", Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Pattern of Obelisk",
                Limelight.INSTANCE.patternFromObelisk() == Limelight.GPP_PATTERN_ID ? "GPP":
                        Limelight.INSTANCE.patternFromObelisk() == Limelight.PGP_PATTERN_ID ? "PGP":
                                Limelight.INSTANCE.patternFromObelisk() == Limelight.PPG_PATTERN_ID ? "PPG":
                                        "Can't be found");

         */
        //telemetry.addData("Turret Positions", Turret.INSTANCE.servoPosition());
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Turret Positions", Turret.INSTANCE.headingToTurretPositionLL());
        telemetry.addData("Turret Position Odometry", Turret.INSTANCE.headingToTurretPositionPinpoint());
        telemetry.addData("Current Error", Turret.INSTANCE.getError());
        telemetry.addData("Turret Nonwrapped Angle", Turret.INSTANCE.getNonWrappedAngleFromEncoder());
        telemetry.addData("Turret Wrapped Angle", Turret.INSTANCE.getWrappedAngleFromEncoder());
        telemetry.addData("Spindexer Relative Angle", Spindexer.INSTANCE.getAbsoluteAngleFromEncoder());
        //telemetry.addData("Turret Power", Turret.INSTANCE.getTurretPower());
        telemetry.addData("Turret PID Correction Wrapped", Turret.INSTANCE.turretPIDCorrectionWrapped());
        telemetry.addData("Turret PID Correction Nonwrapped", Turret.INSTANCE.turretPIDCorrectionNonWrapped());
        telemetry.addData("Shooter One Velocity", Turret.INSTANCE.getVelocityOne());
        telemetry.addData("Shooter Two Velocity", Turret.INSTANCE.getVelocityTwo());
        telemetry.addData("Set Velocity",velocity);
        telemetry.update();
    }
    @Override
    public void onStop(){
    }
}


