package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
@Config
@TeleOp
public class TestTurret extends NextFTCOpMode {
    public static double turretPosition = 0.5;
    public TestTurret() {
        addComponents(
                new SubsystemComponent(Drivetrain.INSTANCE, Turret.INSTANCE, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 144-8.5, 8.875, AngleUnit.DEGREES, 90));
        Drivetrain.INSTANCE.startRobotDrive().schedule();
    }
    @Override
    public void onUpdate(){
        telemetry.addData("x", Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y", Pinpoint.INSTANCE.getPosY());
        telemetry.addData("heading", Pinpoint.INSTANCE.getHeading());
        telemetry.addData("360 Heading", (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addData("Position of Turret", (Turret.INSTANCE.positionToAngle(Turret.INSTANCE.turretOnePosition())));
        telemetry.addData("Turret One Position", Turret.INSTANCE.turretOnePosition());
        telemetry.addData("Turret Two Position", Turret.INSTANCE.turretTwoPosition());
        telemetry.addData("Turret Angle Set", (Turret.INSTANCE.getTurretAngleSet()));
        telemetry.addData("Turret Power Set", (Turret.INSTANCE.getTurretPowerSet()));
        telemetry.update();
        Turret.INSTANCE.setVelocity(0).schedule();
        Turret.INSTANCE.setTurretPosition(turretPosition).schedule();
        Pinpoint.INSTANCE.periodic();
        Turret.INSTANCE.periodic();

        BindingManager.update();
    }
}
