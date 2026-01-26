package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Position.POSITION_ONE;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@Configurable
@TeleOp(name = "Blue TeleOp", group = "Blue")
public class BlueTeleOp extends NextFTCOpMode {
    public static double spinAngle = 18.55;
    public static Spindexer.Position position = POSITION_ONE;
    public static double hoodPosition = 0;
    public static double velocity = 0;
    public static double offsetVelocity = 0;
    public static double shootThreshold = 100;
    public static int rumbleBlips = 5;

    public static String type = "";
    private Follower follower;

    public BlueTeleOp() {
        addComponents(
                new SubsystemComponent(Limelight.INSTANCE, Drivetrain.INSTANCE, Turret.INSTANCE, Transfer.INSTANCE, Spindexer.INSTANCE, Intake.INSTANCE, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.INSTANCE.startRobotDrive().schedule();
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90));
        //Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH,72,72, AngleUnit.DEGREES,90));
        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.on())
                .whenBecomesFalse(Intake.INSTANCE.idle());

        Command transferFlick = new SequentialGroup(Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(() -> Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY))
        );
        Command intakeMode = new SequentialGroup(
                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE)));

        Command shootMode = new SequentialGroup(
                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)));

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(transferFlick);

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesFalse()
                .whenBecomesTrue(intakeMode)
                .whenBecomesFalse(shootMode);

        Gamepads.gamepad1().square()
                .whenBecomesTrue(() -> Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90)));


        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Spindexer.Position::next);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Spindexer.Position::previous);

    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        Turret.INSTANCE.followGoalOdometryPositional().schedule();
        Pinpoint.INSTANCE.periodic();
        telemetry.addData("x:", Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y:", Pinpoint.INSTANCE.getPosY());
        telemetry.addData("heading", Pinpoint.INSTANCE.getHeading());
        telemetry.addData("360 Heading", (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addData("Position of Turret", (Turret.INSTANCE.positionToAngle(Turret.INSTANCE.turretOnePosition())));
        telemetry.addData("Turret One Position", Turret.INSTANCE.turretOnePosition());
        telemetry.addData("Turret Two Position", Turret.INSTANCE.turretTwoPosition());
        telemetry.addData("Turret Angle Set", (Turret.INSTANCE.getTurretAngleSet()));
        telemetry.addData("Turret Power Set", (Turret.INSTANCE.getTurretPowerSet()));
        telemetry.addData("Shooter Velocity", Turret.INSTANCE.getVelocity());
        telemetry.addData("Set Velocity", velocity);
        telemetry.addData("Distance From Blue Tag", Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Hood Position Set", Turret.INSTANCE.getPosition());
        telemetry.addData("Current Color", Spindexer.INSTANCE.readCurrentColor());
        telemetry.addData("Ball at Position One", Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball at Position Two", Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball at Position Three", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Nearest Free Position", Spindexer.INSTANCE.freePosition());
        telemetry.addData("Spindexer Position", Spindexer.INSTANCE.getPosition());
        telemetry.addData("Mode", Spindexer.INSTANCE.getPositionType());
        if (Spindexer.INSTANCE.freePosition() != -1 && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]).schedule();
        } else {
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()).schedule();
        }
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            velocity = 500;
        } else if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT) {
            velocity = Turret.INSTANCE.distanceToVelocity(Pinpoint.INSTANCE.getPosX(), Pinpoint.INSTANCE.getPosY()) + offsetVelocity;
            if (Math.abs(velocity - Turret.INSTANCE.getVelocity()) < shootThreshold) {
                gamepad1.rumbleBlips(rumbleBlips);
            }
        }
        hoodPosition = Turret.INSTANCE.distanceToPosition(Pinpoint.INSTANCE.getPosX(), Pinpoint.INSTANCE.getPosY()) + offsetVelocity;
        Turret.INSTANCE.setVelocity(velocity).schedule();
        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();
        telemetry.update();
    }
}