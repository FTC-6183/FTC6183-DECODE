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
import org.firstinspires.ftc.teamcode.NextFTCPatch.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Utils.Aliance;


import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
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
@TeleOp(name = "Red TeleOp", group = "Red")
public class RedTeleOp extends NextFTCOpMode {
    public static Spindexer.Position position = POSITION_ONE;
    public static double hoodPosition = 0;
    public static double velocity = 0;
    public boolean turretLock = false;
    public RedTeleOp(){
        addComponents(
                new SubsystemComponent(Limelight.INSTANCE, Drivetrain.INSTANCE, Turret.INSTANCE, Transfer.INSTANCE, Spindexer.INSTANCE, Intake.INSTANCE, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.INSTANCE.startRobotDrive().schedule();
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90));
        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.on())
                .whenBecomesFalse(Intake.INSTANCE.idle());

        Command transferFlick = new SequentialGroup(Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY))
        );
        Command intakeMode = new SequentialGroup(
                new InstantCommand(()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE)));

        Command shootMode = new SequentialGroup(
                new InstantCommand(()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)));

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(transferFlick);

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesFalse()
                .whenBecomesTrue(intakeMode)
                .whenBecomesFalse(shootMode);

        Gamepads.gamepad1().square()
                .whenBecomesTrue(()->Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90)));

        Gamepads.gamepad1().dpadLeft()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.reverse())
                .whenBecomesFalse(Intake.INSTANCE.idle());

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Spindexer.Position::next);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Spindexer.Position::previous);

        Gamepads.gamepad2().rightBumper()
                .whenTrue(()->Turret.INSTANCE.updateAngleOffset(-1));

        Gamepads.gamepad2().leftBumper()
                .whenTrue(()->Turret.INSTANCE.updateAngleOffset(1));

        Gamepads.gamepad2().cross()
                .whenBecomesTrue(()->Turret.INSTANCE.zeroAngleOffset());

        Gamepads.gamepad2().b()
                .toggleOnBecomesTrue()
                .whenBecomesTrue( () -> turretLock = true)
                .whenBecomesFalse(() -> turretLock = false);
    }
    @Override
    public void onUpdate() {
        BindingManager.update();
        Pinpoint.INSTANCE.periodic();

        telemetry.addData("x", Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y", Pinpoint.INSTANCE.getPosY());
        telemetry.addData("Heading", (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));

        telemetry.addData("Shooter Velocity", Turret.INSTANCE.getVelocity());
        telemetry.addData("Set Velocity", velocity);

        telemetry.addData("Current Color", Spindexer.INSTANCE.readCurrentColor());
        telemetry.addData("Ball at Position One", Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball at Position Two", Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball at Position Three", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Nearest Free Position", Spindexer.INSTANCE.freePosition());
        telemetry.addData("Spindexer Position", Spindexer.INSTANCE.getPosition());
        telemetry.addData("Mode", Spindexer.INSTANCE.getPositionType());

        if(Spindexer.INSTANCE.freePosition()!=-1 && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]).schedule();
        }
        else{
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()).schedule();
        }

        if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE ){
            velocity = 500;
            if(!turretLock){
                Turret.INSTANCE.setToAngle(90).schedule();
            }
        }
        else if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT){
            velocity = Turret.INSTANCE.distanceToVelocity(Pinpoint.INSTANCE.getPosX() , Pinpoint.INSTANCE.getPosY());
            if(!turretLock){
                Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE).schedule();
            }
            if (Math.abs(Turret.INSTANCE.getVelocity() - velocity) < 30){
                gamepad1.rumbleBlips(1);
            }
        }
        if (turretLock){
            Turret.INSTANCE.setToAngle(90).schedule();
        }

-p; ?        hoodPosition = Turret.INSTANCE.distanceToPosition(Pinpoint.INSTANCE.getPosX(), Pinpoint.INSTANCE.getPosY());
        Turret.INSTANCE.setVelocity(velocity).schedule();
        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();
        telemetry.update();
    }
    @Override
    public void onStop(){
    }
}