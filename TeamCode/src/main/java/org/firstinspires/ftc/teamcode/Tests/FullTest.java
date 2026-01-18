package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Position.POSITION_ONE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@TeleOp(name = "FullTest")
public class FullTest extends NextFTCOpMode {
    public static double spinAngle = 18.55;
    public static Spindexer.Position position = POSITION_ONE;
    public static double hoodPosition = 1;
    public static double velocity = 0;
    public static double turretVelocity = 0;
    public static String type = "";

    public FullTest(){
        addComponents(
                new SubsystemComponent(Limelight.INSTANCE, Drivetrain.INSTANCE, Turret.INSTANCE, Transfer.INSTANCE, Spindexer.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.INSTANCE.startRobotDrive().schedule();

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(new SequentialGroup(
                        new InstantCommand(()-> type ="Shoot"),
                        //Spindexer.INSTANCE.setToPosition(POSITION_ONE,type),
                        Intake.INSTANCE.idle(),
                        /*
                        new LambdaCommand()
                                .setUpdate(new SequentialGroup(
                                        Turret.INSTANCE.waitToShoot,
                                        Transfer.INSTANCE.transferUp(),
                                        Transfer.INSTANCE.transferDown(),
                                        Spindexer.INSTANCE.nextPosition(type))
                                )
                                .setIsDone(()->Spindexer.INSTANCE.getPosition() == POSITION_THREE),

                         */
                        //Turret.INSTANCE.waitToShoot,
                        //Turret.INSTANCE.setVelocity(-velocity),
                        Turret.INSTANCE.on(),
                        new Delay(0.2),
                        Transfer.INSTANCE.transferUp(),
                        new Delay(0.2),
                        Transfer.INSTANCE.transferDown(),
                        new Delay(0.2)
                ))
                .whenBecomesFalse(new ParallelGroup(
                        Intake.INSTANCE.on(),
                        new InstantCommand(()->type = "Intake"),
                        Turret.INSTANCE.setHoodPosition(1),
                        Turret.INSTANCE.off
                ));

        Gamepads.gamepad1().cross()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Transfer.INSTANCE.transferUp())
                .whenBecomesFalse(Transfer.INSTANCE.transferDown());

        Command spinSequence = new SequentialGroup(Transfer.INSTANCE.transferDown(),Spindexer.INSTANCE.nextPosition());

        Gamepads.gamepad1().square()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(spinSequence)
                .whenBecomesFalse(Transfer.INSTANCE.transferUp());

        Gamepads.gamepad1().triangle()
                        .toggleOnBecomesTrue()
                        .whenBecomesTrue(()-> type = "Intake")
                        .whenBecomesFalse(()-> type = "Shoot");

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Spindexer.Position::next);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Spindexer.Position::previous);


    }
    @Override
    public void onUpdate(){
        BindingManager.update();
        //telemetry.addData("Spindexer 360 Angle", Spindexer.INSTANCE.getCurrentAngleFromEncoder());
        //telemetry.addData("Spindexer 180 Angle", Spindexer.INSTANCE.getAbsoluteAngleFromEncoder());
        telemetry.addData("Spin Servo Power", Spindexer.INSTANCE.getPower());
        //telemetry.addData("Current Position", Spindexer.INSTANCE.getPosition());
        //telemetry.addData("Shooter One Velocity", Turret.INSTANCE.getVelocityOne());
        telemetry.addData("Shooter Velocity", Turret.INSTANCE.getVelocityTwo());
        telemetry.addData("Set Velocity",velocity);
        telemetry.addData("Mode",type);
        telemetry.addData("Distance From Blue Tag", Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        //telemetry.addData("Distance From Red Tag", Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Angle From Blue Tag", Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID));
        //telemetry.addData("Angle From Red Tag", Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Pattern of Obelisk",
                Limelight.INSTANCE.patternFromObelisk() == Limelight.GPP_PATTERN_ID ? "GPP":
                        Limelight.INSTANCE.patternFromObelisk() == Limelight.PGP_PATTERN_ID ? "PGP":
                                Limelight.INSTANCE.patternFromObelisk() == Limelight.PPG_PATTERN_ID ? "PPG":
                                        "Can't be found");
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Turret Positions", Turret.INSTANCE.headingToTurretPositionLL());
        telemetry.addData("Turret Relative Angle", Turret.INSTANCE.getWrappedAngleFromEncoder());
        telemetry.addData("Turret Absolute Angle", Turret.INSTANCE.getNonWrappedAngleFromEncoder());
        telemetry.addData("Spindexer Position", Spindexer.INSTANCE.getPosition());
        //telemetry.addData("Turret Power", Turret.INSTANCE.getTurretPower());

        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        //Turret.INSTANCE.followAprilTag().schedule();
        //Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition(),type).schedule();
        //Turret.INSTANCE.setVelocity(-velocity).schedule();
        Spindexer.INSTANCE.periodic();
        Turret.INSTANCE.periodic();
        telemetry.update();
    }
    @Override
    public void onStop(){
    }
}

