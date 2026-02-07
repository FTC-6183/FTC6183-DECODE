package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.NextFTCPatch.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.PositionType.INTAKE;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.PositionType.SHOOT;

import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Position.POSITION_ONE;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Position.POSITION_TWO;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Position.POSITION_THREE;




import android.widget.Spinner;


@TeleOp
@Config
public class TestSpindexer extends NextFTCOpMode {
    public static double spinAngle = 0;
    public static String type = "";
    public boolean shootcycle = false;
    Robot robot = new Robot(Aliance.BLUE);
    public TestSpindexer(){
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE, Intake.INSTANCE, Turret.INSTANCE, Transfer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public Command setToPositionOne(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_ONE));
    }

    public Command setToPositionTwo(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_TWO));
    }

    public Command setToPositionThree(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_THREE));
    }

    public Command shootThree(){
        return new SequentialGroupFixed(
                new InstantCommand(() -> shootcycle = true),
                new InstantCommand(()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)),
                new Delay(0.5),
                setToPositionOne(),
                new Delay(0.5),
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                setToPositionTwo(),
                new Delay(0.5),
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                setToPositionThree(),
                new Delay(0.5),
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                new Delay(0.2),
                new InstantCommand(()->shootcycle = false)
        );
    }
    public Command intakeMode = new IfElseCommand(()->!shootcycle, new InstantCommand( ()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE)));

    public Command shootMode = new IfElseCommand(()->!shootcycle, new InstantCommand( ()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)));

    @Override
    public void onStartButtonPressed() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.INSTANCE.startRobotDrive().schedule();
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE).schedule();

        Command transferFlick = new SequentialGroup(Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY))
        );

        //new WaitUntil(()->(Math.abs(Spindexer.INSTANCE.getAbsoluteAngleFromEncoder() - Spindexer.INSTANCE.getShootAngle(0)) < 10),

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Spindexer.Position::next);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Spindexer.Position::previous);

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(shootThree());

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(shootMode)
                .whenBecomesFalse(intakeMode);

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.on())
                .whenBecomesFalse(Intake.INSTANCE.idle());

        Gamepads.gamepad1().dpadUp()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(transferFlick)
                .whenBecomesFalse(transferFlick);

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(setToPositionOne());

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(setToPositionTwo());

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(setToPositionThree());


        Gamepads.gamepad1().square()
                .whenBecomesTrue(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY));

    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Shoot Cycle ", shootcycle);
        telemetry.addData("Empty ", Spindexer.INSTANCE.getEmpty());
        telemetry.addData("Full ", Spindexer.INSTANCE.getFull());
        telemetry.addData("Filled Position ", Spindexer.INSTANCE.filledPosition());
        telemetry.addData("Nearest Free Position", Spindexer.INSTANCE.freePosition());
        telemetry.addData("Spindexer 360 Angle", Spindexer.INSTANCE.getCurrentAngleFromEncoder());
        telemetry.addData("Current Color", Spindexer.INSTANCE.readCurrentColor());
        telemetry.addData("Ball at Position One", Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball at Position Two", Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball at Position Three", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Spindexer Position", Spindexer.INSTANCE.getPosition());
        telemetry.addData("Mode",Spindexer.INSTANCE.getPositionType());
        Turret.INSTANCE.setVelocity(0).schedule();
        if(Spindexer.INSTANCE.freePosition()!=-1 && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE&& !shootcycle) {
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]).schedule();
        }
        else if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT && Spindexer.INSTANCE.filledPosition()!=-1&&!shootcycle ){
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.filledPosition()]).schedule();
        }
        else if(!shootcycle){
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[0]).schedule();
        }
        else{
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()).schedule();
        }
        Spindexer.INSTANCE.periodic();
        telemetry.update();
    }
    @Override
    public void onStop(){
    }
}
