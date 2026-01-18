package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.PositionType.INTAKE;
import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.PositionType.SHOOT;

import android.widget.Spinner;


@TeleOp
@Config
public class TestSpindexer extends NextFTCOpMode {
    public static double spinAngle = 0;
    public static String type = "";

    public TestSpindexer(){
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.INSTANCE.startRobotDrive().schedule();
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE).schedule();
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Spindexer.Position::next);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Spindexer.Position::previous);

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE))
                .whenBecomesFalse(()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT));

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.on())
                .whenBecomesFalse(Intake.INSTANCE.idle());

        Gamepads.gamepad1().square()
                .whenBecomesTrue(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY));

    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Spindexer 360 Angle", Spindexer.INSTANCE.getCurrentAngleFromEncoder());
        telemetry.addData("Current Color", Spindexer.INSTANCE.readCurrentColor());
        telemetry.addData("Ball at Position One", Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball at Position Two", Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball at Position Three", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Nearest Free Position", Spindexer.INSTANCE.freePosition());
        telemetry.addData("Spindexer Position", Spindexer.INSTANCE.getPosition());
        telemetry.addData("Mode",Spindexer.INSTANCE.getPositionType());
        //Spindexer.INSTANCE.setAngle(spinAngle).schedule();
        if(Spindexer.INSTANCE.freePosition()!=-1 && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]).schedule();
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
