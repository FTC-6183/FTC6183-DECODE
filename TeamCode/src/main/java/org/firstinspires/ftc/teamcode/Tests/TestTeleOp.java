package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "TestTele")
public class TestTeleOp extends NextFTCOpMode {
    Robot robot = new Robot(Aliance.RED);
    public TestTeleOp(){
        addComponents(
                new SubsystemComponent(robot),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        /*
        Gamepads.gamepad1().a()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(robot.intakeOn())
                .whenBecomesFalse(robot.intakeOff());
        Gamepads.gamepad1().x()
                .whenBecomesTrue(robot.shootOneSequence());
        Gamepads.gamepad1().y()
                .whenBecomesTrue(robot.shootAllSequence());
                */
        robot.drive();
    }
    @Override
    public void onUpdate(){
        BindingManager.update();
        robot.periodic();
        telemetry.update();
    }
    @Override
    public void onStop(){
    }

}
