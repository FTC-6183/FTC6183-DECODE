package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "TestDrive")
public class TestDrive extends NextFTCOpMode {
    public TestDrive(){
        addComponents(
                new SubsystemComponent(Drivetrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        Drivetrain.INSTANCE.startFieldDrive().schedule();
    }
    @Override
    public void onUpdate(){
        BindingManager.update();
        telemetry.update();
    }
    @Override
    public void onStop(){
    }

}
