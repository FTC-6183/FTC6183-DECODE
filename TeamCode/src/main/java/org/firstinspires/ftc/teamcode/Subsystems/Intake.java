package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake(){}
    private double on = 1;
    private double off = 0;
    private double out = -1;

    private MotorEx intakeMotor = new MotorEx("intake");

    public Command on(){
        return new SetPower(intakeMotor, on);
    }

    public Command idle(){
        return new SetPower(intakeMotor, off);
    }
    public Command reverse(){
        return new SetPower(intakeMotor, out);
    }

}
