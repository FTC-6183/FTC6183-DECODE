package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer(){};

    private ServoEx leftFork = new ServoEx("leftFork");
    private ServoEx rightFork = new ServoEx("rightFork");

    private double up = 1;
    private double down = 0;
    public Command transferUp(){
        return new SetPosition(leftFork,up).and(new SetPosition(rightFork,up));

    }
    public Command transferDown(){
        return new SetPosition(leftFork,down).and(new SetPosition(rightFork,down));
    }


}
