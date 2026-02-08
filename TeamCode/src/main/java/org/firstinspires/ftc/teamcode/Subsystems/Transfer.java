package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfer implements Subsystem {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer(){};
    @Override public void initialize(){
        leftFork.setPosition(leftDown);
        rightFork.setPosition(rightDown);
    }

    private ServoEx leftFork = new ServoEx("leftFork");
    private ServoEx rightFork = new ServoEx("rightFork");

    private double leftUp = 1;
    private double rightUp = 0.1;

    private double leftDown = 0.1;
    private double rightDown = 1;
    public Command transferUp(){
        return new SetPosition(leftFork,leftUp).and(new SetPosition(rightFork,rightUp));

    }
    public boolean isTransferDown(){
        return ((leftFork.getPosition() == leftDown) && (rightFork.getPosition() == rightDown));
    }
    public Command transferDown(){
        return new SetPosition(leftFork, leftDown).and(new SetPosition(rightFork, rightDown));
    }

    public String servoPosition(){
        return "Left Servo Position is " + leftFork.getPosition()+" Right Servo Position is " + rightFork.getPosition();
    }



}
