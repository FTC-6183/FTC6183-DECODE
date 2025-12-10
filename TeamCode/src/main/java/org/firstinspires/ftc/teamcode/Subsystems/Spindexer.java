package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.opencv.ml.EM;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

public class Spindexer implements Subsystem {
    public static final Spindexer INSTANCE = new Spindexer();
    private Spindexer(){}

    private CRServoEx spinServo = new CRServoEx("spin");
    //Pin 0 detects Green
    //Pin 1 detects Purple
    private DigitalChannel leftColorSensorPin0;
    private DigitalChannel leftColorSensorPin1;
    private DigitalChannel rightColorSensorPin0;
    private DigitalChannel rightColorSensorPin1;

    private MotorEx spinEncoder = new MotorEx("spinEncoder");
    public enum Position{
        POSITION_ONE,
        POSITION_TWO,
        POSITION_THREE;

        public Position next(){
            Position[] positions = values();
            int nextPosition = (currentPosition.ordinal()+1)% positions.length;
            return positions[nextPosition];
        }

    }
    private static Position currentPosition = Position.POSITION_ONE;
    private double angleOne = 0;
    private double angleTwo = 120;
    private double angleThree = 240;
    private double angles[] = new double[3];
    private double TICKTODEGREES = (double) 360 / 4000;
    //Zero is Green, One is Purple, Negative One is empty

    public static final int GREEN = 0;
    public static final int PURPLE = 1;
    public static final int EMPTY = -1;
    public static final int GPP = 21;
    public static final int PGP = 22;
    public static final int PPG = 23;
    public boolean full = false;
    public boolean empty = true;
    private int colorPointer = -1;

    private int[] ballAtPosition = new int[3];
    private PIDCoefficients coefficients = new PIDCoefficients(0,0,0);
    private ControlSystem spindexerControl = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                    feedbackElementBuilder -> feedbackElementBuilder.posPid(coefficients))
            .build();
    //TODO: Remember to change the mode of the Color Sensor
    @Override
    public void initialize(){
        leftColorSensorPin0 = ActiveOpMode.hardwareMap().digitalChannel.get("leftPin0");
        leftColorSensorPin1 = ActiveOpMode.hardwareMap().digitalChannel.get("leftPin1");
        rightColorSensorPin0 = ActiveOpMode.hardwareMap().digitalChannel.get("rightPin0");
        rightColorSensorPin1 = ActiveOpMode.hardwareMap().digitalChannel.get("rightPin1");
        for(int i = 0; i < ballAtPosition.length; i++){
            ballAtPosition[i] = EMPTY;
        }
        full = false;
        empty = true;
        colorPointer = -1;

    }
    public int[] getBallAtPosition(){
        return ballAtPosition;
    }

    public Position getPosition(){
        return currentPosition;
    }
    public void readCurrentValue(){
        int color = EMPTY;
        if(leftColorSensorPin0.getState()||rightColorSensorPin0.getState()){
            color = GREEN;
        }
        else if(leftColorSensorPin1.getState()||rightColorSensorPin1.getState()){
            color = PURPLE;
        }
        ballAtPosition[currentPosition.ordinal()] = color;
    }
    public int findColor(double color){
        for (int i = 0; i < ballAtPosition.length; i++) {
            if(ballAtPosition[i]==color){
                return i;
            }
        }
        return -1;
    }
    public Command setToPosition(Position position){
        currentPosition = position;
        switch(position){
            case POSITION_ONE:
                return new RunToPosition(spindexerControl,angleOne);
            case POSITION_TWO:
                return new RunToPosition(spindexerControl,angleTwo);
            case POSITION_THREE:
                return new RunToPosition(spindexerControl,angleThree);
        }
        return new RunToPosition(spindexerControl,angleOne);
    }
    public Command nextPosition(){
        return setToPosition(currentPosition.next());
    }
    public int freePosition(){
        int position = currentPosition.ordinal();
        for (int i = 0; i < ballAtPosition.length; i++) {
            if(ballAtPosition[position]==EMPTY) {
                return position;
            }
            position++;
            position = position%3;
        }
        return -1; //No Free Positions
    }
    public int filledPosition(){
        int position = currentPosition.ordinal();
        for (int i = 0; i < ballAtPosition.length; i++) {
            if(ballAtPosition[position]==GREEN||ballAtPosition[position]==PURPLE) {
                return position;
            }
            position++;
            position = position%3;
        }
        return -1; //No Free Positions
    }
    public Command setToFreePosition(){
        if(freePosition()!=-1){
            return setToPosition(Position.values()[freePosition()]).requires(this);
        }
        else{
            full = true;
        }
        return new NullCommand();
    }
    public Command setToFilledPosition(){
        if(filledPosition()!=-1){
            return setToPosition(Position.values()[filledPosition()]).requires(this);
        }
        else{
            empty = true;
        }
        return new NullCommand();
    }
    public Command setToFilledPosition(int pattern){
        return new InstantCommand(() -> {
            colorPointer = (colorPointer + 1) % 3;
            int[] color;
            switch (pattern) {
                case GPP:
                    color = new int[]{GREEN, PURPLE, PURPLE};
                case PGP:
                    color = new int[]{PURPLE, GREEN, PURPLE};
                case PPG:
                    color = new int[]{PURPLE, PURPLE, GREEN};
                default:
                    color = new int[]{GREEN, GREEN, GREEN};
            };
            setToPosition(Position.values()[findColor(color[colorPointer])]).requires(this);
        });
    }
    public Command stop(){
        return new NullCommand().requires(this);
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();
        spinServo.setPower(spindexerControl.calculate(spinEncoder.getState().times(TICKTODEGREES)));
        readCurrentValue();
    }
}
