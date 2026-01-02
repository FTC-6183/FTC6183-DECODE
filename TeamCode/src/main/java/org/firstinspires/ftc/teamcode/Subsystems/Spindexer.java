package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;

@Config
public class Spindexer implements Subsystem {
    public static PIDCoefficients spinCoefficients = new PIDCoefficients(0.001,0,0);
    private double power = 0;

    public double spindexerOffset = 0;
    public static double pValue = 0.006;
    public static final Spindexer INSTANCE = new Spindexer();

    private Spindexer(){}

    //Pin 0 detects Green
    //Pin 1 detects Purple
    private CRServoEx spinServo = new CRServoEx("spinServo");

    private DigitalChannel leftColorSensorPin0;
    private DigitalChannel leftColorSensorPin1;
    private DigitalChannel rightColorSensorPin0;
    private DigitalChannel rightColorSensorPin1;

    private AnalogInput spinEncoder;
    public enum Position{
        POSITION_ONE,
        POSITION_TWO,
        POSITION_THREE;

        public static Position next(){
            Position[] positions = values();
            int nextPosition = (currentPosition.ordinal()+1) % positions.length;
            currentPosition = positions[nextPosition];
            return positions[nextPosition];
        }
        public static Position previous(){
            Position[] positions = values();
            int nextPosition = Math.abs(currentPosition.ordinal()-1) % positions.length;
            currentPosition = positions[nextPosition];
            return positions[nextPosition];
        }

    }
    private static Position currentPosition = Position.POSITION_ONE;
    /*
    private double angleOne = 18.55;
    private double angleTwo = 137.89;
    private double angleThree = 259.2;
     */
    private double intakeAngleOne = 20;
    private double intakeAngleTwo = 140;
    private double intakeAngleThree = 270;

    private double shootAngleOne = 75;
    private double shootAngleTwo = 193;
    private double shootAngleThree = 320;


    private double intakeAngles[] = new double[]{intakeAngleOne, intakeAngleTwo, intakeAngleThree};
    private double shootAngles[] = new double[]{shootAngleOne,shootAngleTwo,shootAngleThree};
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

    private ControlSystem spindexerControl = ControlSystem.builder()
    .angular(AngleType.DEGREES,
                    feedback -> feedback.posPid(spinCoefficients))
     .build();


    //TODO: Remember to change the mode of the Color Sensor
    @Override
    public void initialize(){
        /*
        leftColorSensorPin0 = ActiveOpMode.hardwareMap().digitalChannel.get("leftPin0");
        leftColorSensorPin1 = ActiveOpMode.hardwareMap().digitalChannel.get("leftPin1");
        rightColorSensorPin0 = ActiveOpMode.hardwareMap().digitalChannel.get("rightPin0");
        rightColorSensorPin1 = ActiveOpMode.hardwareMap().digitalChannel.get("rightPin1");
         */
        spinEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class,"spinEncoder");
        spinServo.setPower(0);

        for(int i = 0; i < ballAtPosition.length; i++){
            ballAtPosition[i] = EMPTY;
        }
        full = false;
        empty = true;
        colorPointer = -1;

    }
    public double getCurrentAngleFromEncoder() {
        if (spinEncoder == null) return 0;
        return (spinEncoder.getVoltage() / 3.3) * 360;
    }
    public double getAbsoluteAngleFromEncoder(){
        double angle = getCurrentAngleFromEncoder();
        double wrappedAngle = ((angle+180)%360+360)%360-180;
        return wrappedAngle;
    }

    /*
    public double getAbsoluteAngleFromEncoder(){
        double angle = getCurrentAngleFromEncoder()-spindexerOffset;
        double wrappedAngle = ((angle+180)%360+360)%360-180;
        return wrappedAngle;
    }
    */
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

    public Command setToPosition(Position position, String type){
        currentPosition = position;
        if(type.equals("Intake")){
            return new InstantCommand(setAngle(intakeAngles[position.ordinal()]));
        }
        else if(type.equals("Shoot")){
            return new InstantCommand(setAngle(shootAngles[position.ordinal()]));
        }
        return new InstantCommand(setAngle(intakeAngles[position.ordinal()]));
    }

    /*public Command setToAngle(double angle){
        return new RunToPosition(spindexerControl,angle);
    }
    */

    public Command nextPosition(String type){
        currentPosition = currentPosition.next();
        return setToPosition(currentPosition.next(),"Intake");
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
            return setToPosition(Position.values()[freePosition()],"Intake").requires(this);
        }
        else{
            full = true;
        }
        return new NullCommand();
    }
    public Command setToFilledPosition(){
        if(filledPosition()!=-1){
            return setToPosition(Position.values()[filledPosition()],"Intake").requires(this);
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
            setToPosition(Position.values()[findColor(color[colorPointer])],"Intake").requires(this);
        });
    }
    public static double wrapDeg(double angle) {
        angle %= 360.0;
        if (angle >= 180.0) angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }
    public double getPower(){
        return power;

    }
    public Command setAngle(double goal){
        return new InstantCommand(()-> power = pValue * wrapDeg(getAbsoluteAngleFromEncoder()-goal));

    }
    public Command stop(){
        return new NullCommand().requires(this);
    }

    @Override
    public void periodic() {
        spinServo.setPower(power);
    }
}
