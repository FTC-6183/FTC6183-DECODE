package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.SensorTest.SensorColor;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;

@Config
public class Spindexer implements Subsystem {
    public static PIDCoefficients spinCoefficients = new PIDCoefficients(0.001,0,0);
    private double power = 0;
    public static float plUpper = 250;
    public static float plLower = 210;
    public static float glUpper = 170;
    public static float glLower = 145;
    public static float prUpper = 240;
    public static float prLower = 200;
    public static float grUpper = 160;
    public static float grLower = 140;
    final float[] hsvValuesLeft = new float[3];
    final float[] hsvValuesRight = new float[3];


    public double spindexerOffset = 0;
    public static double pValue = 0.005;
    public static double dValue = 0.001;
    public static double kE = 1;
    public static final Spindexer INSTANCE = new Spindexer();

    private Spindexer(){}

    //Pin 0 detects Green
    //Pin 1 detects Purple
    private CRServoEx spinServo = new CRServoEx("spinServo");
    private AnalogInput spinEncoder;

    NormalizedColorSensor leftColorSensor;
    NormalizedColorSensor rightColorSensor;

//    public static DigitalChannel leftColorSensorPurple;
//    public static DigitalChannel leftColorSensorGreen;
//    public static DigitalChannel rightColorSensorPurple;
//    public static DigitalChannel rightColorSensorGreen;
//    public static AnalogInput leftColorSensor;
//    public static AnalogInput rightColorSensor;
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
            int nextPosition = Math.abs(currentPosition.ordinal() - 1 + positions.length) % positions.length;
            currentPosition = positions[nextPosition];
            return positions[nextPosition];
        }

    }
    public enum DetectedColor{
        GREEN,
        PURPLE,
        EMPTY;

        private static DetectedColor getDetectedColor(float[] hsvValuesLeft, float[] hsvValuesRight) {
            DetectedColor currentValue = DetectedColor.EMPTY;
            boolean greenDetection = ((hsvValuesLeft[0]>=glLower) && (hsvValuesLeft[0]<=glUpper)) || ((hsvValuesRight[0]>=grLower) && (hsvValuesRight[0]<=grUpper)) ;
            boolean purpleDetection = ((hsvValuesLeft[0]>=plLower) && (hsvValuesLeft[0]<=plUpper)) || ((hsvValuesRight[0]>=prLower) && (hsvValuesRight[0]<=prUpper));
            if(greenDetection){
                currentValue = DetectedColor.GREEN;
            }
            else if(purpleDetection){
                currentValue = DetectedColor.PURPLE;
            }
            return currentValue;
        }

//        private static DetectedColor getDetectedColor(){
//            DetectedColor currentValue = DetectedColor.EMPTY;
//            boolean purpleDetection = (leftColorSensorPurple.getState() || rightColorSensorPurple.getState());
//            boolean greenDetection = (leftColorSensorGreen.getState() || rightColorSensorGreen.getState());
//
//            if(greenDetection){
//                currentValue = DetectedColor.GREEN;
//            }
//            else if(purpleDetection){
//                currentValue = DetectedColor.PURPLE;
//            }
//            return currentValue;
//        }
    }

    public enum PositionType{
        INTAKE,
        SHOOT
    }
    /*
    private double angleOne = 18.55;
    private double angleTwo = 137.89;
    private double angleThree = 259.2;
     */
    public static Position currentPosition = Position.POSITION_ONE;
    public static PositionType positionType = PositionType.SHOOT;
    public static double intakeAngleOne = 67;//14;
    public static double intakeAngleTwo = 191;//132;
    public static double intakeAngleThree = 315;//254.7;

    public static double shootAngleOne = 14;//67;
    public static double shootAngleTwo = 132;//191;
    public static double shootAngleThree = 254.7;//315;


    public static double intakeAngles[] = new double[]{intakeAngleOne, intakeAngleTwo, intakeAngleThree};
    public static double shootAngles[] = new double[]{shootAngleOne,shootAngleTwo,shootAngleThree};
    private double TICKTODEGREES = (double) 360 / 4000;
    //Zero is Green, One is Purple, Negative One is empty
    public static final int GPP = 21;
    public static final int PGP = 22;
    public static final int PPG = 23;
    public boolean full = false;
    public boolean empty = true;
    private int colorPointer = -1;

    private DetectedColor[] ballAtPosition = new DetectedColor[3];

/*
    private ControlSystem spindexerControl = ControlSystem.builder()
    .angular(AngleType.RADIANS,
                    feedback -> feedback.posPid(spinCoefficients))
     .build();
     */

    /*

    private ControlSystem spindexerControl = ControlSystem.builder()
            .posPid(spinCoefficients)
            .build();

     */

    //TODO: Remember to change the mode of the Color Sensor
    @Override
    public void initialize(){
        leftColorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "leftColorSensor");
        rightColorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "rightColorSensor");
//        leftColorSensorPurple = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "LCSP");
//        rightColorSensorPurple = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "RCSP");
//        leftColorSensorGreen = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "LCSG");
//        rightColorSensorGreen = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "RCSG");
//        leftColorSensor = ActiveOpMode.hardwareMap().get(AnalogInput.class,"leftColorSensor");
//        rightColorSensor = ActiveOpMode.hardwareMap().get(AnalogInput.class, "rightColorSensor");


        spinEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class,"spinEncoder");
        spinServo.setPower(0);
        for(int i = 0; i < ballAtPosition.length; i++){
            ballAtPosition[i] = DetectedColor.EMPTY;
        }
        full = false;
        empty = true;
        colorPointer = -1;
    }
    public DetectedColor readCurrentColor(){
        leftColorSensor.setGain(2);
        rightColorSensor.setGain(2);
        NormalizedRGBA colorsLeft = leftColorSensor.getNormalizedColors();
        NormalizedRGBA colorsRight = rightColorSensor.getNormalizedColors();
        Color.colorToHSV(colorsLeft.toColor(), hsvValuesLeft);
        Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);
        DetectedColor currentColor = DetectedColor.getDetectedColor(hsvValuesLeft,hsvValuesRight);
        if(Math.abs(getCurrentAngleFromEncoder()-intakeAngles[currentPosition.ordinal()])<20){
            ballAtPosition[currentPosition.ordinal()] = currentColor;
        }
        return currentColor;
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
    public PositionType getPositionType(){
        return positionType;
    }
    public void setPositionType(PositionType input){
        positionType = input;
    }
    public DetectedColor[] getBallAtPosition(){
        return ballAtPosition;
    }

    public Position getPosition(){
        return currentPosition;
    }

    public void setCurrentPosition(Position position){
        Spindexer.currentPosition = position;
    }


    public Command setToPosition(Position position){
        Spindexer.currentPosition = position;
        if(positionType == PositionType.INTAKE){
            return new InstantCommand(setAngle(intakeAngles[position.ordinal()]));
        }
        else if(positionType == PositionType.SHOOT){
            return new InstantCommand(setAngle(shootAngles[position.ordinal()]));
        }
        return new InstantCommand(setAngle(intakeAngles[position.ordinal()]));
    }


    public Command nextPosition(){
        currentPosition = currentPosition.next();
        return setToPosition(currentPosition);
    }

    //Custom Spindexer Implementation
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
        return new InstantCommand(()-> power = pValue * wrapDeg(getAbsoluteAngleFromEncoder()-goal)
        );

    }
    public int freePosition(){
   int position = currentPosition.ordinal();
   for (int i = 0; i < ballAtPosition.length; i++) {
       if(ballAtPosition[position]==DetectedColor.EMPTY) {
           return position;
       }
       position++;
       position = position%3;
   }
   full = true;
   return -1; //No Free Positions
    }

    public void setColor(Position position, DetectedColor Color){
        ballAtPosition[currentPosition.ordinal()] = Color;
    }

//    public double getLeftColor(){
//        return leftColorSensor.getVoltage() / 3.3 * 360;
//    }
//    public double getRightColor(){
//        return rightColorSensor.getVoltage() / 3.3 * 360;
//    }

    /*
    public int filledPosition(){
        int position = currentPosition.ordinal();
        for (int i = 0; i < ballAtPosition.length; i++) {
            if(ballAtPosition[position]==DetectedColor.GREEN||ballAtPosition[position]==DetectedColor.PURPLE) {
                return position;
            }
            position++;
            position = position%3;
        }
        return -1; //No Filled
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
    */

    @Override
    public void periodic() {
        spinServo.setPower(power);
    }
}
