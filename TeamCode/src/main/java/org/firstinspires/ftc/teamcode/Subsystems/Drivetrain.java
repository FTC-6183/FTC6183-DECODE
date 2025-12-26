package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.driving.RobotCentric;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

public class Drivetrain implements Subsystem {
    public static final Drivetrain INSTANCE = new Drivetrain(); // Creates a singleton instance of the Drivetrain ensuring that all parts
    private Drivetrain(){}

    private MotorEx frontLeftMotor = new MotorEx("fl").brakeMode();
    private MotorEx backLeftMotor = new MotorEx("bl").brakeMode();
    private MotorEx frontRightMotor = new MotorEx("fr").brakeMode().reversed();
    private MotorEx backRightMotor = new MotorEx("br").brakeMode().reversed();
    private IMUEx imu = new IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed();
    
    public Command startRobotDrive() {
        {
            return new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX()
            );
        }
    }

        public Command startFieldDrive () {
            {
                return new MecanumDriverControlled(
                        frontLeftMotor,
                        frontRightMotor,
                        backLeftMotor,
                        backRightMotor,
                        Gamepads.gamepad1().leftStickY().negate(),
                        Gamepads.gamepad1().leftStickX(),
                        Gamepads.gamepad1().rightStickX(),
                        new FieldCentric(imu)
                );
            }

        }
    public void run(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
    }



}
