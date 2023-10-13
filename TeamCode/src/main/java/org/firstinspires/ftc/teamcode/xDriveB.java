package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp
public class xDriveB extends LinearOpMode {
    private DcMotorEx northWheel;
    private DcMotorEx southWheel;
    private DcMotorEx westWheel;
    private DcMotorEx eastWheel;
    private DcMotorEx neck;
    private DcMotorEx shoulder;
    private Servo wrist;
    private Servo leftGrip;
    private Servo rightGrip;

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections =
            RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections =
            RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length -1;
    static float TRIGGER_THRESHOLD = 0.2f;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;


    @Override
    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu");
        northWheel = hardwareMap.get(DcMotorEx.class, "northWheel");
        southWheel = hardwareMap.get(DcMotorEx.class, "southWheel");
        westWheel = hardwareMap.get(DcMotorEx.class, "westWheel");
        eastWheel = hardwareMap.get(DcMotorEx.class, "eastWheel");
        neck = hardwareMap.get(DcMotorEx.class, "neck");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward

        boolean justChangedLogoDirection= false;
        boolean justChangedUsbDirection= false;

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        //variables for drivetrain
        double forwardPower;
        double strafePower;
        double turnPower;

        //variables for arm
        double neckInput;
        double shoulderInput;

        //variable for the wrist
        double wristInput = 0;

        //initializes positions for claw grips
        leftGrip.setPosition(0);
        rightGrip.setPosition(0);

        //set the directions of motors
        northWheel.setDirection(DcMotor.Direction.FORWARD);
        southWheel.setDirection(DcMotor.Direction.REVERSE);
        westWheel.setDirection(DcMotor.Direction.FORWARD);
        eastWheel.setDirection(DcMotor.Direction.REVERSE);

        //sets BulkCachingMode to "AUTO"
        //Minimize the number of discrete read commands, by performing bulk-reads
        // and then returning values that have been cached.
        for (LynxModule module: allHubs){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while (opModeIsActive()) {
            //Sets variables for driving on gamepad1
            forwardPower = gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = gamepad1.right_stick_x;

            //changes variables for the wrist with inputs received from the dpad on gamepad2
            while (gamepad2.dpad_up) {
                wristInput = wrist.getPosition() + 0.1;
            }
            while (gamepad2.dpad_down) {
                wristInput = wrist.getPosition() - 0.1;
            }

            //Sets power of wheels based on double variables
            northWheel.setPower(forwardPower + turnPower);
            southWheel.setPower(forwardPower + turnPower);
            westWheel.setPower(strafePower - turnPower);
            eastWheel.setPower(strafePower - turnPower);

            //Sets variables for inputs received from gamepad2
            neckInput = gamepad2.left_stick_x;
            shoulderInput = gamepad2.right_stick_y;

            //Sets power of the neck and shoulder based on double variables
            neck.setPower(neckInput);
            shoulder.setPower(shoulderInput);

            //changes the current wrist position based on the state of the "wristInput" variable
            wrist.setPosition(wristInput);


            //changes current grip position based on inputs received from A and B buttons on gamepad2
            if (gamepad2.a) {
                //returns grip to retracted position
                leftGrip.setPosition(0);
                rightGrip.setPosition(0);
            }
            if (gamepad2.b) {
                //sets grip to grab position
                leftGrip.setPosition(0.5);
                rightGrip.setPosition(0.5);
            }
        }
    }
}
