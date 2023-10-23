package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.lang.Math;
import java.util.Random;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.util.Range;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Stack;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name = "Odometry Testing - Tensor Flow", group = "Concept")


public class Odometry_Writing extends LinearOpMode
{
    // Control Hub
    /// Wheel Motors
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor backLeftOdometry = null;
    private DcMotor backRightOdometry = null;
    private DcMotor centerOdometry = null;
        //ODOMETRY VARIABLES:

    //robot specific:
    private final double SL = 5.50;
    private final double SR = 6.74;
    private  final double SS = 0.01;
    /**
     * ^INSTRUCTIONS:
     * SL is the distance in inches horizontally between the back left odometry pod and the center odometry pod.
       MEASUREMENT DOESN'T CONNECT THE 2!!
     * SR is the distance in inches orrizontally between the back right odometry pod and the center odometry pod.
       (SL, but the other side)
     * SS is Front back distance between the center odometry pod and invisible line between the 2 rear pods.
     */

    // position variables
    private double[] position_Vector = {0 , 0};
    private final double tickConverter = 106.157;
    //tick per inch converter^ (tick per rotation / circumference)

    // first 2 variables are position, second two are previous

    private double rightWheelTravel;
    private double leftWheelTravel;
    private double centerWheelTravel;
    private double tempAngle;

    private double leftWheelConstant = 0;
    private double rightWheelConstant = 0;
    private double centerWheelConstant = 0;

    private double previousLeftWheelReading = 0;
    private double previousRightWheelReading = 0;
    private double previousCenterWheelReading = 0;

    private double ticksToInches = 0;





    public static double[] odometry (double SL, double SR, double SS, double rightWheelTravel, double leftWheelTravel, double centerWheelTravel,
                                 double[] position_Vector )
    {


        double tempAngle;
        double[] previous_Position_Vector;
        double[] temp_Position_Vector = {0,0};





        //reseting 'previous' variables
        previous_Position_Vector = position_Vector;

        tempAngle = (leftWheelTravel - rightWheelTravel)/(SL + SR);

        if (tempAngle == 0)
        {
            position_Vector[0] = previous_Position_Vector[0] + centerWheelTravel;
            position_Vector[1] = previous_Position_Vector[1] + rightWheelTravel;

        }
        else
        {
            temp_Position_Vector[0] = 2 * Math.sin(tempAngle/2) * ((centerWheelTravel / tempAngle) + SS);
            temp_Position_Vector[1] = 2 * Math.sin(tempAngle/2) * ((rightWheelTravel/tempAngle) + SR);

            position_Vector[0] = previous_Position_Vector[0] + temp_Position_Vector[0];
            position_Vector[1] = previous_Position_Vector[1] + temp_Position_Vector[1];

        }


        return position_Vector;



    }

    public static void Basic_pathfinding (double[] position_Vector, double[] target_Position_Vector)
    {
        double slope;
        double start_Position_Vector;
        double targetY;
        double actualY;




        slope = (position_Vector[1]-target_Position_Vector[1]) / (position_Vector[0]-target_Position_Vector[0]);

        targetY = slope * (position_Vector[0] - target_Position_Vector[0]) + target_Position_Vector[1];

        if (targetY > position_Vector[1])
        {

        }
        if (targetY < position_Vector[1])
        {

        }














    }






    @Override

    public void runOpMode()
    {
        // Initialize connection to motors/odometry
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        backLeftOdometry = backLeftMotor;
        backRightOdometry = backRightMotor;
        centerOdometry = frontLeftMotor;

        // Set direction to the motors (may need to change depending on orientation of robot)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset the encoder count - initialize to 0
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/


        /** Wait for the game to begin */

        telemetry.addLine("Working.");
        telemetry.update();



        waitForStart();

        if (opModeIsActive()) {


            long initTime = System.currentTimeMillis();
            long currentTime = 0;

            while (opModeIsActive())
            {
                previousLeftWheelReading = leftWheelConstant;
                previousRightWheelReading = rightWheelConstant;
                previousCenterWheelReading = centerWheelConstant;

                leftWheelConstant = backLeftOdometry.getCurrentPosition();
                rightWheelConstant = backRightOdometry.getCurrentPosition();
                centerWheelConstant = centerOdometry.getCurrentPosition();

                leftWheelTravel = leftWheelConstant - previousLeftWheelReading;
                rightWheelTravel = rightWheelConstant - previousRightWheelReading;
                centerWheelTravel = centerWheelConstant - previousCenterWheelReading;

                leftWheelTravel /= tickConverter;
                rightWheelTravel /= tickConverter;
                centerWheelTravel /= tickConverter;


                position_Vector = odometry(SL , SR, SS , rightWheelTravel , leftWheelTravel, centerWheelTravel, position_Vector );

                telemetry.addLine("X coord " + position_Vector[0]);
                telemetry.addLine("Y coord " + position_Vector [1]);
                telemetry.update();



                //insert STEPCODE here

            }

            // Stop function - cancel all power to the motors
            frontLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            backRightMotor.setPower(0.0);




        }
    }
}