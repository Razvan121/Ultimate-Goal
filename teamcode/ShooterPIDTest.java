package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


@TeleOp
public class ShooterPIDTest extends LinearOpMode {
    // Copy your PIDF Coefficients here
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 2, 13.1);

    double unghi=0;
    double pas = 0.001;
    boolean putere = false;
    double power = 0.80;

    @Override
    public void runOpMode() throws InterruptedException {
        // SETUP MOTOR //
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "m6");
        Servo tragaci = hardwareMap.get(Servo.class,"tragaci");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"intake");

        // Reverse as appropriate
        intake.setDirection(DcMotorEx.Direction.REVERSE);
         myMotor.setDirection(DcMotorEx.Direction.REVERSE);


            tragaci.setPosition(0.4);


        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        // Turn on RUN_USING_ENCODER
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set PIDF Coefficients with voltage compensated feedforward value
        myMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));

        // Insert whatever other initialization stuff you do here

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {



if(gamepad1.x)
    tragaci.setPosition(0);
else
    tragaci.setPosition(0.4);

if(gamepad1.y)
{
    putere = !putere;
    if(putere==true)
        intake.setPower(power);
    else
        intake.setPower(0);

}




        //    if(gamepad1.y)
         //       myMotor.setVelocity(1700);

            if(gamepad1.a)
                myMotor.setVelocity(1900);
          //  if(gamepad1.x)
             //   myMotor.setVelocity(5400*0.9);

            // Do your opmode stuff
            telemetry.addData("motor velocity", myMotor.getVelocity());
            telemetry.addData("Pozitie servo", tragaci.getPosition());

                    telemetry.update();
        }
    }

    public void suge (){

    }
}