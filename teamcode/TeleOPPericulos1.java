package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.NEDBOT;

@TeleOp
public class TeleOPPericulos1 extends LinearOpMode {

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(130, 0, 9, 16.5);   //100 0 20 15.9
    enum Mode{
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    Vector2d TargetAVector = new Vector2d(50,-5);     ////////////POZITIE TRAS//////////////////////
    double targetAHeading = Math.toRadians(0);             ////////////UNGHI TRAS////////////////////////


    boolean mod =true;
    boolean modtras= true;
    boolean modintake = false;

    @Override
    public void runOpMode() throws InterruptedException{


        NEDBOT drive = new NEDBOT(hardwareMap);

        drive.prins.setPosition(0.8);
        drive.brat.setTargetPosition(0);
        drive.brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        drive.brat.setPower(0.3);
        drive.tragaci.setPosition(0.8);

        drive.setPoseEstimate(new Pose2d(65,20,Math.toRadians(180)));    //180




        waitForStart();

        if(isStopRequested())
            return;

        while(opModeIsActive() && !isStopRequested())
        {

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("Heading", poseEstimate.getHeading());
            telemetry.addData("brat.ticks", drive.brat.getCurrentPosition());
            telemetry.addData("prins_pozitie", drive.prins.getPosition());
            telemetry.addData("tragaci_pozitie", drive.tragaci.getPosition());
            telemetry.addData("shooter_viteza", drive.m6.getVelocity());
            telemetry.update();

            switch(currentMode)
            {
                case DRIVER_CONTROL:


/////////////////////////////////////////////////////////   POWERSHOTS -- ENDGAME   ///////////////////////////////////////////////////
                    if(gamepad1.y) {
                        drive.setPoseEstimate(new Pose2d(52.8, 30.2, Math.toRadians(0)));   //53.62  -18.39  0
                        Trajectory POWERSHOTS = drive.trajectoryBuilder(poseEstimate)
                                .addDisplacementMarker(() ->
                                {
                                    drive.m6.setVelocity(1550);
                                })
                                .lineToLinearHeading(new Pose2d(50.80, -5.0, Math.toRadians(-10)))
                                .build();
                        drive.followTrajectory(POWERSHOTS);
                        modtras = false;
                        mod=false;
                        currentMode = Mode.AUTOMATIC_CONTROL;

                    }

                    // drive.m6.setVelocity(1500);


                    if(gamepad1.x)
                    {
                        drive.rintake.setPosition(0);
                        modintake = false;
                    }

                    if(gamepad1.a)
                    {
                        drive.rintake.setPosition(0.85);
                        modintake = true;
                    }


                    if(gamepad1.b)
                    {
                        drive.rintake.setPosition(0.70);
                        modintake = true;
                    }




//////////////////////////////////////////////////////////   TRAS -- MANUAL  ////////////////////////////////////////////////
                    if(gamepad1.dpad_down)
                    {
                        drive.m6.setVelocity(1550);
                        mod=false;
                        modtras=false;
                    }

                    if(gamepad1.dpad_up)
                    {
                        drive.m6.setVelocity(1575);
                        mod=false;
                    }




/////////////////////////////////////////////////////////////    TRAS -- AUTO    ////////////////////////////////////////////////////

                    if(gamepad1.left_bumper)
                    {
                        drive.m6.setVelocity(1700);
                        mod=false;
                    }


                    if(gamepad1.right_bumper)
                    {
                        if(modtras==true) {

                            for(int i=1;i<=3;i++) {
                                drive.tragaci.setPosition(0.5);  //SERVO//////////////////////
                                sleep(150);
                                drive.tragaci.setPosition(0.8);
                                sleep(150);
                            }
                            drive.m6.setVelocity(0);
                            //drive.setPoseEstimate(new Pose2d(53.62,-18.39,Math.toRadians(0)));
                            mod=true;
                        }

                        if(modtras==false) {
                            drive.tragaci.setPosition(0.5);
                            sleep(150);
                            drive.tragaci.setPosition(0.8);
                            sleep(150);
                            drive.turn(Math.toRadians(-7));     ///////////////////////
                            drive.tragaci.setPosition(0.5);
                            sleep(150);
                            drive.tragaci.setPosition(0.8);
                            sleep(150);
                            drive.turn(Math.toRadians(-7));    //////////////////////
                            drive.tragaci.setPosition(0.5);
                            sleep(150);
                            drive.tragaci.setPosition(0.8);
                            sleep(150);
                            drive.m6.setVelocity(0);
                            modtras=true;
                            mod=true;
                        }

                    }



///////////////////////////////////////////////////   MERS   //////////////////////////////////////////////


                    if(mod==true)
                    {
                        Vector2d input = new Vector2d(
                                -gamepad1.left_stick_x,   //1.5
                                gamepad1.left_stick_y
                        ).rotated(-poseEstimate.getHeading());

                        // Pass in the rotated input + right stick value for rotation
                        // Rotation is not part of the rotated input thus must be passed in separately
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        input.getX(),
                                        input.getY(),
                                        -gamepad1.right_stick_x
                                )

                        );

                    }

                    if(mod==false)
                    {
                        Vector2d input = new Vector2d(
                                -gamepad1.left_stick_x/2.6,  ///2.5
                                gamepad1.left_stick_y/2.6
                        ).rotated(-poseEstimate.getHeading());

                        // Pass in the rotated input + right stick value for rotation
                        // Rotation is not part of the rotated input thus must be passed in separately
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        input.getX(),
                                        input.getY(),
                                        -gamepad1.right_stick_x/2.6
                                )

                        );
                    }



                    ////////////////////////////////////////////////////////  D2   //////////////////////////////////////////////

                    if(-gamepad2.left_stick_y > 0.1)
                        drive.intake.setPower(1);
                    else
                    if(-gamepad2.left_stick_y< -0.1)
                        drive.intake.setPower(-1);
                    else
                        drive.intake.setPower(0);

                    if(gamepad2.right_trigger > 0.1)
                    {
                        drive.prins.setPosition(0.83);
                    }

                    if(gamepad2.left_trigger > 0.1)
                    {
                        drive.prins.setPosition(0);
                    }

                    if(gamepad2.dpad_up)
                    {
                        drive.brat.setTargetPosition((int)((2786.2/360)*70));
                        drive.brat.setPower(0.3);
                    }

                    if(gamepad2.dpad_down)
                    {
                        drive.brat.setTargetPosition((int)((2786.2/360)*170));
                        drive.brat.setPower(0.5);
                    }

                    if(gamepad2.x)
                    {
                        drive.brat.setTargetPosition(0);
                        drive.brat.setPower(0.3);
                        while(drive.brat.isBusy())
                        {
                            Vector2d input = new Vector2d(
                                    0,
                                    0
                            ).rotated(0);

                        }
                        drive.prins.setPosition(0.83);
                    }
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    break;
                case AUTOMATIC_CONTROL:
                    if(gamepad1.x){
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;

                    }
                    if(!drive.isBusy()){

                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;




            }


        }


    }



}
