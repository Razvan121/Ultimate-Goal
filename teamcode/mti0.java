package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.NEDBOT;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class mti0 extends LinearOpMode {

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(130, 0, 9, 16.5);
    int nrAUTO;

    OpenCvInternalCamera phoneCam;
    mti0.SkystoneDeterminationPipeline pipeline;

    public DcMotorEx brat;
    public Servo prins;
    public DcMotorEx intake;
    public Servo tragaci;
    public DcMotorEx m6;
    public Servo telefon;

    @Override
    public void runOpMode()
    {

        NEDBOT drive = new NEDBOT(hardwareMap);

        brat= hardwareMap.get(DcMotorEx.class,"brat");
        prins = hardwareMap.get(Servo.class,"prins");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        tragaci= hardwareMap.get(Servo.class,"tragaci");
        m6 = hardwareMap.get(DcMotorEx.class, "m6");
        telefon = hardwareMap.get(Servo.class,"telefon");



        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m6.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // m6.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        m6.setDirection(DcMotorEx.Direction.REVERSE);







        for (
                LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = m6.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        m6.setMotorType(motorConfigurationType);
        // Turn on RUN_USING_ENCODER
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Coefficients with voltage compensated feedforward value
        m6.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));


        /////////////////////////////////////////////   A1-Alberto Bosinceanu  ///////////////////////////////////////////////////

        Trajectory SHOOT1 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition((int)((2786.2/360)*70));
                    brat.setPower(0.5);
                })
                .splineToConstantHeading(new Vector2d(41,-15),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52.80,-15),Math.toRadians(0))   //53 1 0   57.80  6.25
                .build();

        Trajectory WOBBLE1_1 = drive.trajectoryBuilder(SHOOT1.end().plus(new Pose2d(0,0,Math.toRadians(-10))))
                .addDisplacementMarker(() ->{
                    brat.setTargetPosition((int)((2786.2/360)*170));
                    brat.setPower(0.3);
                })
                .lineToLinearHeading(new Pose2d(56,-21,Math.toRadians(-6.0)))   //-20
                .addDisplacementMarker(() ->{
                    prins.setPosition(0);
                })
                .build();



        Trajectory INTERMEDIAR = drive.trajectoryBuilder(WOBBLE1_1.end())
                .lineToLinearHeading(new Pose2d(30,-10,Math.toRadians(0)))
                .build();



        Trajectory PARK1 = drive.trajectoryBuilder(INTERMEDIAR.end(),true)
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);
                    prins.setPosition(0.8);
                })
                .splineToLinearHeading(new Pose2d(70, 7,Math.toRadians(180)),Math.toRadians(180))
                .build();

        ///////////////////////////////////////   A2-Marian Tiganu   ///////////////////////////////////////////////////////////


        Trajectory SHOOT2_1 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition((int)((2786.2/360)*70));
                    brat.setPower(0.5);
                })
                .splineToConstantHeading(new Vector2d(41,-15),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51.80,-15),Math.toRadians(0))
                .build();


        Trajectory PozIntake = drive.trajectoryBuilder(SHOOT2_1.end())

                .addDisplacementMarker(()->{
                    drive.rintake.setPosition(0.85);
                    drive.intake2.setPower(-1);
                })
                .splineToConstantHeading(new Vector2d(57,9),Math.toRadians(0))
                .build();

        Trajectory discBACK = drive.trajectoryBuilder(PozIntake.end())
                .back(25)
                .build();

        Trajectory SHOOT2_2 = drive.trajectoryBuilder(discBACK.end())
                .addDisplacementMarker(()->{
                    drive.rintake.setPosition(0);
                    drive.intake2.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(52,-15,Math.toRadians(0)))
                .build();


        Trajectory WOBBLE2_1 = drive.trajectoryBuilder(SHOOT2_2.end())
                .addDisplacementMarker(() ->{
                    brat.setTargetPosition((int)((2786.2/360)*170));
                    brat.setPower(0.3);
                })
                .lineToLinearHeading(new Pose2d(85,5,Math.toRadians(0)))
                .addDisplacementMarker(() ->{
                    prins.setPosition(0);
                })
                .build();




        Trajectory PARK2 = drive.trajectoryBuilder(WOBBLE2_1.end(),true)
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);
                    prins.setPosition(0.8);
                })
                .splineToLinearHeading(new Pose2d(67.5, 4,Math.toRadians(170)),Math.toRadians(170))
                .build();

        //////////////////////////////////////////////////   A3 -gaza  /////////////////////////////////////////////////////

        Trajectory WOBBLE3_1 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition((int)((2786.2/360)*165));
                    brat.setPower(0.3);
                })
                .splineToConstantHeading(new Vector2d(33,3),Math.toRadians(0)) //-13
                .splineToConstantHeading(new Vector2d(112,3),Math.toRadians(0)) //-13
                .addDisplacementMarker(() -> {
                    prins.setPosition(0);
                    drive.m6.setVelocity(1750);
                })
                .build();

        Trajectory SHOOT3_1 = drive.trajectoryBuilder(WOBBLE3_1.end(),true)
                .splineToLinearHeading(new Pose2d(57.80,6.25,Math.toRadians(0)),Math.toRadians(0)) ///60 1 -6
                .addTemporalMarker(0.5,() -> {
                    prins.setPosition(0.8);
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);
                })
                .build();

        Trajectory SHOOT3_1_1 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition((int)((2786.2/360)*70));
                    brat.setPower(0.5);
                })
                .splineToConstantHeading(new Vector2d(41,-16),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52.80,-15),Math.toRadians(12))  //53 1 0    //57.80 6.25 0
                .build();


        Trajectory INCEPUT_INTAKE = drive.trajectoryBuilder(SHOOT3_1.end())
                .lineToLinearHeading(new Pose2d(52,6,Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    drive.intake.setPower(1);
                })
                .build();


        Trajectory BACK3_2 = drive.trajectoryBuilder(INCEPUT_INTAKE.end(),true)
                .splineToLinearHeading( new Pose2d(37,7,Math.toRadians(0)),Math.toRadians(0))
                .build();


        Trajectory BACK3_3 = drive.trajectoryBuilder(BACK3_2.end())
                .back(14)
                .build();


        Trajectory BACK3_4 = drive.trajectoryBuilder(BACK3_3.end())
                .forward(2)
                .build();
        //ubb frate


        Trajectory BACK3_5 = drive.trajectoryBuilder(BACK3_4.end())
                .back(37)
                .build();

        Trajectory SHOOT3_2 = drive.trajectoryBuilder(BACK3_5.end())
                .splineToLinearHeading(new Pose2d(53,-15,Math.toRadians(-3)),Math.toRadians(0))
                .build();

        Trajectory WOBBLE3_1_1 = drive.trajectoryBuilder(SHOOT3_2.end())
                .addDisplacementMarker(() ->{
                    brat.setTargetPosition((int)((2786.2/360)*170));
                    brat.setPower(0.3);
                })
                .lineToLinearHeading(new Pose2d(110,-19,Math.toRadians(0)))
                .addDisplacementMarker(() ->{
                    prins.setPosition(0);
                })
                .build();




        Trajectory PARK3 = drive.trajectoryBuilder(WOBBLE3_1_1.end())
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);
                    prins.setPosition(0.8);
                })
                .lineToLinearHeading(new Pose2d(67, 0 ,Math.toRadians(175)))
                .build();






///////////////////////////////////////////////////////////////////////////////////////////////////////////////


        drive.m6.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        drive.m6.setDirection(DcMotorEx.Direction.REVERSE);
        drive.m6.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        drive.prins.setPosition(0.83);
        drive.brat.setTargetPosition(0);
        drive.brat.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        drive.brat.setPower(0.3);
        drive.tragaci.setPosition(0.8);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new mti0.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }
        });


        while(!isStarted())
        {

            if(pipeline.position==mti0.SkystoneDeterminationPipeline.RingPosition.FOUR)
                nrAUTO=3;
            else if(pipeline.position==mti0.SkystoneDeterminationPipeline.RingPosition.ONE)
                nrAUTO=2;
            else
                nrAUTO=1;


            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("Pozitie", telefon.getPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }


        waitForStart();


        if(isStopRequested())
            return;


        double pozitie = telefon.getPosition();

        telemetry.addData("Pozitie", pozitie);
        telemetry.update();

        if(nrAUTO==1)
        {
            drive.m6.setVelocity(1700);
            drive.followTrajectory(SHOOT1);
            TrageTare();
            drive.m6.setVelocity(0);
            drive.followTrajectory(WOBBLE1_1);
            drive.followTrajectory(INTERMEDIAR);
            sleep(18000);
            drive.followTrajectory(PARK1);
            drive.turn(Math.toRadians(-7));


        }

        if(nrAUTO==2)
        {
            drive.m6.setVelocity(1700);
            drive.followTrajectory(SHOOT2_1);
            TrageTare();
            drive.intake.setPower(1);
            drive.followTrajectory(PozIntake);
            drive.followTrajectory(discBACK);
            drive.m6.setVelocity(1650);
            drive.followTrajectory(SHOOT2_2);
            TrageTare();
            drive.intake.setPower(0);
            sleep(3000);
            drive.followTrajectory(WOBBLE2_1);
            sleep(6000);

            drive.followTrajectory(PARK2);
            drive.turn(Math.toRadians(13));

        }

        if(nrAUTO==3)
        {   drive.m6.setVelocity(1700);
            drive.followTrajectory(SHOOT3_1_1);
            TrageTare();
            drive.m6.setVelocity(1650);
            drive.followTrajectory(INCEPUT_INTAKE);
            drive.followTrajectory(BACK3_2);
            drive.turn(Math.toRadians(-18));
            TragePower();
            drive.turn(Math.toRadians(15));
            drive.followTrajectory(BACK3_3);
            drive.followTrajectory(SHOOT3_2);
            TrageTare();
            drive.intake.setPower(0);
            sleep(3000);
            drive.followTrajectory(WOBBLE3_1_1);
            sleep(6000);
            drive.followTrajectory(PARK3);
            drive.turn(Math.toRadians(9));
        }





    }   /////////////////////////////////////////////////////////////////////////////////////

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {

        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(340,980);  //////////////////  550  338  350

        static final int REGION_WIDTH = 145;   /////////////////
        static final int REGION_HEIGHT = 180;  /////////////////

        final int FOUR_RING_THRESHOLD = 155;
        final int ONE_RING_THRESHOLD = 140;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile mti0.SkystoneDeterminationPipeline.RingPosition position = mti0.SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = mti0.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = mti0.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = mti0.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = mti0.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }



    public void wobbleSUS(){

        prins.setPosition(0.8);
        sleep(800);
        //  brat.setTargetPosition((int)((2786.2/360)*70));
        brat.setTargetPosition(0);
        brat.setPower(0.3);

    }

    public void wobbleJOS(){
        brat.setTargetPosition((int)((2786.2/360)*165));
        brat.setPower(0.3);
        while(brat.isBusy())
        {

        }
        sleep(1200);
        prins.setPosition(0);
    }

    public void TrageTare(){

        for(int i=1;i<=3;i++) {
            tragaci.setPosition(0.5);
            sleep(150);
            tragaci.setPosition(0.8);
            sleep(150);
        }
        m6.setVelocity(0);
    }

    public void TragePower(){

        tragaci.setPosition(0.5);
        sleep(150);
        tragaci.setPosition(0.8);
        sleep(150);
    }





}
