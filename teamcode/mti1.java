package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
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
public class  mti1 extends LinearOpMode {

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(130, 0, 9, 16.5);
    int nrAUTO;

    OpenCvInternalCamera phoneCam;
    mti1.SkystoneDeterminationPipeline pipeline;
    //www.programareserioasa.ro
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


        /////////////////////////////////////////////   A1   ///////////////////////////////////////////////////

        Trajectory SHOOT1 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition((int)((2786.2/360)*70));
                    brat.setPower(0.5);
                })
                //.splineToConstantHeading(new Vector2d(54,-5),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52.68,3.97),Math.toRadians(-20))   //53 1 0   57.80  6.25
                .build();


        Trajectory WOBBLE1_1 = drive.trajectoryBuilder(SHOOT1.end().plus(new Pose2d(0,0,Math.toRadians(-9))))
                .addDisplacementMarker(() ->{
                    brat.setTargetPosition((int)((2786.2/360)*170));
                    brat.setPower(0.3);
                })
                .lineToLinearHeading(new Pose2d(90,-40.5,Math.toRadians(180)))   //-20
                .addDisplacementMarker(() ->{
                    prins.setPosition(0);
                })
                .build();


        Trajectory WOBBLE1_BACK1 = drive.trajectoryBuilder(WOBBLE1_1.end(),true)
                .lineToLinearHeading(new Pose2d(31,19,Math.toRadians(180)))   ////////////////////////
                .build();
        Trajectory WOBBLE1_BACK2 =  drive.trajectoryBuilder(WOBBLE1_BACK1 .end())
                .lineToLinearHeading(new Pose2d(24,19,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    prins.setPosition(0.83);
                })
                .build();

        Trajectory WOBBLE1_2 = drive.trajectoryBuilder(WOBBLE1_BACK2.end())
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition((int)((2786.2/360)*165));
                    brat.setPower(0.3);
                })
                .lineToLinearHeading(new Pose2d(60,-10,Math.toRadians(-7)))
                .addDisplacementMarker(() -> {
                    prins.setPosition(0);
                })
                .build();




Trajectory BB1 =  drive.trajectoryBuilder(WOBBLE1_1.end(),true)
        .addDisplacementMarker(() -> {
            drive.intake.setPower(1);
                brat.setTargetPosition(0);
                brat.setPower(0.3);
                prins.setPosition(0.8);
                drive.rintake.setPosition(0.85);
                drive.intake2.setPower(-1);

    })
        .lineToLinearHeading(new Pose2d(117,-37,Math.toRadians(180)))
        .build();

Trajectory BB2 = drive.trajectoryBuilder(BB1.end().plus(new Pose2d(0,0,Math.toRadians(90))),true)
        .lineToLinearHeading(new Pose2d(117,30,Math.toRadians(-135)))
        /*.addDisplacementMarker(() -> {
            drive.m6.setVelocity(1650);

        })*/
        .build();

Trajectory BB3 = drive.trajectoryBuilder(BB2.end(),true)
        .lineToLinearHeading(new Pose2d(75,2,Math.toRadians(-37)))
        /*.addDisplacementMarker(() -> {
            drive.m6.setVelocity(1650);
            drive.rintake.setPosition(0);
            drive.intake2.setPower(0);

        })*/
        .build();

Trajectory SHOOT_M6 = drive.trajectoryBuilder(BB3.end(),true)
        .lineToLinearHeading(new Pose2d(52,10,Math.toRadians(-35)))
        .addDisplacementMarker(() -> {

            drive.intake.setPower(0);
            drive.rintake.setPosition(0);
            drive.intake2.setPower(0);
        })
        .build();

Trajectory M6MAFIA = drive.trajectoryBuilder(BB2.end(),true)
        .lineToLinearHeading(new Pose2d(52,10,Math.toRadians(-45)))
        .addDisplacementMarker(() -> {

            drive.intake.setPower(0);
            drive.rintake.setPosition(0);
            drive.intake2.setPower(0);
        })
        .build();


        Trajectory PARK1 = drive.trajectoryBuilder(SHOOT_M6.end(),true)
                .splineToLinearHeading(new Pose2d(66, 5,Math.toRadians(180)),Math.toRadians(180))
                .build();


        ///////////////////////////////////////   A2   ///////////////////////////////////////////////////////////


        Trajectory SHOOT2_1 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    // brat.setTargetPosition((int)((2786.2/360)*70));
                    //brat.setPower(0.5);
                })
                .splineToConstantHeading(new Vector2d(54,-2),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(57.25,-19.25),Math.toRadians(0))
                .build();


        Trajectory discBACK = drive.trajectoryBuilder(SHOOT2_1.end())
                .back(25)
                .build();

        Trajectory SHOOT2_2 = drive.trajectoryBuilder(discBACK.end())
                .lineToLinearHeading(new Pose2d(55,-21,Math.toRadians(-9)))
                .build();


        Trajectory WOBBLE2_1 = drive.trajectoryBuilder(SHOOT2_2.end())
                .addDisplacementMarker(() ->{
                    brat.setTargetPosition((int)((2786.2/360)*170));
                    brat.setPower(0.3);
                })
                .lineToLinearHeading(new Pose2d(110,-10,Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    prins.setPosition(0);
                })
                .build();


        Trajectory BB2_1 =  drive.trajectoryBuilder(WOBBLE2_1.end(),true)
                .addDisplacementMarker(() -> {
                    drive.intake.setPower(1);
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);
                    prins.setPosition(0.8);
                    drive.rintake.setPosition(0.85);
                    drive.intake2.setPower(-1);

                })
                .lineToLinearHeading(new Pose2d(118,-37,Math.toRadians(180)))
                .build();

        Trajectory WOBBLE2_BACK1 = drive.trajectoryBuilder(WOBBLE2_1.end(),true)
                .lineToLinearHeading(new Pose2d(24,15,Math.toRadians(180)))   //31
                .build();
        Trajectory WOBBLE2_BACK2 =  drive.trajectoryBuilder(WOBBLE2_BACK1 .end())
                .lineToLinearHeading(new Pose2d(25,15,Math.toRadians(180)))
                .build();


        Trajectory WOBBLE2_2 = drive.trajectoryBuilder(WOBBLE2_BACK1.end())    //WOBBLE2_BACK2.end()
                .addDisplacementMarker(() -> {
                    prins.setPosition(0.83);
                    brat.setTargetPosition((int)((2786.2/360)*165));
                    brat.setPower(0.3);
                })
                .lineToLinearHeading(new Pose2d(80,5,Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    prins.setPosition(0);
                })
                .build();


        Trajectory PARK2 = drive.trajectoryBuilder(WOBBLE2_1.end(),true)
                .addDisplacementMarker(() -> {
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);
                    prins.setPosition(0.8);
                })
                .splineToLinearHeading(new Pose2d(66, 0.69,Math.toRadians(180)),Math.toRadians(180))
                .build();

        //////////////////////////////////////////////////   A3   /////////////////////////////////////////////////////



        Trajectory BB3_1 =  drive.trajectoryBuilder(SHOOT1.end().plus(new Pose2d(0,0,Math.toRadians(180))),true)
                .addDisplacementMarker(() -> {
                    drive.intake.setPower(1);
                    drive.intake2.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(116,4,Math.toRadians(180)))
                .addDisplacementMarker(() ->{

                })
                .build();

        Trajectory BB3_2 = drive.trajectoryBuilder(BB3_1.end().plus(new Pose2d(0,0,Math.toRadians(-90))),true)
                .lineToLinearHeading(new Pose2d(116,-32,Math.toRadians(90)))
                .build();

        Trajectory BB3_3 = drive.trajectoryBuilder(BB3_2.end().plus(new Pose2d(0,0,Math.toRadians(180))),true)
                .addDisplacementMarker(() -> {
                    prins.setPosition(0);
                })
                .splineToLinearHeading(new Pose2d(120,-29,Math.toRadians(-90)),Math.toRadians(180))
                .addDisplacementMarker(()->{
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);
                    prins.setPosition(0.8);
                })
                .build();
        Trajectory BB3_4 = drive.trajectoryBuilder(BB3_3.end().plus(new Pose2d(0,0,Math.toRadians(-55))))
                .lineToConstantHeading(new Vector2d(124,32))
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
        pipeline = new SkystoneDeterminationPipeline();
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

            if(pipeline.position==mti1.SkystoneDeterminationPipeline.RingPosition.FOUR)
                nrAUTO=3;
            else if(pipeline.position==mti1.SkystoneDeterminationPipeline.RingPosition.ONE)
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
            drive.m6.setVelocity(1470);
            drive.followTrajectory(SHOOT1);
            drive.turn(Math.toRadians(-20));
            TragePower();
            drive.turn(Math.toRadians(7));
            TragePower();
            drive.turn(Math.toRadians(7));
            TragePower();
            drive.m6.setVelocity(0);
            drive.followTrajectory(WOBBLE1_1);
            drive.followTrajectory(BB1);
            drive.turn(Math.toRadians(-135));
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(BB2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(BB3);

            drive.m6.setVelocity(1700);
            drive.followTrajectory(SHOOT_M6);
            TrageTare();
            drive.intake.setPower(0);
            drive.followTrajectory(PARK1);
            drive.turn(Math.toRadians(-7));


        }

        if(nrAUTO==2)
        {
            drive.m6.setVelocity(1470);
            drive.followTrajectory(SHOOT1);
            drive.turn(Math.toRadians(-20));
            TragePower();
            drive.turn(Math.toRadians(7));
            TragePower();
            drive.turn(Math.toRadians(7));
            TragePower();
            drive.m6.setVelocity(0);
            drive.followTrajectory(WOBBLE2_1);
            drive.followTrajectory(BB2_1);
            drive.turn(Math.toRadians(-135));
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(BB2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(BB3) ;
            drive.m6.setVelocity(1700);
            drive.followTrajectory(SHOOT_M6);
            TrageTare();
            drive.intake.setPower(0);
            drive.followTrajectory(PARK1);
            drive.turn(Math.toRadians(-7));
        }

        if(nrAUTO==3)
        {
            drive.m6.setVelocity(1470);
            drive.followTrajectory(SHOOT1);
            drive.turn(Math.toRadians(-20));
            TragePower();
            drive.turn(Math.toRadians(7));
            TragePower();
            drive.turn(Math.toRadians(7));
            TragePower();
            drive.m6.setVelocity(0);
            drive.rintake.setPosition(0.85);
            brat.setTargetPosition((int)((2786.2/360)*170));
            brat.setPower(0.3);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(BB3_1);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(BB3_2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(BB3_3);
            drive.turn(Math.toRadians(-55));
            drive.followTrajectory(BB3_4);
            drive.m6.setVelocity(1700);
            drive.followTrajectory(M6MAFIA);
            TrageTare();
            drive.intake.setPower(0);
            drive.followTrajectory(PARK1);
            drive.turn(Math.toRadians(-7));




            /*
            drive.followTrajectory(BB3_1);
            drive.followTrajectory(BB3_2);
            drive.followTrajectory(BB3_3);
            drive.turn(Math.toRadians(175));
            drive.followTrajectory(BB3) ;
            drive.m6.setVelocity(1700);
            drive.followTrajectory(SHOOT_M6);
            TrageTare();
            drive.intake.setPower(0);
            drive.followTrajectory(PARK1);
            drive.turn(Math.toRadians(-7));
            */



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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(770,990);  //////////////////  550  338

        static final int REGION_WIDTH = 130;   /////////////////
        static final int REGION_HEIGHT = 150;  /////////////////

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
        private volatile mti1.SkystoneDeterminationPipeline.RingPosition position = mti1.SkystoneDeterminationPipeline.RingPosition.FOUR;

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

            position = mti1.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = mti1.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = mti1.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = mti1.SkystoneDeterminationPipeline.RingPosition.NONE;
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
