package org.firstinspires.ftc.teamcode.drive.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class testservo extends LinearOpMode {
    public Servo dreapta;
    public Servo stanga;
    public CRServo elice1;
    public CRServo elice2;



    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        dreapta = hardwareMap.get(Servo.class, "dreapta");
        stanga = hardwareMap.get(Servo.class, "stanga");
        elice1 = hardwareMap.get(CRServo.class, "elice1");
        elice2 = hardwareMap.get(CRServo.class, "elice2");
        while (opModeIsActive()) {
            if (gamepad1.x) {
                dreapta.setPosition(0.45);
            }
            if(gamepad1.y)
            {
                dreapta.setPosition(0);
            }
            if(gamepad1.a)
            {
                elice1.setPower(1);
            }
        }
    }

}
