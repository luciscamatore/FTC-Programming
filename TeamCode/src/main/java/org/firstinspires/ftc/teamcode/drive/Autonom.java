package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Config
@Disabled
@Autonomous
public class Autonom extends LinearOpMode {
    DcMotor intake;
    DcMotor intake2;
    DcMotor carusel;
    Servo baza, arm, macara, cleste;

    DistanceSensor dist;

    private static final String TFOD_MODEL_ASSET = "model_20220216_195212.tflite";
    private static final String[] LABELS = {
            "TSE"
    };
    private static final String VUFORIA_KEY =
            "AXea9Af/////AAABmXkJssyFt0iWhZPf0JjyRbIl6AI1zZwBsw3gB+ULAv6vvt7AmINgZcObH0Os3hwhF6vuHUjPnqhKmgvwuQnKhxfUQ4GRmNAqeDxbZBaDKw/P10qiZkrT8R34eAMVZkgugm55RZCN1R/gLwtAQnb+/B6iYkmeItUnvIf74mB6fMwwCM7eX/H1L8ke3w2prQ2ZiLCoRARM+ZRKdPBE1x/u4EyzJ+cFJ/eg1QOFFZOnkaOewZbesLU++qQvHO4/tO2VzQrbh86tVGgCsJqzlyjMbIL9BXNSUcxXcpa6CFixaGOS5g65eScfchxKaUOUhL9dj3HJO3/l2UPQJujFC3BuOaq8qNJuCtSJ/5sv/35VjOKH";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int p;
    int nivelTurn;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-33.07, -64.17, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        intake = hardwareMap.dcMotor.get("intake");
        intake2 = hardwareMap.dcMotor.get("intake2");
        carusel = hardwareMap.dcMotor.get("carusel");
        carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        baza = hardwareMap.servo.get("baza");
        arm = hardwareMap.servo.get("arm");
        arm.setDirection(Servo.Direction.REVERSE);
        macara = hardwareMap.servo.get("macara");
        cleste = hardwareMap.servo.get("cleste");
        cleste.setDirection(Servo.Direction.REVERSE);

        dist = hardwareMap.get(DistanceSensor.class, "dist");

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.3, 16.0/9.0);
        }


        TrajectorySequence caruselTurnBariera = drive.trajectorySequenceBuilder(startPose)
                //merge la carusel
                .lineToConstantHeading(new Vector2d(-60,-55.31)) //,Math.toRadians(180)

                //ne lipim de carusel
                .strafeLeft(2.7,
                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //pornim motor
                .addTemporalMarker(() -> {
                    //rotire motor
                    duckSlow();
                })
                .waitSeconds(2)

                //oprim motor
                .addTemporalMarker(() -> {
                    carusel.setPower(0);
                })
                .waitSeconds(0.1)

                //mergem spre hub
                .lineTo(new Vector2d(-9.33, -48.95))
                //intre timp rotim brat
                /*.addTemporalMarker(3, ()->{
                    macara.setPosition(0.65);
                })*/

                //ne rotim spre hub
                .turn(Math.toRadians(170))
                .addTemporalMarker(() -> {
                    rotateRight();
                })
                .waitSeconds(0.7)
                //ridicam la nivel
                .addTemporalMarker(() -> {
                    //selectam nivelu
                    if(nivelTurn == 1 ){
                        nivelJos();
                    }else if(nivelTurn == 2)
                    {
                        nivelMijloc();
                    }else if(nivelTurn == 3)
                    {
                        nivelSus();
                    }
                    else
                    {
                        nivelSus();
                    }
                })
                .waitSeconds(0.7)

                //mergem inspre hub si dam dropu
                //.strafeLeft(8)
                .strafeTo(new Vector2d(-10.03, -42))
                .addTemporalMarker(() -> {
                    dropCube();
                })
                .waitSeconds(0.3)

                //mergem spre perete si strangem bratu si rotim
                //.strafeRight(5.90)
                .lineToLinearHeading(new Pose2d(-10.03, -53.9, Math.toRadians(-10)))
                .addTemporalMarker(() -> {
                    strangeBrat();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(18, () -> {
                    rotateCenter();
                })

                //mergem spre warehouse
                .strafeTo(new Vector2d(-10, -67.5))

                .lineToLinearHeading(new Pose2d(30, -67.5))
                .strafeRight(5.5)
                .build();

        TrajectorySequence ciclare = drive.trajectorySequenceBuilder(caruselTurnBariera.end())
                //no amu ii atunci
                //ciclam fram ca ce sa facem

                //luam cub
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    sugeCub();
                })
                .addDisplacementMarker(() -> {
                    fataCub();
                })
                .lineToConstantHeading(new Vector2d(-10, -65))
                .addDisplacementMarker(() -> {
                    loFatat();
                })
                .waitSeconds(0.1)


                .addTemporalMarker(() -> {
                    rotateLeft();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    nivelSus();
                })
                .waitSeconds(1)
                .strafeLeft(8)
                .addTemporalMarker(() -> {
                    dropCube();
                })
                .waitSeconds(0.3)
                .strafeRight(5.90)
                .addTemporalMarker(() -> {
                    strangeBrat();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    rotateCenter();
                })
                .waitSeconds(0.7)
                .build();


        cleste.setPosition(0.3);

        waitForStart();

        nivelTurn = pozitie();

        drive.followTrajectorySequence(caruselTurnBariera);

        //drive.followTrajectorySequence(ciclare);
        //drive.followTrajectorySequence(ciclare);



    }


    /****************************************************************************************
      ______   _    _   _   _    _____   _______   _____   _____
     |  ____| | |  | | | \ | |  / ____| |__   __| |_   _| |_   _|
     | |__    | |  | | |  \| | | |         | |      | |     | |
     |  __|   | |  | | | . ` | | |         | |      | |     | |
     | |      | |__| | | |\  | | |____     | |     _| |_   _| |_
     |_|       \____/  |_| \_|  \_____|    |_|    |_____| |_____|
    ****************************************************************************************/
    public void sugeCub(){
        while(dist.getDistance(DistanceUnit.MM) > 35)
        {
            intake.setPower(1);
            intake2.setPower(-0.65);
        }
        cleste.setPosition(0.3);
        intake.setPower(0);
        intake2.setPower(0);
    }
    public void fataCub(){
        intake.setPower(-1);
        intake2.setPower(0.65);
    }
    public void loFatat(){
        intake.setPower(0);
        intake2.setPower(0);
    }
    public void duckSlow(){
        carusel.setPower(0.4);
    }
    public void duckFast(){
        carusel.setPower(1);
    }
    public void rotateLeft(){
        macara.setPosition(1);
    }
    public void rotateRight(){
        macara.setPosition(0);
    }
    public void rotateCenter(){macara.setPosition(0.5);}

    public void ridicaBrat(){
        baza.setPosition(0.2);
        arm.setPosition(0.45);
        cleste.setPosition(0.3);
    }
    public void strangeBrat() {
        baza.setPosition(0.3);
        arm.setPosition(0);
        cleste.setPosition(0);
    }
    public void dropCube()
    {
        cleste.setPosition(0);
    }
    public void nivelJos()
    {
        baza.setPosition(0.2);
        arm.setPosition(0.15);
    }
    public void nivelMijloc(){
        baza.setPosition(0.2);
        arm.setPosition(0.3);
    }
    public void nivelSus(){
        baza.setPosition(0.2);
        arm.setPosition(0.45);
    }
    public int pozitie()
    {
        if(tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                for (Recognition recognition : updatedRecognitions) {
                    //CONDITII
                    if(recognition.getLeft() < 200) {
                        p=1;
                    }
                    else if(recognition.getLeft() > 200 && recognition.getLeft() < 400)
                    {
                        p=2;
                    }else if(recognition.getLeft() > 400)
                        p= 3;

                }
                telemetry.update();
            }
        }
        return p;
    }
    public void pozitieVoid()
    {
        if(tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                int i=0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    //CONDITII
                    if(recognition.getLeft() < 200) {
                        telemetry.addLine("1");
                    }
                    else if(recognition.getLeft() > 200 && recognition.getLeft() < 400)
                    {
                        telemetry.addLine("2");
                    }else if(recognition.getLeft() > 400)
                        telemetry.addLine("3");

                }
                telemetry.update();
            }
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}