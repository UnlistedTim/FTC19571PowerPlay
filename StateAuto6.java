package org.firstinspires.ftc.teamcode.drive;

//mport  org.firstinspires.ftc.teamcode.drive.Trial.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

/**
 * This is a test op - drop the cone at the level 3 pole.
 * Movement:
 *  1. backward to the pole, turn counter clock 135;
 *  2. move a little forward, drop cone, backward the same distance
 *  3. turn counter clockwise 135, move forward to pick cone
 *  4. back to the first drop-cone position, turn right 136 for drop
 *  repeat 2-4 for more pick and drop
 *  5. if parking at zone 3, last drop action in step 4's to backward one block further to make it in parking zone 4 for drop at the same pole
 */
@Autonomous(name = "StateAuto6", group = "A")
@Disabled
@Config
public class StateAuto6 extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "ABG_Model1.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "Apple",
            "Banana",
            "Grape"
    };
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AWg52rL/////AAABmdjtwukCMEIWkkw/HJ019KRqk7zvrUf22qbakKlQZyyRyVlV7TowKMdq3JsOb5hi/6txnmzwdetP6kmm8uEZsQ+aN+rldPdNnwv/Aqo21HOx0fmRykuUthzWk2QHBoS0nwRkRRCr8fdGT8h+0He56QHhsfQNsYWi0By5ECXrss+bKu8t36cgcoOqTlfBz/msn9iCWzznek+vPjkNj6hWxjg6/uqTv10wn+NlNw5V4H/G+T0M9Bgux5gkuA1VYWU9KXUN3uyVrzMXVOJzpMt1OmpJsqWZIBs7GJ64nrIlyQZbWz7SA3i4o3+uC1cAMRTGHSJmswBc/y+oh2XrwHK8E5yl3kbbazFKRCWOk8uYGHP+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private DcMotorEx linear_slide;
    private DcMotorEx front_right;
    private Servo cam;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private DigitalChannel take_touch;
    private DigitalChannel guide_touch;
    private BNO055IMU imu;
    private DistanceSensor cam_distance;
    private DistanceSensor right_distance;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor color;


    BNO055IMU.Parameters imuParameters; //added
    Orientation angles; //added
    Acceleration gravity; //added
   // trial trial;
    SampleMecanumDrive drive;

    TrajectorySequence trajSeq_A_B_C_drop = null;
    TrajectorySequence trajSeq_A_B_C_dropc1 = null;
    TrajectorySequence trajSeq_A_B_C_dropr = null;
    TrajectorySequence trajSeq_A_B_C_dropa = null;
    TrajectorySequence trajSeq_p2l_p1l_pick  = null;
    TrajectorySequence trajSeq_p2lc_p1l_pick  = null;
    TrajectorySequence trajSeq_p2r_p1r_pick  = null;


    TrajectorySequence trajSeq_p3l_p1l_pick = null;
    TrajectorySequence trajSeq_p3r_p1r_pick = null;
    TrajectorySequence trajSeq_p3rb_p1r_pick = null;
    TrajectorySequence  trajSeq_p1l_p4l_drop = null;
    TrajectorySequence  trajSeq_p1l_p2lc_drop = null;
    TrajectorySequence  trajSeq_p1r_p4r_drop = null;
    TrajectorySequence  trajSeq_p1r_p3rb_drop = null;
    TrajectorySequence trajSeq_p4l_p1l_pick= null;
    TrajectorySequence trajSeq_p4r_p1r_pick= null;
    TrajectorySequence trajSeq_p3lb_p1l_pick = null;
    TrajectorySequence trajSeq_p1l_p3l_drop   = null;
    TrajectorySequence trajSeq_p1l_p3lb_drop = null;
    TrajectorySequence trajSeq_p1l_p3la_dropa  = null;
    TrajectorySequence trajSeq_p1r_p3r_drop   = null;
    TrajectorySequence trajSeq_F_E_Parking3  = null;
    TrajectorySequence trajSeq_C_B_Parking1  = null;
    TrajectorySequence trajSeq_C_B_Parking2  = null;
    TrajectorySequence trajSeq_F_E_Parking3b  = null;
    TrajectorySequence trajSeq_C_B_Parking1b  = null;
    TrajectorySequence trajSeq_C_B_Parking2b  = null;
    TrajectorySequence trajSeq_F_E_Parking3r  = null;
    TrajectorySequence trajSeq_C_B_Parking1r  = null;
    TrajectorySequence trajSeq_C_B_Parking2r  = null;
    TrajectorySequence trajSeq_F_E_Parking3rb  = null;
    TrajectorySequence trajSeq_C_B_Parking1rb  = null;
    TrajectorySequence trajSeq_C_B_Parking2rb  = null;


    int  opinion=0;
    boolean left_side=true;
    boolean blue_color=true;
    int defaultParkingzone = 3; // default

    int dropLevel1 = -1320; // the height for pole level 1
    int dropLevel2 = -2150; // the height for pole level 2
    int dropLevel3 = -2970;//-2980; // the height for pole level 3
    int droplevel=0;
    int prset_gap=720;
    double tick_length=0.7837653;
    double spin_distance=0;
    int pk_chk_height=-120;
    int pk_l5=-850;
    // the height for pick up the cone
    int conePosition5stack = -626;//-1143;
    int conePosition4stack = -500;//-985;
    int conePosition3stack = -380;//-826;
    int conePosition2stack = -200;//-668;
    double gyro = 0;
    boolean  guide_touch_flag= false;
    boolean take_touch_flag=false;
    boolean firstdrop=true;
    boolean H1=true;
   Pose2d clocation;

    double start_time = 0;


    boolean error =false;



    // TODO adjust the move distance for the drop
    double forwardDropDistance = 5.5; // 9-10
    double dropShift = forwardDropDistance * Math.sqrt(2)/2;

    Pose2d startPoseA = new Pose2d();

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPoseA);




        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        linear_slide = hardwareMap.get(DcMotorEx.class, "linear_slide");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");

        cam = hardwareMap.get(Servo.class, "cam");
        take_touch = hardwareMap.get(DigitalChannel.class, "take_touch");
        guide_touch = hardwareMap.get(DigitalChannel.class, "guide_touch");
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //added
        cam_distance = hardwareMap.get(DistanceSensor.class, "cam_distance");
        right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");
        color = hardwareMap.get(ColorSensor.class,"color");
//        guide_touch = hardwareMap.get(DigitalChannel.class, "guide_touch");
//        take_touch.setMode(DigitalChannel.Mode.INPUT);
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cam.setPosition(0);
        buildTrajectories();
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
       // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
       // gyro = angles.firstAngle;
        telemetry.addLine("Driver      X-Left         B-Right ");
        telemetry.addLine("Driver      A-1H           Y-2H ");
        telemetry.addLine("Gunner     X-Blue         B-Red");



        //telemetry.addData("Attach the cone now");
        telemetry.update();
       // color.enableLed(false);
        left_side=false; H1=false; blue_color=false;// only for debug, remove later;
        // gamestart
        while(true){
           if(gamepad1.b) {
               left_side=false;
               telemetry.addLine("RIGHT SIDE SELECTED");
               telemetry.update();
           }
            if(gamepad1.x) {
                left_side=true;
                telemetry.addLine("LEFT SIDE SELECTED");
                telemetry.update();
            }
            if(gamepad1.a) {
                H1=true;
                telemetry.addLine("1H Mode SELECTED");
                telemetry.update();
            }
            if(gamepad1.y) {
                H1=false;
                telemetry.addLine("2H Mode SELECTED");
                telemetry.update();
            }

            if(gamepad2.x) {
                blue_color=true;
                telemetry.addLine("BLUE COLOR SELECTED");
                telemetry.update();
            }
            if(gamepad2.b) {
                blue_color=false;
                telemetry.addLine("RED COLOR SELECTED");
                telemetry.update();
            }
            if (!take_touch.getState()){
                cam.setPosition(0.33);
                if(left_side)   telemetry.addLine("LEFT SIDE SELECTED");
                else   telemetry.addLine("RIGHT SIDE SELECTED");
                if(blue_color)    telemetry.addLine("BLUE COLOR SELECTED");
                else  telemetry.addLine("RED COLOR SELECTED");
                if(H1)    telemetry.addLine("1H Route");
                else  telemetry.addLine("2H Route");
                break;
            }
        }
        // initialize camera to get ready
        initVuforia();
        tfod = initTfod();
        if (tfod == null) {
            telemetry.addData(">", "initTfod failed");
            telemetry.update();
        } else {
            // Activate TensorFlow Object Detection before we wait for the start command.
            tfod.activate();
            // zoom can be adjustaed, it seems 1.0 is the best
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        // when competition, comment out FtcDashboard related
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        /** Wait for the game to begin */
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start op mode ");
        telemetry.update();
        waitForStart();
        runtime.reset();
        droplevel=dropLevel1;
        // slide up asynchronously
       startSlideUp(true);
        if (isStopRequested())
           return;
     //   droplevel=dropLevel3;
      //  linearslidets(droplevel+prset_gap,2000);

        while (opModeIsActive() && !isStopRequested()) {
            int parkingZone = getParkingZoneFromPic();

            autoDriveDropAndPark(parkingZone);
           sleep(500);
            break;
        }
    }
    private void startSlideUp(boolean asyn){
        if(asyn) {
            Thread thread = new Thread() {
                public void run() {
                    startSlideUp();
                }
            };
            thread.start();
        }else
            startSlideUp();
    }

    private void startSlideUp(){
        // slide up initially
        linear_slide.setTargetPosition(droplevel+prset_gap);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear_slide.setTargetPosition(droplevel+prset_gap);
        linear_slide.setVelocity(2600);
         //sleep(400);
    }

    private void slideUpAsyncTo(int targetPosition){
        Thread thread = new Thread(){
            public void run(){
                linear_slide.setTargetPosition(targetPosition);
                linear_slide.setVelocity(2200);
            }
        };
        thread.start();
    }

    /**
     * Define the point coordinates, and build each movement between pick and drop
     */
    private void buildTrajectories(){
        // B C position angle



        Pose2d  p1l = new Pose2d(-50, -23.5, Math.toRadians(-88));
        Pose2d  p2l = new Pose2d(-48, -6, Math.toRadians(-60));
        Pose2d  p3l = new Pose2d(-51, 21, Math.toRadians(-65));
        Pose2d  p4l = new Pose2d(-52.5, 45, Math.toRadians(-70));

        Pose2d  p1r = new Pose2d(-52, 22, Math.toRadians(90));
        Pose2d  p2r = new Pose2d(-50, 4, Math.toRadians(60));
        Pose2d  p3r = new Pose2d(-49, -21, Math.toRadians(70));
        Pose2d  p4r = new Pose2d(-46.5, -46, Math.toRadians(76));

        Pose2d  p1la = new Pose2d(-50, -23.5, Math.toRadians(-88));
        Pose2d  p2la = new Pose2d(-48, -6, Math.toRadians(-42));
        Pose2d  p3la = new Pose2d(-51, 22, Math.toRadians(-40));
        Pose2d  p4la = new Pose2d(-52.5, 42, Math.toRadians(-45));

        Pose2d  p3lb = new Pose2d(-50, 21, Math.toRadians(-115));
        Pose2d  p3rb = new Pose2d(-50, -21, Math.toRadians(115));
        Pose2d  p2lc = new Pose2d(-51, 4, Math.toRadians(145));
        Pose2d  p2lc1 = new Pose2d(-52, -1, Math.toRadians(145));




        Pose2d p1Pose = new Pose2d(-48, -24, Math.toRadians(-90));
        Pose2d p2Pose = new Pose2d(-48, -2, Math.toRadians(-90));
        Pose2d p3Pose = new Pose2d(-49, 23, Math.toRadians(-90));

        Pose2d p1Poser = new Pose2d(-49, -24, Math.toRadians(93));
        Pose2d p2Poser = new Pose2d(-50, -0.5, Math.toRadians(93));
        Pose2d p3Poser = new Pose2d(-51, 22.5, Math.toRadians(93));


        // first path to drop

       // drive.setPoseEstimate(new Pose2d(x, y, heading));




        trajSeq_A_B_C_drop = drive.trajectorySequenceBuilder(startPoseA)
                //  .lineTo(new Vector2d(PushconePoseG.getX(), dropPoseC.getY()),
                //         SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(53.5)//15
                .lineToSplineHeading(p2l)
              //  .forward(forwardDropDistance)
                //               .lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //                       SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_A_B_C_dropc1 = drive.trajectorySequenceBuilder(startPoseA)
                //  .lineTo(new Vector2d(PushconePoseG.getX(), dropPoseC.getY()),
                //         SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(15)//15
                .lineToSplineHeading(p2lc1)
                //  .forward(forwardDropDistance)
                //               .lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //                       SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_A_B_C_dropa = drive.trajectorySequenceBuilder(startPoseA)
                //  .lineTo(new Vector2d(PushconePoseG.getX(), dropPoseC.getY()),
                //         SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(53)//15
                .lineToSplineHeading(p2la)
                //  .forward(forwardDropDistance)
                //               .lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //                       SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_A_B_C_dropr = drive.trajectorySequenceBuilder(startPoseA)
                //  .lineTo(new Vector2d(PushconePoseG.getX(), dropPoseC.getY()),
                //         SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(53)//15
                .lineToSplineHeading(p2r)
                //  .forward(forwardDropDistance)
                //               .lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //                       SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // the paths to go to pick the cone
        trajSeq_p2l_p1l_pick = drive.trajectorySequenceBuilder(p2l)
                //    .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //      SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1l)
                //.lineToLinearHeading(p1Pose)

                //  .lineTo(new Vector2d(pickConePoseD.getX(), pickConePoseD.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p2lc_p1l_pick = drive.trajectorySequenceBuilder(p2lc)
                //    .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //      SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1l)
                //.lineToLinearHeading(p1Pose)

                //  .lineTo(new Vector2d(pickConePoseD.getX(), pickConePoseD.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p2r_p1r_pick = drive.trajectorySequenceBuilder(p2r)
                //    .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //      SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1r)
                //.lineToLinearHeading(p1Pose)

                //  .lineTo(new Vector2d(pickConePoseD.getX(), pickConePoseD.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*1.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p3l_p1l_pick = drive.trajectorySequenceBuilder(p3l)
                //    .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //      SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1l)
                //.lineToLinearHeading(p1Pose)

                //  .lineTo(new Vector2d(pickConePoseD.getX(), pickConePoseD.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajSeq_p3r_p1r_pick = drive.trajectorySequenceBuilder(p3r)
                //    .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //      SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1r)
                //.lineToLinearHeading(p1Pose)

                //  .lineTo(new Vector2d(pickConePoseD.getX(), pickConePoseD.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajSeq_p3lb_p1l_pick = drive.trajectorySequenceBuilder(p3lb)
                //    .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //      SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1l)
                //.lineToLinearHeading(p1Pose)

                //  .lineTo(new Vector2d(pickConePoseD.getX(), pickConePoseD.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // after pick cone, the path to drop position for parking 1 & 2
        trajSeq_p1l_p3l_drop = drive.trajectorySequenceBuilder(p1l)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              //  .lineToLinearHeading(p2Pose)
             //   .lineToSplineHeading(p2l)
             //   .back(24)
                .lineToSplineHeading(p3l)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p1l_p2lc_drop = drive.trajectorySequenceBuilder(p1l)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(p2Pose)
                //   .lineToSplineHeading(p2l)
                //   .back(24)
                .lineToSplineHeading(p2lc)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajSeq_p1l_p3lb_drop = drive.trajectorySequenceBuilder(p1l)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(p2Pose)
                //   .lineToSplineHeading(p2l)
                //   .back(24)
                .lineToSplineHeading(p3lb)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p1l_p3la_dropa = drive.trajectorySequenceBuilder(p1l)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(p2Pose)
                //   .lineToSplineHeading(p2l)
                //   .back(24)
                .lineToSplineHeading(p3la)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p1r_p3r_drop = drive.trajectorySequenceBuilder(p1r)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(p2Pose)
                //   .lineToSplineHeading(p2l)
                //   .back(24)
                .lineToSplineHeading(p3r)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p1l_p4l_drop = drive.trajectorySequenceBuilder(p1l)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(p2Pose)
                //   .lineToSplineHeading(p2l)
              //  .lineToSplineHeading(p3al)
            //    .back(24)
                .lineToSplineHeading(p4l)
               // .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
              //  .lineToSplineHeading(p4l)
              //  .forward(3)
            //    .forward(7)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // last time after pick cone, the path to drop position for parking 3
        // slow get out of cone stack, fast to E, slow to F
        trajSeq_p1r_p4r_drop = drive.trajectorySequenceBuilder(p1r)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(p2Pose)
                //   .lineToSplineHeading(p2l)
                //  .lineToSplineHeading(p3al)
                //    .back(24)
                .lineToSplineHeading(p4r)
                // .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
                //  .lineToSplineHeading(p4l)
                //  .forward(3)
                //    .forward(7)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajSeq_p1r_p3rb_drop = drive.trajectorySequenceBuilder(p1r)
                // .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .lineToLinearHeading(p2Pose)
                //   .lineToSplineHeading(p2l)
                //  .lineToSplineHeading(p3al)
                //    .back(24)
                .lineToSplineHeading(p3rb)
                // .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
                //  .lineToSplineHeading(p4l)
                //  .forward(3)
                //    .forward(7)
                // .forward(forwardDropDistance)
                //.lineTo(new Vector2d(dropPoseC.getX(), dropPoseC.getY()),
                //     SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
       trajSeq_p4l_p1l_pick = drive.trajectorySequenceBuilder(p4l)
        //        .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
        //                SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
         //               SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .lineToSplineHeading(p1l)
            //    .lineTo(new Vector2d(dropPoseF.getX(), dropPoseF.getY()),
            //            SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
           //             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
               .build();

        // trajectory from where it is to the designated parking zone
        trajSeq_p4r_p1r_pick = drive.trajectorySequenceBuilder(p4r)
                //        .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //                SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //               SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1r)
                //    .lineTo(new Vector2d(dropPoseF.getX(), dropPoseF.getY()),
                //            SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        trajSeq_p3rb_p1r_pick = drive.trajectorySequenceBuilder(p3rb)
                //        .lineTo(new Vector2d(p1Pose.getX(), p1Pose.getY()),
                //                SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //               SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p1r)
                //    .lineTo(new Vector2d(dropPoseF.getX(), dropPoseF.getY()),
                //            SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //             SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        // trajectory from where it is to the designated parking zone

        trajSeq_F_E_Parking3 = drive.trajectorySequenceBuilder(p3l)
//                .lineTo(new Vector2d(p3Pose.getX(), p3Pose.getY()),
//                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), // old 0.7
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p3Pose)
                .build();

        trajSeq_F_E_Parking3b = drive.trajectorySequenceBuilder(p3lb)
//                .lineTo(new Vector2d(p3Pose.getX(), p3Pose.getY()),
//                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), // old 0.7
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p3Pose)
                .build();
        trajSeq_F_E_Parking3r = drive.trajectorySequenceBuilder(p3r)
//                .lineTo(new Vector2d(p3Pose.getX(), p3Pose.getY()),
//                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), // old 0.7
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p3Poser)
                .build();
        trajSeq_F_E_Parking3rb = drive.trajectorySequenceBuilder(p3rb)
//                .lineTo(new Vector2d(p3Pose.getX(), p3Pose.getY()),
//                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), // old 0.7
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(p3Poser)
                .build();
        trajSeq_C_B_Parking1 = drive.trajectorySequenceBuilder(p3l)
//
                //  .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .back(9)
                .lineToLinearHeading(p1Pose)
                .build();

        trajSeq_C_B_Parking1b = drive.trajectorySequenceBuilder(p3lb)
//
                //  .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .back(9)
                .lineToLinearHeading(p1Pose)
                .build();
        trajSeq_C_B_Parking1r = drive.trajectorySequenceBuilder(p3r)
//
                //  .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .back(9)
                .lineToLinearHeading(p1Poser)
                .build();
        trajSeq_C_B_Parking2rb = drive.trajectorySequenceBuilder(p3rb)
                // .back(3)
                .lineToSplineHeading(p2Poser)
                .build();
        trajSeq_C_B_Parking1rb = drive.trajectorySequenceBuilder(p3rb)
//
                //  .lineTo(new Vector2d(dropPrepPoseB.getX(), dropPrepPoseB.getY()),
                //          SampleMecanumDrive.getVelocityConstraint(MAX_VEL*0.9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //          SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //  .back(9)
                .lineToLinearHeading(p1Poser)
                .build();
        trajSeq_C_B_Parking2r = drive.trajectorySequenceBuilder(p3r)
                // .back(3)
                .lineToSplineHeading(p2Poser)
                .build();
        trajSeq_C_B_Parking2b = drive.trajectorySequenceBuilder(p3lb)
                // .back(3)
                .lineToSplineHeading(p2Pose)
                .build();
        trajSeq_C_B_Parking2 = drive.trajectorySequenceBuilder(p3l)
                // .back(3)
                .lineToSplineHeading(p2Pose)
                .build();
    }

    private void autoDriveDropAndPark(int parkingZone) {
        int pickIndex = 1;

        if(left_side&&!H1) {
            drive.followTrajectorySequence(trajSeq_A_B_C_drop);
            spina(-1);
            stop_motor();
            droplevel = dropLevel3;
        if(!error) {
            drive.followTrajectorySequence(trajSeq_p2l_p1l_pick);
            pickCone(pickIndex++, dropLevel3);
            drive.followTrajectorySequence(trajSeq_p1l_p3lb_drop);
            spina(1);
            sleep(150);
            stop_motor();
        }
        if(!error) {
            drive.followTrajectorySequence(trajSeq_p3lb_p1l_pick);
            pickCone(pickIndex++, dropLevel3);
            drive.followTrajectorySequence(trajSeq_p1l_p3lb_drop);
            spina(1);
            sleep(150);
            stop_motor();
        }
            slidePositionTo(0,2000 , 0);
            if (parkingZone == 1) drive.followTrajectorySequence(trajSeq_C_B_Parking1b);
            else if (parkingZone == 2) drive.followTrajectorySequence(trajSeq_C_B_Parking2b);
            else drive.followTrajectorySequence(trajSeq_F_E_Parking3b);
            return;
        }


        if(left_side&&H1) {
            drive.followTrajectorySequence(trajSeq_A_B_C_drop);
            spina(-1);
            stop_motor();
            droplevel = dropLevel3;
//            int pickIndex = 1;
            if (!error) {
                drive.followTrajectorySequence(trajSeq_p2l_p1l_pick);
                pickCone(pickIndex++, dropLevel3);
                drive.followTrajectorySequence(trajSeq_p1l_p4l_drop);
                spina(-1);
                sleep(100);
                stop_motor();
            }

            droplevel = dropLevel2;
            if (!error) {
                drive.followTrajectorySequence(trajSeq_p4l_p1l_pick);
                pickCone(pickIndex, dropLevel3);
                drive.followTrajectorySequence(trajSeq_p1l_p3l_drop);
                spina(-1);
                stop_motor();
            }

            slidePositionTo(0, 2000, 0);
            if (parkingZone == 1) drive.followTrajectorySequence(trajSeq_C_B_Parking1);
            else if (parkingZone == 2) drive.followTrajectorySequence(trajSeq_C_B_Parking2);
            else drive.followTrajectorySequence(trajSeq_F_E_Parking3);
            return;

        }


//        else if (opinion==2) {
//
//            drive.followTrajectorySequence(trajSeq_A_B_C_dropa);
//            spinb(1);
//            stop_motor();
//
//            droplevel = dropLevel2;
//          //  sleep(500000);
////            int pickIndex = 1;
//            if (!error) {
//                drive.followTrajectorySequence(trajSeq_p2l_p1l_pick);
//                pickCone(pickIndex++, dropLevel2);
//                drive.followTrajectorySequence(trajSeq_p1l_p3la_dropa);
//                spinb(1);
//              //  sleep(200);
//                stop_motor();
//            }
//
//            droplevel = dropLevel3;
//            if (!error) {
//                drive.followTrajectorySequence(trajSeq_p3l_p1l_pick);
//                pickCone(pickIndex, dropLevel3);
//                drive.followTrajectorySequence(trajSeq_p1l_p4l_drop);
//                spina(-1);
//                sleep(200);
//                stop_motor();
//                sleep(500000);
//            }
//
//            slidePositionTo(0, 1800, 0);
//            drive.setMotorPowers(0, 0, 0, 0);
//            if (parkingZone == 1) drive.followTrajectorySequence(trajSeq_C_B_Parking1);
//            else if (parkingZone == 2) drive.followTrajectorySequence(trajSeq_C_B_Parking2);
//            else drive.followTrajectorySequence(trajSeq_F_E_Parking3);
//            return;
//
//
//        }
//


        //right side
       if(!left_side &&H1)
        {
            drive.followTrajectorySequence(trajSeq_A_B_C_dropr);
            spina(1);
           stop_motor();
            droplevel = dropLevel3;
//            int pickIndex = 1;
            if (!error) {
                drive.followTrajectorySequence(trajSeq_p2r_p1r_pick);
                pickCone(pickIndex++, dropLevel3);
                drive.followTrajectorySequence(trajSeq_p1r_p4r_drop);
                spina(1);
               sleep(200);
                stop_motor();
            }
            droplevel = dropLevel2;
            if (!error) {
                drive.followTrajectorySequence(trajSeq_p4r_p1r_pick);
                pickCone(pickIndex, dropLevel3);
                drive.followTrajectorySequence(trajSeq_p1r_p3r_drop);
                spina(1);
              sleep(50);
                stop_motor();
            }

            slidePositionTo(0, 1800, 0);
           stop_motor();
            if (parkingZone == 1) drive.followTrajectorySequence(trajSeq_C_B_Parking1r);
            else if (parkingZone == 2) drive.followTrajectorySequence(trajSeq_C_B_Parking2r);
            else drive.followTrajectorySequence(trajSeq_F_E_Parking3r);
            return;
       }

        if(!left_side &&!H1)
        {
            drive.followTrajectorySequence(trajSeq_A_B_C_dropr);
            spina(1);
            //  sleep(100);
            stop_motor();
            droplevel = dropLevel3;
//            int pickIndex = 1;
            if (!error) {
                drive.followTrajectorySequence(trajSeq_p2r_p1r_pick);
                pickCone(pickIndex++, dropLevel3);
                drive.followTrajectorySequence(trajSeq_p1r_p3rb_drop);
                spina(-1);
                sleep(150);
                stop_motor();
            }
            droplevel = dropLevel3;
            if (!error) {
                drive.followTrajectorySequence(trajSeq_p3rb_p1r_pick);
                pickCone(pickIndex, dropLevel3);
                drive.followTrajectorySequence(trajSeq_p1r_p3rb_drop);
                spina(-1);
                sleep(150);
                stop_motor();
            }

            slidePositionTo(0, 1800, 0);
            stop_motor();
            if (parkingZone == 1) drive.followTrajectorySequence(trajSeq_C_B_Parking1rb);
            else if (parkingZone == 2) drive.followTrajectorySequence(trajSeq_C_B_Parking2rb);
            else drive.followTrajectorySequence(trajSeq_F_E_Parking3rb);
            return;
        }


    }





    public void spina(int counter) {
        stop_motor();
        driveturn(counter, 0.3);

        double angle;
        boolean back_mode=false;
        double target_angle = 0;
        double dist;
        double target_distance = 450;
        double time_passed=0;
        double angle_gap=2 ;
        double target_angle1=0;
        double target_distance1=500;
        Pose2d poseEstimate;
        int  count=0;

        start_time = runtime.seconds();
        while (time_passed< 0.5 && !back_mode && opModeIsActive()) {

            dist = cam_distance.getDistance(DistanceUnit.MM);
            while ((dist>600 ||dist<60)&& opModeIsActive())
            {
                dist = cam_distance.getDistance(DistanceUnit.MM);
            }

            if (dist<target_distance) {
                target_angle1= target_angle;
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                target_angle=poseEstimate.getHeading();
                target_distance1=target_distance;
                target_distance=dist;
                count++;
                telemetry.addData("target",dist);
                }
            else if (dist<target_distance1) {
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                target_angle1=poseEstimate.getHeading();
                target_distance1=dist;
                telemetry.addData("t Angle1",target_angle1);
            }

            time_passed=runtime.seconds()-start_time;

            if (count>0) {
                back_mode = true;
                driveturn(-counter, 0.25);
            }
           // telemetry.addData("fdist",dist);
        }

        start_time = runtime.seconds();
        time_passed=0;

        while (abs(angle_gap) > 0.02 && opModeIsActive()&&time_passed<0.4) {
            dist=cam_distance.getDistance(DistanceUnit.MM);
            drive.update();
            poseEstimate=drive.getPoseEstimate();
            angle=poseEstimate.getHeading();
           // telemetry.addData("back scan ",dist);
          //  telemetry.addData("back Angle",angle);
            if (dist<target_distance) {
                target_angle1= target_angle;
                target_angle=angle;
                target_distance1=target_distance;
                target_distance=dist;
                //telemetry.addData("target Angle",target_angle);
                }
            else if (dist<target_distance1) {
                target_angle1= angle;
                target_distance1=dist;
                //telemetry.addData("target1 Angle",target_angle1);
            }
            angle_gap = angle - target_angle;
            if (angle_gap > 0) counter = 1;
            else
                counter = -1;
            driveturn(counter, 0.25 );
            time_passed=runtime.seconds()-start_time;
        }
        stop_motor();

        dist=cam_distance.getDistance(DistanceUnit.MM);
       // telemetry.addData("final",dist);

        if(dist>350) {
        //    telemetry.addData("distance measurement over ",dist);
            target_angle=target_angle1;
            angle_gap=2.0;
            while (abs(angle_gap) > 0.02 && opModeIsActive()) {
                drive.update();
                poseEstimate=drive.getPoseEstimate();
                angle_gap = poseEstimate.getHeading()- target_angle;
                if (angle_gap > 0) counter = 1;
                else counter = -1;
                driveturn(counter, 0.25 );
            }

           // dist=cam_distance.getDistance(DistanceUnit.MM);
        }
        stop_motor();
       // front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   //     front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   drive.update();
      //  poseEstimate=drive.getPoseEstimate();
       // telemetry.addData("distance measurement ",dist);
       // telemetry.addData("Angle measurement",  poseEstimate.getHeading());
     //   telemetry.update();
       // sleep(5000000);
        //if(dist>350) return;//ACCIDENT PRCCESSING
        distance_outtake2(dist);

    }

    private double abs(double input) {
        if (input > 0) return (input);
        else
            return (-input);
    }

    private void linearslidets(int target, int speed) {
        linear_slide.setTargetPosition(target);
        linear_slide.setVelocity(speed);

    }




    public void straft (int left ,double power)

    {
        drive.setMotorPowers(-power*left, power*left, -power*left, power*left);
    }

    public void stop_motor()
    {drive.setMotorPowers(0, 0, 0, 0);}
    public void forward(double power)
    {drive.setMotorPowers(power, power, power, power);}

    public void distance_outtake2A(double spin_distance) {

       // double drive_distance=200;
        linearslidets(droplevel, 2600);//check
        double time_passed;
        int start_encoder;
        boolean touch_guide_flag=false;
        double tencoder=0;
        double encoder=0;
        int outtake_up_gap=-200;
        start_time = runtime.seconds();
        time_passed = 0;// old 0.3
       tencoder=spin_distance/tick_length-21;
        while (opModeIsActive() && (linear_slide.getCurrentPosition() > (droplevel+ 50 )) && time_passed < 0.5)
        {// +10 for tolerance

            time_passed = runtime.seconds() - start_time;

        }

        start_time = runtime.seconds();
        time_passed = 0;
        start_encoder= front_right.getCurrentPosition();

        telemetry.addData("start_encoder",start_encoder);
        start_time = runtime.seconds();

        forward(0.35);
        while(encoder-start_encoder<tencoder && opModeIsActive()&& time_passed<0.9&&!touch_guide_flag) {
            encoder=front_right.getCurrentPosition();
            if (!guide_touch.getState()) touch_guide_flag = true;
            time_passed=runtime.seconds()-start_time;
        }

        stop_motor();
        telemetry.addData("enc1",encoder);
      //  while ( right_distance.getDistance(DistanceUnit.MM) >drive_distance  && !touch_guide_flag &&time_passed <0.6 && opModeIsActive() )
       // {
      //      current_time = runtime.seconds();
        //    time_passed = current_time - start_time;


     //   }
        if (!guide_touch.getState()) touch_guide_flag = true;

        if(touch_guide_flag) {
            forward(-0.2);
            sleep(200);
            stop_motor();

        }

        while (opModeIsActive() && linear_slide.getCurrentPosition() > (droplevel+10)) {
            // telemetry.addData("initialize angle and you can mount the cone now",linear_slide.getCurrentPosition());

        }
        cam.setPosition(0);
        sleep(400);
        linearslidets(droplevel+outtake_up_gap, 2600);
        sleep(100);
        forward(-0.4);
        sleep(200);
        linearslidets(pk_l5,2600);
       // forward(-0.5);
        sleep(200);

       // linearslidets(pk_l5,2000);
        telemetry.update();
       // sleep(500000);

    }


//for fixed angle spin;
    public void spinb(int counter) {
        stop_motor();
        double dist;
        dist = cam_distance.getDistance(DistanceUnit.MM);
        telemetry.addData("distance measurement1 ",dist);

        double target=0;
        boolean back_mode=false;

        double time_passed=0;
        if(droplevel==dropLevel1) target=150;
        if(droplevel==dropLevel2)  target=230;
        dist = cam_distance.getDistance(DistanceUnit.MM);

        counter=1;
        if (dist>target) driveturn(counter, 0.2);
        start_time = runtime.seconds();
        while (time_passed< 0.6 && dist>target && opModeIsActive()) {
            dist = cam_distance.getDistance(DistanceUnit.MM);
            if(time_passed>0.2&&!back_mode) {
           counter = -counter;
           back_mode=true;
           driveturn(counter, 0.2);
            }
            time_passed=runtime.seconds()-start_time;
        }

      stop_motor() ;
    dist = cam_distance.getDistance(DistanceUnit.MM);
    if(dist <70 )        dist=cam_distance.getDistance(DistanceUnit.MM);

        //   drive.update();
        //  poseEstimate=drive.getPoseEstimate();
        //  telemetry.addData("distance measurement2 ",dist);
        // telemetry.addData("Angle measurement",  poseEstimate.getHeading());
      // if(droplevel==dropLevel2) telemetry.update();


        distance_outtake2(dist);

    }


    public void driveturn(int counter, double power) {
        drive.setMotorPowers(power*counter, power*counter, -power*counter, -power*counter);
    }

    private void dropCone(){
        // TODO make sure a smooth drop


        double encoder0;
        double encoder =0;
     //   spin(-1);
        slidePositionTo(dropLevel3,2600,0);

        //linear_slide.getCurrentPosition()>

        sleep(200);
        drive.setMotorPowers(0, 0, 0, 0);

        spin_distance=spin_distance-5;

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder0= front_right.getCurrentPosition();
        drive.setMotorPowers(0.35, 0.35, 0.35, 0.35);

        while ( encoder <(spin_distance/tick_length) )
        {
            encoder= front_right.getCurrentPosition()-encoder0;



        }

        drive.setMotorPowers(0.0,0.0,0.0,0.0);

        cam.setPosition(0);
        firstdrop=false;



        sleep(600);
        drive.setMotorPowers(-0.4,-0.4,-0.4,-0.4);
        sleep(400);
        drive.setMotorPowers(0.0,0.0,0.0,0.0);
        //sleep(500);
        linear_slide.setTargetPosition(-840);
        linear_slide.setVelocity(2200);
        sleep(500000);

    }
    public void distance_outtake2(double spin_distance) {
        sleep(150);
        spin_distance= cam_distance.getDistance(DistanceUnit.INCH)+1.66;
        // double drive_distance=200;
        linearslidets(droplevel, 2600);//check
        double time_passed;
        boolean far=false;
        boolean touch_guide_flag=false;

        int outtake_up_gap=-200;
        start_time = runtime.seconds();
        time_passed = 0;// old 0.3

        if(spin_distance>18||spin_distance<2) spin_distance=0;
        while (opModeIsActive() && (linear_slide.getCurrentPosition() > (droplevel+ 100 )) && time_passed < 0.5)
        {// +10 for tolerance

            time_passed = runtime.seconds() - start_time;

        }
        if(spin_distance>0) moveforward(spin_distance);
        else  far=true;

        if(far)  {
            forward(0.35);
            sleep(150);
        }
        start_time = runtime.seconds();
        time_passed = 0;
        while ( right_distance.getDistance(DistanceUnit.MM) > 160 && !touch_guide_flag &&time_passed < 1.0&& far && opModeIsActive() )
        {
            time_passed = runtime.seconds() - start_time;
        }
        if (far) sleep(20);
        stop_motor();
        if(!guide_touch.getState()) {
            forward(-0.2);
            sleep(200);
            stop_motor();
        }

        while (opModeIsActive() && linear_slide.getCurrentPosition() > (droplevel+10)) {

        }
        cam.setPosition(0);
        sleep(300);
        linearslidets(droplevel+outtake_up_gap, 2600);
        sleep(100);
        if(droplevel<dropLevel2) forward(-0.45);
        else forward(-0.6);
        sleep(200);
        linearslidets(pk_l5,2000);
        sleep(100);


    }

    private void moveforward(double DISTANCE) {

       // clocation = drive.getPoseEstimate();
        drive.update();
        clocation = drive.getPoseEstimate();
        Trajectory move = drive.trajectoryBuilder(clocation)
                .forward(DISTANCE)
                .build();
        drive.followTrajectory(move);
    }


     private void coloralign2 () {
         double start_time;
         boolean capture=false;
         boolean cerror=false;
         boolean display=true;
         int count=0;
         int btarget,rtarget,btri,rtri,bhigh, rhigh,cvalue,ctarget,chigh,ctri;
         int dir=1;
         int turn=0;
         btri=300;
         rtri=210;
         bhigh=375;
         rhigh=270;
         btarget=360;
         rtarget=240;
         if(!left_side)dir=-1;
         telemetry.update();

         if(blue_color) {
             ctarget=btarget;
             chigh=bhigh;
             ctri=btri;
         }
        else{
             ctarget=rtarget;
             chigh=rhigh;
             ctri=rtri;
         }

        if(blue_color) cvalue= color.blue();
        else cvalue= color.red();

         if (cvalue>chigh) {
             stop_motor();
             return;
         }
         if (cvalue>ctri) {
             capture=true;
             stop_motor();
             telemetry.addData("ctr1", cvalue);
         }

         if (!capture)straft(dir,0.25);
         start_time=runtime.milliseconds();
         while (!capture && !cerror && opModeIsActive()) {
             if (runtime.milliseconds() - start_time > 500)
                 straft(-dir,0.25);
             if (runtime.milliseconds() - start_time > 1100) {
                 straft(dir,0.2);
                 cerror = true;
             }
             if(blue_color) cvalue= color.blue();
             else cvalue= color.red();


             if (cvalue>ctri) {
                 capture=true;
                 stop_motor();
                 telemetry.addData("tr2", cvalue);
             }
             if (cvalue>chigh) {
                 stop_motor();
                 return;
             }
         }



         while ( capture && opModeIsActive()) {

             if (abs(count) > 120) {

                 dir = -dir;
                 turn++;
             }
             if (turn>1&&display) {
                 chigh= ctarget-10;
                 display=false;
             }

             straft(dir, 0.2);
             if (dir == 1) count++;
             else count--;
             if(blue_color) cvalue= color.blue();
             else cvalue= color.red();
           //  if(cvalue>360)  telemetry.addData("sl", cvalue);
             if(cvalue>ctarget) {
                ctarget = cvalue;
                 telemetry.addData("btarget", ctarget);
                 display=true;
             }

             if (cvalue>chigh) {
                 stop_motor();
                // telemetry.update();
                // sleep(5000000);
                 return;
             }
         }



     }


    private void location () {
        drive.update();
        Pose2d loc = drive.getPoseEstimate();
        telemetry.addData("finalX", loc.getX());
        telemetry.addData("finalY", loc.getY());
        telemetry.addData("finalHeading",  Math.toDegrees(loc.getHeading()));
        telemetry.update();

    }

    private void coloralign () {
        double start_time;
        boolean capture=false;
        boolean cerror=false;
        boolean display=true;
        int count=0;
        int btarget,rtarget,btri,rtri,bhigh, rhigh,cvalue,ctarget,chigh,ctri;
        int dir=1;
        int turn=0;
        btri=300;
        rtri=210;
        bhigh=370;
        rhigh=260;
        btarget=360;
        rtarget=240;
        if(!left_side)dir=-1;
        telemetry.update();

        if(blue_color) {
            ctarget=btarget;
            chigh=bhigh;
            ctri=btri;
        }
        else{
            ctarget=rtarget;
            chigh=rhigh;
            ctri=rtri;
        }

        if(blue_color) cvalue= color.blue();
        else cvalue= color.red();

        if (cvalue>chigh) {
            stop_motor();
            return;
        }

        if (!capture)straft(dir,0.25);
        start_time=runtime.milliseconds();
        while (!capture && !cerror && opModeIsActive()) {
            if (runtime.milliseconds() - start_time > 500)
                straft(-dir,0.25);
            if (runtime.milliseconds() - start_time > 1100) {
                straft(dir,0.25);
                cerror = true;
            }
            if(blue_color) cvalue= color.blue();
            else cvalue= color.red();


            if (cvalue>chigh) {
                stop_motor();
                return;
            }
        }

    }




    private void coloraligna () {
        double start_time;
        boolean capture=false;
        boolean cflag=false;
        boolean cerror=false;
        boolean display=true;
        int count=0;
        int   bcolor,rcolor,bcolortarget,rcolortarget;
        int dir=1;
        int turn=0;
        int bhigh,rhigh;
        bhigh=390;
        rhigh=270;
        bcolortarget=370;
        rcolortarget=240;
        bcolor=color.blue();
        rcolor=color.red();
        if (bcolor > bhigh ||(rcolor>rhigh)) {
            stop_motor();


            //  telemetry.addData("f1bcolor", bcolor);
            //   telemetry.addData("f1rcolor", rcolor);

            return;
        }
        if (bcolor > 240 ||(rcolor > 190)) {
            capture=true;
            telemetry.addData("c1bcolor", bcolor);
            //  telemetry.addData("c1rcolor", rcolor);

        }

        if (!capture)straft(1,0.25);
        start_time=runtime.milliseconds();
        while (!capture && !cerror && opModeIsActive()) {
            if (runtime.milliseconds() - start_time > 350)
                straft(-1,0.25);
            if (runtime.milliseconds() - start_time > 800) {
                straft(1,0.25);
                cerror = true;
            }
            bcolor = color.blue();
         //   rcolor = color.red();

            if(bcolor>200) {
                telemetry.addData("wb", bcolor);
                //   telemetry.addData("wr", rcolor);
            }

            if (bcolor > 240 ||(rcolor > 190)) {
                capture=true;
                stop_motor();
                telemetry.addData("c2bcolor", bcolor);
                // telemetry.addData("c2rcolor", rcolor);
            }
        }
        //   telemetry.update();
        //  sleep(3000);

        if (bcolor > bhigh ||(rcolor> rhigh)) {
            stop_motor();
            //  telemetry.addData("f2b", bcolor);
            // telemetry.addData("f2rcolor", rcolor);
            return;
        }
        stop_motor();
        while (!cflag && capture && opModeIsActive()) {

            if (abs(count) > 100) {

                dir = -dir;
                turn++;
            }
            if (turn>1&&display) {

                bhigh= bcolortarget-10;
                rhigh=rcolortarget-10;
                display=false;
                telemetry.addData("turn ", turn);
                telemetry.addData("btarget",bcolortarget);
                //  telemetry.addData("rtarget", rcolortarget);
            }

            straft(dir, 0.2);
            if (dir == 1) count++;
            else count--;
            bcolor = color.blue();
           // rcolor = color.red();
            if(bcolor>360) {
                telemetry.addData("nb", bcolor);
                //  telemetry.addData("nrcolor", rcolor);
            }
            if(bcolor>bcolortarget) {
                bcolortarget = bcolor;
                telemetry.addData("btarget", bcolor);
                display=true;
            }
            if(rcolor>rcolortarget) rcolortarget=rcolor;

            if (bcolor > bhigh|| rcolor > rhigh) {
                cflag = true;
                stop_motor();

                // telemetry.addData("f3b", bcolor);
                // telemetry.addData("f3rcolor", rcolor);
                // telemetry.update();
                //  sleep(500000);
                return;

            }
        }



    }



    /**
     * pick the cone, and move the slide to next position
     * @param indexOfPick
     * @param targetPosition
     */



    private void pickCone(int indexOfPick, int targetPosition){
        // TODO test for the right heights and enough wait time

      //  color.enableLed(true);

       // telemetry.update();
        coloralign();

        telemetry.update();

       // sleep(5000000);
        //while(color.blue() <420&& (color.red()<270)&&!error&&opModeIsActive()){
         //  if(runtime.milliseconds()-start_time>400)
         //      straft(-1);
         // if(runtime.milliseconds()-start_time>1000){
         //     straft(1);
          //    error=true;
        //   }
        //}

      //  telemetry.addData("blue: ", color.blue());
     //   telemetry.addData("red: ", color.red());

       // if(error) sleep(600);// move the robot back to the normal track
       // stop_motor();
        gyro=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (left_side)
        drive.turn(Math.toRadians(-90-gyro));
        else
            drive.turn(Math.toRadians(90-gyro));
        stop_motor();
       // telemetry.addData("blue: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
       // telemetry.update();
        if(error) return;

        forward(0.18);
        //sleep (400);



        sleep(200);
        guide_touch_flag=false;
        take_touch_flag=false;
        runtime.reset();
        while (!guide_touch_flag && opModeIsActive() && runtime.seconds() <1.2){
            if(!guide_touch.getState()&&abs(front_right.getVelocity())<5) guide_touch_flag=true;
        }
        // sleep(100);
        stop_motor();
        slidePositionTo(conePosition2stack,1600,0);
        if (runtime.seconds() > 1.2) error = true;
        runtime.reset();
        while(opModeIsActive() && !take_touch_flag && runtime.seconds() < 1.2){

            take_touch_flag=!take_touch.getState()&&(linear_slide.getCurrentPosition()>-700);
        }

        cam.setPosition(0.33);
        // drive.setMotorPowers(0,0,0,0);
        linear_slide.setVelocity(0);
        if (runtime.seconds() > 1.2) error = true;
        // pick

        sleep(300);
        //current_pos = linear_slide.getCurrentPosition();
        slidePositionTo( droplevel+prset_gap, 2000, 0);


        // move out of the stack first, then move further in another thread to take advantage of the moving time
        // if the move distance too short, then just use slidePositionTo to targetPosition directly
        //current_pos = linear_slide.getCurrentPosition();
        while (linear_slide.getCurrentPosition() > -900 ){
            // slidePositionTo(linear_slide.getCurrentPosition() - 500, 2800, 0);
        }
        //  slideUpAsyncTo(targetPosition);
       // forward(-0.5);
       // sleep(200);
       // drive.setMotorPowers(0,0,0,0);

    }

//    /**
//     * Check position for the waiting time - seems not that accurate
//     * @param targetPosition
//     * @param moveLower
//     */
//    private void waitSlideToPosition(int targetPosition, boolean moveLower){
//        int currentPosition = linear_slide.getCurrentPosition();
//        boolean reached = moveLower? currentPosition > targetPosition:currentPosition<targetPosition;
//
//        while(!reached && Math.abs(currentPosition - targetPosition)>20) {
//            sleep(50);
//            currentPosition = linear_slide.getCurrentPosition();
//            reached = moveLower? currentPosition > targetPosition:currentPosition<targetPosition;
//        }
//        sleep(50);
//    }

    /**
     * move slide to a position, given an option to wait or not for the slide to reach the position
     * @param targetPosition
     * @param velocity
     * @param timeout 0 means no timeout
     */



    private void slidePositionTo(int targetPosition, double velocity, long timeout){
        linear_slide.setTargetPosition(targetPosition);
        linear_slide.setVelocity(velocity);

        if(timeout > 0) {
            sleep(timeout);
        }
    }


    private void deactivateTFOD(){
        Thread t = new Thread(){
            public void run(){
                if(tfod != null)
                    tfod.deactivate();
            }
        };
        t.start();
    }

    private int getParkingZoneFromPic(){
        int parkingzone = defaultParkingzone;
        long start = System.currentTimeMillis();
        boolean toDeactivated = true;
        while (opModeIsActive() && !isStopRequested() && tfod != null && (System.currentTimeMillis() - start) <300) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && !updatedRecognitions.isEmpty()) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                String objLabel = updatedRecognitions.get(0).getLabel();
                if(objLabel != null) {
                    if (objLabel.equals("Apple")) {
                        parkingzone = 1;
                    } else if (objLabel.equals("Banana")) {
                        parkingzone = 2;
                    } else if (objLabel.equals("Grape")) {
                        parkingzone = 3;
                    }
                }
                telemetry.addData("Image: ", objLabel==null?"null":objLabel);
                telemetry.addData("Parking zone", "" + parkingzone);
                telemetry.update();
                deactivateTFOD();
                toDeactivated = false;
                break; // found it
            }
        }
        if(toDeactivated)
            deactivateTFOD();

        return parkingzone;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private TFObjectDetector initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        try {
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        }catch (Exception e){
            return null;
        }
        return tfod;
    }
}
