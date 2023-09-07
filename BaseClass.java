package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;
import   java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class BaseClass extends SampleMecanumDrive {
    protected DcMotor front_left;
    protected DcMotorEx front_right;
    protected DcMotor rear_left;
    protected DcMotor rear_right;
    protected DcMotorEx intake_left;
    protected DcMotorEx intake_right;
    protected DcMotorEx outtake_left;
    protected DcMotorEx outtake_right;

    protected Servo intake_grab;
    protected Servo intake_handle;
    protected Servo intake_arm;

    protected Servo outtake_cam;
    protected Servo outtake_handle;
    protected Servo outtake_arm;
    protected Servo uservo;
   protected  ColorSensor grab_color;

    //protected DistanceSensor grab_distance;
    protected TouchSensor cam_touch;

    protected Servo control_turn;
    protected Servo expansion_turn;
    sservo in_grab = new sservo();
    sservo in_handle = new sservo();
    sservo in_arm = new sservo();

    sservo out_cam = new sservo();
    sservo out_handle = new sservo();
    sservo out_arm = new sservo();

    sservo con_turn = new sservo();
    sservo exp_turn = new sservo();

 //   Thread parallel = new Thread();
    private ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode Op;

    static final double PRE_INGRAB = 0;
    static final double PRE_INPARTGRAB = 0.2;
    static final double PRE_INHANDLE = 0;
    static final double PRE_INARM = 0.1;//0.1
    static final double cam_unlock = 0.3;
    static final double cam_lock = 0;
    static final double PRE_OUTARM = 0;
    static final double PRE_OUTHANDLE = 0.31;//0.31


    static final double GRAB_CLOSE = 0.36;

    static final double TRAN_INGRAB = 0.2;
    static final double TRAN_INHANDLE = 0.61;
    static final double TRAN_INARM = 0.6;
    static final double TRAN_OUTCAM = 0;
    static final double TRAN_OUTHANDLE= 0.36;


    static final double DROP_OUTARM = 0.8;//0.92
    static final double DROP_OUTCAM = 0.3;
    static final double DROP_OUTHANDLE = 0.2;

    int caro_cent_index = 5;
    int caro_index=caro_cent_index;
    boolean carousel_lock=false;
    boolean has_cone=false;
    int[] out_linear_pos =  {1698,1800,1900, 2000,2108};
    double[] out_arm_pos  = {0.72,0.77,0.82, 0.87, 0.92 };
   double [] out_handle_pos={0.19,0.22,0.25 ,0.28,0.31};

    int[] out_linear_pos2 =  {1698,1800,1900, 2000,2108};
    double[] out_arm_pos2  = {0.72,0.77,0.82, 0.87, 0.92 };
    double [] out_handle_pos2={0.19,0.22,0.25 ,0.28,0.31};


    final double[] carousel = {0.02,0.04 , 0.06, 0.08, 0.10, 0.12,0.14,0.16,0.18,0.20,0.22,0.24,0.26,0.28,0.30};


    public BaseClass (LinearOpMode linearOpMode){
        super(linearOpMode.hardwareMap);
        Op = linearOpMode;

        initHardware(Op.hardwareMap);
    }

    public BaseClass(HardwareMap hardwareMap) {
        super(hardwareMap);
        initHardware(hardwareMap);

    }

    void post_outtake (int position, int level) {
        double post_out_arm,post_out_handle;

        if(position>=2107 || position<= 1699) return;
        int ans = -1;
        for (int i = 0;i<5;i++){
            if ((position+0.001)>out_linear_pos[i]){// +0.001 to avoid the equal
                ans = i;
            }
            else break;

        }

        post_out_arm= out_arm_pos[ans]+ (out_arm_pos[ans+1]-out_arm_pos[ans])*(position-out_linear_pos[ans])/(out_linear_pos[ans+1]-out_linear_pos[ans]);
        post_out_handle= out_handle_pos[ans]+ (out_handle_pos[ans+1]-out_handle_pos[ans])*(position-out_linear_pos[ans])/(out_linear_pos[ans+1]-out_linear_pos[ans]);
        out_arm.setp( post_out_arm);
        out_handle.setp( post_out_handle);
    }



    public void initHardware (HardwareMap hardwareMap){
        front_left = hardwareMap.get(DcMotor.class,"front_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        intake_left = hardwareMap.get(DcMotorEx.class, "intake_left");
        intake_right = hardwareMap.get(DcMotorEx.class,"intake_right");
        outtake_left = hardwareMap.get(DcMotorEx.class, "outtake_left");
        outtake_right = hardwareMap.get(DcMotorEx.class,"outtake_right");

        intake_grab = hardwareMap.get(Servo.class,"intake_grab");
        intake_handle = hardwareMap.get(Servo.class,"intake_handle");
        intake_arm = hardwareMap.get(Servo.class,"intake_arm");

        outtake_cam = hardwareMap.get(Servo.class,"outtake_cam");
        outtake_handle = hardwareMap.get(Servo.class,"outtake_handle");
        outtake_arm = hardwareMap.get(Servo.class,"outtake_arm");

        control_turn = hardwareMap.get(Servo.class,"control_turn");
        expansion_turn = hardwareMap.get(Servo.class,"expansion_turn");

      //  grab_distance = hardwareMap.get(DistanceSensor.class, "grab_distance");
        cam_touch = hardwareMap.get(TouchSensor.class,"cam_touch");
        grab_color = hardwareMap.get(ColorSensor.class,"grab_color");
        in_handle.temp = intake_handle;
        in_arm.temp = intake_arm;
        in_grab.temp=intake_grab;

        out_cam.temp = outtake_cam;
        out_handle.temp = outtake_handle;
        out_arm.temp = outtake_arm;

        con_turn.temp = control_turn;
        exp_turn.temp = expansion_turn;

//        con_turn.start = 0.006;
        con_turn.end = 0.33;
//
//        exp_turn.start = 0.25;
        exp_turn.end = 0.33;

        out_arm.end = 1.0;
        in_handle.end =0.8;

        in_arm.end = 1.0;

        out_arm.end = 1.0;


        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtake_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_left.setTargetPosition(0);
        intake_right.setTargetPosition(0);

        outtake_left.setTargetPosition(0);
        outtake_right.setTargetPosition(0);

        intake_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtake_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }






    protected void robot_centric(double iy,double ix, double irx) {
        double y = iy; // Remember, this is reversed!
        double x = -ix* 1.1; // Counteract imperfect strafing
        double rx = -irx * 0.75;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        front_left.setPower(-frontLeftPower);
        front_right.setPower(-frontRightPower);
        rear_left.setPower(-backLeftPower);
        rear_right.setPower(-backRightPower);
    }

    public void intake() {

        // start intake sequence



       // intake_slide(2800, 1000);

//        while (grab_distance.getDistance(DistanceUnit.MM) >= 75) {
//
//        }

//        while (grab_color.red()<100 & grab_color.blue()<100) {
//
//        }
//        intake_left.setVelocity(0);
//        intake_right.setVelocity(0);

//        sleep(200);
//
        if(!has_cone) {
            intake_slide(2800, 800);
            sleep(250);
            in_grab.setp(GRAB_CLOSE); //close grabber;
            intake_left.setVelocity(0);
            intake_right.setVelocity(0);
            in_grab.setp(GRAB_CLOSE); //close grabber;
            sleep(400);
        }
        else
            return;


        in_arm.setp(TRAN_INARM );//-0.2
        in_handle.setp(TRAN_INHANDLE - 0.2);
        caro_index=caro_cent_index;
        turn(carousel[caro_index]);
        carousel_lock=true;

        // sleep(100);
        intake_slide(0, 2000);


    }

        // bring slide back for transition
    public void transfer() {
        caro_index=caro_cent_index;
        turn(carousel[caro_index]);

        while(intake_right.getCurrentPosition() >= 50 ){

        }

        // intake to outtake transition code
     //   in_arm.setp(TRAN_INARM);
      //  sleep(450);
        in_handle.setp(TRAN_INHANDLE);
       while(!cam_touch.isPressed()){
       }
       // sleep(300);
        out_cam.setp(cam_lock); // open and lock
        in_grab.setp(TRAN_INGRAB);
        has_cone=true;

        sleep(250);

        //perpendicular
        in_arm.setp(PRE_INARM);
        in_grab.setp(PRE_INPARTGRAB);
        in_handle.setp(PRE_INHANDLE);

        out_arm.setp(DROP_OUTARM);
      //  out_handle.setp(TRAN_OUTHANDLE);
        carousel_lock= false;
        sleep (300);
        outtake_slide(1400,1800);


    }



    public void out_arm_adjust(int dir){
        outtake_arm.setPosition(outtake_arm.getPosition()+dir*0.01);
        //outtake_handle.setPosition(outtake_handle.getPosition()+0.03);
        sleep(500);
    }
    public void out_handle_adjust(int dir){

        outtake_handle.setPosition(outtake_handle.getPosition()+dir*0.01);
        sleep(500);
    }
//    public void arm_adjust2(){
//        outtake_arm.setPosition(outtake_arm.getPosition()-0.05);
//        outtake_handle.setPosition(outtake_handle.getPosition()-0.03);
//    }
    public void out_arm_hundle (int dir) {
        outtake_arm.setPosition(outtake_arm.getPosition()+dir*0.05);
        outtake_handle.setPosition(outtake_handle.getPosition()+dir*0.03);


    }



    public void outtake_ready(){
       if (outtake_right.getCurrentPosition()>100) out_arm.setp(PRE_OUTARM+0.10);
       else out_arm.setp(PRE_OUTARM);
        outtake_slide(0,1800);
        out_cam.setp(cam_unlock);
        out_handle.setp(PRE_OUTHANDLE);
        caro_index=caro_cent_index;
        turn(carousel[caro_index]);

        Thread thread = new Thread(){
            public void run(){
                while(outtake_right.getCurrentPosition()>100) {


                    if(outtake_right.getCurrentPosition()<400) {
                        outtake_slide(0,1000);
                        out_arm.setp(PRE_OUTARM+0.05);
                    }
                  //  if(outtake_right.getCurrentPosition()<200) out_arm.setp(PRE_OUTARM+0.05);

                }
                out_arm.setp(PRE_OUTARM);
            }
        };
        thread.start();


      // out_arm.setp(PRE_OUTARM);
        // out_arm.setp(PRE_OUTARM+0.1);
        //  sleep(1500);


       // sleep(200);
    }


    public void intake_ready(){


        in_grab.setp(PRE_INGRAB);
        in_handle.setp(PRE_INHANDLE);
        in_arm.setp(PRE_INARM);


    }



    public void turn(double tur){
        con_turn.setp(tur);
        exp_turn.setp(tur);

    }

    public void manual_drop(){
       // sleep(100);
        out_cam.setp(cam_unlock);
        sleep(200);

        out_handle.setp(DROP_OUTHANDLE);
        has_cone=false;
        sleep(200);
        out_arm.setp(DROP_OUTARM-0.45);
        sleep(300);

        outtake_ready();

    }



    public class sservo{
        double start=0.0;
        double end =0.5 ;
        public Servo temp;


          void setp(double pos){

            if(pos>=0 && pos+start<=end){
                temp.setPosition(pos+start);
            }

        }
    }

public void sleep(double mstime) {
    double start_time= runtime.milliseconds();
   while (runtime.milliseconds()-start_time<mstime) {

   }


}

public void intake_slide(int targ,int vel){
    if( targ<2810 && targ>-1  ) {

        intake_left.setTargetPosition(-targ);
        intake_right.setTargetPosition(targ);

        intake_left.setVelocity(vel);
        intake_right.setVelocity(vel);
    }
}

public void outtake_slide(int targ,int vel){
     if( targ<2200 && targ>-1  ){
        outtake_left.setTargetPosition(-targ);
        outtake_right.setTargetPosition(targ);

        outtake_left.setVelocity(vel);
        outtake_right.setVelocity(vel);
    }
}

// 100 encoder per touch
public void m_outtake_slide(float dir) {


//      intake_slide((intake_left.getCurrentPosition()+ Math.round(dir*100)),1500);
    if (dir>0.1){
        outtake_slide(outtake_right.getCurrentPosition()+100,1500);
        sleep(1000);
    }
    if (dir<-0.1){
        outtake_slide(outtake_right.getCurrentPosition()- 100,1500);
        sleep(1000);
    }
}
    public void adjust_outtake_slide(float dir, int level) {

         int gap=outtake_right.getTargetPosition()-outtake_right.getCurrentPosition();
          int tar=outtake_right.getCurrentPosition() + Math.round(dir * 100);
         if (( dir >0 && gap<100 ) || (dir<0 && gap>-100)) {

             outtake_slide((tar), 1500);
             post_outtake(tar,level);

         }

    }

    public void adjust_intake_slide(float dir) {

        int gap=intake_right.getTargetPosition()-intake_right.getCurrentPosition();
        if (( dir >0 && gap<200 ) || (dir<0 && gap>-200))


            intake_slide((intake_right.getCurrentPosition()+ Math.round(dir*150)),2000);


    }

public void carousel_turn(boolean colwise)

{        if (carousel_lock) return;
        double gap= carousel[caro_index]-expansion_turn.getPosition();
        if(colwise && caro_index < 13&& gap<0.02)
            caro_index++;
        else if (!colwise && caro_index >0&&gap>-0.02)
            caro_index--;


        turn(carousel[caro_index]);

        sleep(200);

    }




public void wait_while(boolean condition){
        while(condition){

        }
}


}
