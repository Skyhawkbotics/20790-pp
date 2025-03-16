package pedroPathing.opmode;


import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;


import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "opmode nnnniiga ", group = "MAIN")
public class opmode_MOTOR_TEST extends OpMode {
    private Follower follower;


    private DcMotorEx leftFront;
    private DcMotorEx leftRear;

    boolean hang = false;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;




    private Servo servo_outtake_flip1, servo_outtake_flip2, servo_intake_wrist, servo_intake_rotate, sweeper, servo_intake, servo_outtake, servo_outtake_rotate;
    private DcMotorEx up1, up2, out;
    private TouchSensor up_zero, out_zero;
    //from rr version of opmode_MAIN
    int arm_upper_lim = 2700;
    int up_true_target_pos;
    int out_true_target_pos;
    double servo_outtake_flip_location = 0;
    double servo_intake_wrist_location = 0.7;


    boolean goingdown = false;


    boolean defend = false;


    boolean outisclosed = false;


    boolean inisclosed = false;

    Path align;
    double servo_intake_rotate_location = 0.47;


    double servo_intake_open = 0.9;
    double servo_intake_closed = 0.25;
    double servo_intake_location = 0.5;
    int out_max_pos = 1330;

    double servo_outtake_location= 0.5;

    double servo_outtake_r = 0.5;

    double pivot_pos = 0;

    double pivot_pose2 = 1;
    int up_specimen_hang = 1907; // Viper


    double outtake_specimen_hang = 0.45;


    double driving_multiplier_fast = 0.7;
    double driving_multiplier_slow = 0.3;


    double driving_multiplier;

    int nigga;
    int nigga2;

    private final ElapsedTime runtime = new ElapsedTime();


    private Gamepad currentgamepad2 = new Gamepad();
    private Gamepad previousgamepad2 = new Gamepad();
    //path
    private Gamepad currentgamepad1 = new Gamepad();
    private Gamepad previousgamepad1 = new Gamepad();
    private PathChain park;


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void start() {
        //PATHING
    }


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        follower.startTeleopDrive();


        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //from rr version


        //setup arm to use velocity
        //setup arm variable
        /*up1 = hardwareMap.get(DcMotorEx.class, "up1");
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);


        up2 = hardwareMap.get(DcMotorEx.class, "up2");
        up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         */

        servo_intake = hardwareMap.get(Servo.class, "intake");
        servo_intake_rotate = hardwareMap.get(Servo.class, "intakeRotate");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");




        servo_outtake_flip1 = hardwareMap.get(Servo.class, "ofip1");
        servo_outtake_flip2 = hardwareMap.get(Servo.class, "ofip2");


        servo_outtake = hardwareMap.get(Servo.class, "outtake");
        servo_outtake_rotate = hardwareMap.get(Servo.class, "outtaker");


        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");


        out_zero = hardwareMap.get(TouchSensor.class, "out_zero");


        Pose startPose = new Pose(15, 40.0, Math.toRadians(0)); //TODO
        follower.setStartingPose(startPose);


        //inisclosed = false;


        //outisclosed = false;




    }


    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {


        previousgamepad2.copy(currentgamepad2);
        currentgamepad2.copy(gamepad2);
        previousgamepad1.copy(currentgamepad1);
        currentgamepad1.copy(gamepad1);
        //viper_slide();
        //misumi_slide();
        intake_claw();
        outtake_claw();
        /*
        if(!defend) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_x * 0.5);
        } else if (defend){
            follower.holdPoint(follower.getPose());
        }
        if (currentgamepad1.triangle && !previousgamepad1.triangle) {
            defend = !defend;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }
        if (currentgamepad1.circle && !previousgamepad1.circle) {
            defend = false;
            follower.startTeleopDrive();
        }
        if(currentgamepad1.left_bumper && !previousgamepad1.left_bumper) {
            follower.setPose(new Pose(20, 20, 0));
        }
        if(currentgamepad1.right_bumper && !previousgamepad1.right_bumper) {
            follower.breakFollowing();
            align = new Path(
                    new BezierLine(
                            new Point(follower.getPose()),
                            new Point(20,20,0)
                    )
            );
            align.setConstantHeadingInterpolation(0);
            align.setZeroPowerAccelerationMultiplier(1);
            follower.setMaxPower(0.5);
            follower.followPath(align);
        }

         */




        /*telemetry.addData("gamepad2.rightstickx", gamepad2.right_stick_x);
        telemetry.addData("gamepad2.rightsticky", gamepad2.right_stick_y);
        telemetry.addData("up1 pos", up1.getCurrentPosition());
        telemetry.addData("up2 pos", up2.getCurrentPosition());
        telemetry.addData("servo intake pos", servo_intake.getPosition());
        telemetry.addData("servo rotate pos", servo_intake_rotate.getPosition());
        telemetry.addData("servo wrist ", servo_intake_wrist.getPosition());
        telemetry.addData("gamepad2 left stick x", gamepad2.left_stick_x);

         */
        telemetry.addData("flip1", servo_outtake_flip1.getPosition());
        telemetry.addData("flip2", servo_outtake_flip2.getPosition());




telemetry.addData("servo out pos", servo_outtake.getPosition());
        telemetry.addData("servo out rotate pos", servo_outtake_rotate.getPosition());


        telemetry.update();
        follower.update();

    }




    public void viper_slide() {
        /*if(up1.getCurrentPosition() == arm_upper_lim) {
            telemetry.addData("upper limit reached", true);
        }
        if (gamepad2.right_stick_y < -0.5) { // Up
            goingdown = false;
            up1.setTargetPosition(arm_upper_lim);
            up2.setTargetPosition(arm_upper_lim);


            up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            up1.setPower(1);
            up2.setPower(1);


            telemetry.addData("up1", true);
            telemetry.addData("up2", true);
        } else if (gamepad2.right_stick_y > 0.5 && !up_zero.isPressed()) { // Down
            goingdown = true;
        } else if(gamepad2.right_stick_y == 0) {
            goingdown = false;
            up1.setTargetPosition(up1.getCurrentPosition());
            up2.setTargetPosition(up1.getCurrentPosition());


            up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            up1.setPower(1);
            up2.setPower(1);
        }
        if(gamepad2.touchpad) {
            hang = true;
            nigga = up1.getCurrentPosition();
            nigga2 = up2.getCurrentPosition();
        }
        if(hang) {
            up1.setTargetPosition(nigga);
            up2.setTargetPosition(nigga2);
            up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up1.setPower(1);
            up2.setPower(1);


        }
        if(goingdown) {
            if(!up_zero.isPressed()) {
                up1.setTargetPosition(-1000);
                up2.setTargetPosition(-1000);
                up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up1.setPower(-1);
                up2.setPower(-1);
            } else if(up_zero.isPressed()) {
                up1.setPower(0);
                up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                up2.setPower(0);
                up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                goingdown = false;
            }
        }

         */
    }
    /*public void misumi_slide() {
        // Misumi Slide
        if (gamepad2.dpad_right && !out_zero.isPressed()) { //in
            out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
            out.setPower(0.2);
        } else if (gamepad2.dpad_left ) { //out
            out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            out.setPower(-0.2);
        } else if (gamepad2.dpad_right && out_zero.isPressed()) {
            out.setPower(0);
            telemetry.addData("limit reached", true);

        } else {
            out.setPower(0);
            telemetry.addData("no power for out slide", true);
        }
    }

     */
    public void outtake_claw() {
        // manual outtake flip location //TODO: switch if needed
       /* if (gamepad2.cross) { //arm pickup
            servo_outtake_flip2.setPosition(1);
            servo_outtake_flip1.setPosition(0);
            servo_outtake_rotate.setPosition(0.1);
        }
        if (gamepad2.triangle) { //arm up
            servo_outtake_flip2.setPosition(0.6);
            servo_outtake_flip1.setPosition(0.5);
            servo_outtake_rotate.setPosition(0.75);
        }
        if (gamepad2.options) { //arm hang pos (pull down)
            servo_outtake_flip2.setPosition(0.3);
            servo_outtake_flip1.setPosition(0.7);
            servo_outtake_rotate.setPosition(0.75);
        }

        */
        /*
        if (gamepad2.left_stick_x > 0.1) {
            pivot_pos -= 0.015;
            pivot_pose2 += 0.015;
        }
        if (gamepad2.left_stick_x < -0.1) {
            pivot_pos += 0.015;
            pivot_pose2 -= 0.015;
        }



        if (pivot_pos > 1) {
            pivot_pos = 1;
        } else if (pivot_pos < 0) {
            pivot_pos = 0;
        }


        if (pivot_pose2 > 1) {
            pivot_pose2 = 1;
        } else if (pivot_pose2 < 0) {
            pivot_pose2 = 0;
        }
        servo_outtake_flip1.setPosition(pivot_pos); // todo : 0.015 is the 0 pos , 0.435 ;  1  is the  tranfser
        servo_outtake_flip2.setPosition(pivot_pose2); // todo : 0.985 is the 0 pos, however 1 works as well 0.565  , transfer is 0



         */


        if (gamepad2.left_stick_x > 0.1) {
            servo_outtake_location -= 0.015;
        }
        if (gamepad2.left_stick_x < -0.1) {
            servo_outtake_location += 0.015;
        }

        if (servo_outtake_location > 1) {
            servo_outtake_location = 1;
        } else if (servo_outtake_location < 0) {
            servo_outtake_location = 0;
        }
        /// todo 0.805 for rotate
        // toido niga 0.165 hang
        /// todo 0.105 for outtake clsoed
        /// todo 0.705 open
        servo_outtake.setPosition(servo_outtake_location);
        if (gamepad2.left_stick_y > 0.1) {
            servo_outtake_r -= 0.015;
        }
        if (gamepad2.left_stick_y < -0.1) {
            servo_outtake_r += 0.015;
        }

        if (servo_outtake_r > 1) {
            servo_outtake_r = 1;
        } else if (servo_outtake_r < 0) {
            servo_outtake_r = 0;
        }
        servo_outtake_rotate.setPosition(servo_outtake_r);




    }
    public void intake_claw() {
        //servo intake control
        /*if (currentgamepad2.right_bumper && !previousgamepad2.right_bumper) {
            inisclosed = !inisclosed;

        }
        if(inisclosed) {
            servo_intake.setPosition(0.5);
        }
        if(!inisclosed) {
            servo_intake.setPosition(0.9);
        }

        //intake rotate reset
        if (gamepad2.share) {
            servo_intake_rotate_location = 0.5; //TODO: tune if needed
            servo_intake_wrist.setPosition(0.25);
        }
        // manual intake rotate location
        if (gamepad2.left_stick_x > 0.1) {
            servo_intake_rotate_location -= 0.015;
        }
        if (gamepad2.left_stick_x < -0.1) {
            servo_intake_rotate_location += 0.015;
        }


        if (servo_intake_rotate_location > 1) {
            servo_intake_rotate_location = 1;
        } else if (servo_intake_rotate_location < 0) {
            servo_intake_rotate_location = 0;
        }


        servo_intake_rotate.setPosition(servo_intake_rotate_location);


        //intake wrist control
        if (gamepad2.dpad_up) {
            servo_intake_wrist.setPosition(0.45);
        }
        if (gamepad2.dpad_down) {
            servo_intake_wrist.setPosition(0.55);
        }


         */
    }
}
