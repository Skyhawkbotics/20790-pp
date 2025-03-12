package pedroPathing.opmode;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
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

@TeleOp(name = "opmode MAIN ", group = "MAIN")
public class opmode_MAIN_Redesign extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
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
    double servo_intake_rotate_location = 0.47;

    double servo_intake_open = 0.9;
    double servo_intake_closed = 0.25;
    double servo_intake_location = 0.5;
    int out_max_pos = 1330;

    int up_specimen_hang = 1907; // Viper

    double outtake_specimen_hang = 0.45;

    double driving_multiplier_fast = 0.7;
    double driving_multiplier_slow = 0.3;

    double driving_multiplier;

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

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();

        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //from rr version

        //setup arm to use velocity
        //setup arm variable
        up1 = hardwareMap.get(DcMotorEx.class, "up1");
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);

        up2 = hardwareMap.get(DcMotorEx.class, "up2");
        up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo_intake = hardwareMap.get(Servo.class, "intake");
        servo_intake_rotate = hardwareMap.get(Servo.class, "intakeRotate");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");


        servo_outtake_flip1 = hardwareMap.get(Servo.class, "ofip1");
        servo_outtake_flip2 = hardwareMap.get(Servo.class, "ofip2");

        servo_outtake = hardwareMap.get(Servo.class, "outtake");
        servo_outtake_rotate = hardwareMap.get(Servo.class, "outtaker");

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");

        Pose startPose = new Pose(15, 40.0, Math.toRadians(0)); //TODO
        follower.setStartingPose(startPose);

        inisclosed = false;

        outisclosed = false;


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
        viper_slide();
        misumi_slide();
        intake_claw();
        outtake_claw();
        if(!defend) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_x * 0.5);
        } else if (defend){
            follower.holdPoint(follower.getPose());
        }
        if (currentgamepad1.triangle && !previousgamepad1.triangle) {
            defend = true;
            follower.breakFollowing();
        }
        if (currentgamepad1.circle && !previousgamepad1.circle) {
            defend = false;
            follower.startTeleopDrive();
        }

            follower.update();

        telemetry.addData("gamepad2.rightstickx", gamepad2.right_stick_x);
        telemetry.addData("gamepad2.rightsticky", gamepad2.right_stick_y);
        telemetry.addData("up1 pos", up1.getCurrentPosition());
        telemetry.addData("up2 pos", up2.getCurrentPosition());
        telemetry.addData("servo intake pos", servo_intake.getPosition());
        telemetry.addData("servo rotate pos", servo_intake_rotate.getPosition());
        telemetry.addData("servo wrist ", servo_intake_wrist.getPosition());
        telemetry.addData("gamepad2 left stick x", gamepad2.left_stick_x);
        telemetry.addData("flip1", servo_outtake_flip1.getPosition());
        telemetry.addData("flip2", servo_outtake_flip2.getPosition());
        telemetry.addData("defend", defend);

        telemetry.update();
    }


    public void viper_slide() {
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
        } else if (gamepad2.right_stick_y > 0.5) { // Down
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
    }
    public void misumi_slide() {
        // Misumi Slide
        if (gamepad2.dpad_right) { //in
            //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
            out.setPower(0.2);
            out_true_target_pos = 0;
        } else if (gamepad2.dpad_left) { //out
            out.setPower(-0.2);
        /*} else if (gamepad2.dpad_right) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("reset out", true);

         */
        } else {
            out.setPower(0);
            out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void outtake_claw() {
        // manual outtake wrist location
        if (gamepad2.dpad_down) {
            servo_outtake_flip2.setPosition(1);
            servo_outtake_flip1.setPosition(0);
            servo_outtake_rotate.setPosition(0.1);
        }
        if (gamepad2.dpad_up) {
            servo_outtake_flip2.setPosition(0.60);
            servo_outtake_flip1.setPosition(0.4);
            servo_outtake_rotate.setPosition(0.9);
        }
        if (gamepad2.share) {
            servo_outtake_flip2.setPosition(0.5);
            servo_outtake_flip1.setPosition(0.5);
            servo_outtake_rotate.setPosition(0.9);
        }
        if (currentgamepad2.left_bumper && !previousgamepad2.left_bumper) {
            outisclosed = !outisclosed;

        }
        if(outisclosed) {
            servo_outtake.setPosition(0.0);
        }
        if(!outisclosed) {
            servo_outtake.setPosition(0.7);
        }
    }
    public void intake_claw() {
        //servo intake control
        if (currentgamepad2.right_bumper && !previousgamepad2.right_bumper) {
            inisclosed = !inisclosed;

        }
        if(inisclosed) {
            servo_intake.setPosition(0.5);
        }
        if(!inisclosed) {
            servo_intake.setPosition(servo_intake_open);
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


       // manual intake wrist location
        if (gamepad2.cross) { // down pos
            servo_intake_wrist.setPosition(0.6);
        }
        if (gamepad2.triangle) { // ready pos
            servo_intake_wrist.setPosition(0.45);
        }
        if(gamepad2.options) {
            servo_intake_wrist.setPosition(0);

        }
       /* if (gamepad2.circle) {
            servo_intake_wrist_location += 0.05;
        }
        if (gamepad2.square) {
            servo_intake_wrist_location -= 0.05;
        }

        // limits
        if (servo_intake_wrist_location > 1) {
            servo_intake_wrist_location = 1;
        } else if (servo_intake_wrist_location < 0) {
            servo_intake_wrist_location = 0;
        }

        servo_intake_wrist.setPosition(servo_intake_wrist_location);

        */
    }
}