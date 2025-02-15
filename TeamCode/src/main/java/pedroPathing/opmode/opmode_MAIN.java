package pedroPathing.opmode;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName; // unsure if needed, but they are motors
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "opmode_MAIN", group = "MAIN")
public class opmode_MAIN extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    //define these up here instead of in the init section like rr, idk why but it seems to work fine.
    private Servo servo_outtake_wrist, servo_intake_wrist, servo_intake_rotate;
    private CRServo servo_intake, servo_outtake;
    private DcMotorEx up, out;
    private TouchSensor up_zero, out_zero;
    //from rr version of opmode_MAIN
    int arm_upper_lim = 4000;
    int up_true_target_pos;
    int out_true_target_pos;
    double servo_outtake_wrist_location = 0;
    double servo_intake_wrist_location = 0;
    double servo_intake_rotate_location = 0.47;


    //vars for set positions for transfer:
    /// DONE FOR NOW (do when we try full auto transfer: CHANGE THESE
    // int transfer_step = 0;
    double intake_wrist_pos_transfer = 0.25;
    double outtake_wrist_pos_transfer = 0.21;
    int out_pos_transfer = 0;
    int up_pos_transfer1 = 825;

    int out_max_pos = 1330;

    int up_specimen_hang = 1907; // Viper

    double outtake_specimen_hang = 0.45;

    double driving_multiplier_fast = 0.7;
    double driving_multiplier_slow = 0.3;

    double driving_multiplier;
    boolean pathing = false;
    // double up_pos_transfer2 = 10;
    // double up_pos_transfer3 = 20;
    // double outtake_wrist_pos_ready = 300;


    //time stuff
    double last_time = 0;
    private final ElapsedTime runtime = new ElapsedTime();

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void start() {
        //PATHING
        Pose pickupPoseBack = new Pose(15, 40, Math.toRadians(180)); // TODO: This value too!
        Pose hangPose = new Pose(36.5, 67.0, Math.toRadians(0)); // TODO


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


        //from rr version

        //setup arm to use velocity
        //setup arm variable
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //example position setup
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //example velocity setup
        //up = hardwareMap.get(DcMotorEx.class, "up");
        //up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //up.setDirection(DcMotorSimple.Direction.REVERSE);

        //setup servos for intake and outtake
        servo_intake = hardwareMap.get(CRServo.class, "intake");
        servo_outtake = hardwareMap.get(CRServo.class, "outtake");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");
        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");
        servo_intake_rotate = hardwareMap.get(Servo.class, "intakeRotate");


        //initialize touch sensor
        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
        out_zero = hardwareMap.get(TouchSensor.class, "out_zero");

        Pose startPose = new Pose(15, 40.0, Math.toRadians(0)); //TODO
        follower.setStartingPose(startPose);
    }
    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        //drive code from TeleOpEnhancements


        //TESTING PATH THING VERSION
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * driving_multiplier, -gamepad1.left_stick_x * driving_multiplier, -gamepad1.right_stick_x * 0.5);
        follower.update();

        //change drive speed for more accuracy if needed
        if (gamepad1.left_bumper) {
            driving_multiplier = driving_multiplier_slow;
        } else {
            driving_multiplier = driving_multiplier_fast;
        }


        viper_slide();
        misumi_slide();
        intake_claw();
        outtake_claw();
        macros();

        //TEST PATH THING


        //telemetry
        telemetry.addData("gamepad2.rightstickx", gamepad2.right_stick_x);
        telemetry.addData("gamepad2.rightsticky", gamepad2.right_stick_y);
        telemetry.addData("out.getCurrentpos", out.getCurrentPosition());
        telemetry.addData("servo pos", servo_outtake_wrist.getPosition());
        telemetry.addData("intake_servo", servo_intake_wrist.getPosition());
        telemetry.addData("rotate_pos", servo_intake_rotate.getPosition());
        telemetry.addData("up pos", up.getCurrentPosition());
        telemetry.addData("out_zero", out_zero.isPressed());
        telemetry.addData("gamepad1.touchpad", gamepad1.touchpad);
        telemetry.addData("gamepad1.touchpad_finger_1", gamepad1.touchpad_finger_1);
        telemetry.addData("gamepad1.touchpad_finger_2", gamepad1.touchpad_finger_2);
        telemetry.addData("gamepad1.touchpad_finger_1_x", gamepad1.touchpad_finger_1_x);
        telemetry.addData("gamepad1.touchpad_finger_1_y", gamepad1.touchpad_finger_1_y);
        telemetry.addData("gamepad1.touchpad_finger_2_x", gamepad1.touchpad_finger_2_x);
        telemetry.addData("gamepad1.touchpad_finger_2_y", gamepad1.touchpad_finger_2_y);
        telemetry.addData("gamepad1.share", gamepad1.share);
        telemetry.addData("gamepad1.share", gamepad1.guide);
        telemetry.update();
    }



    public void viper_slide() {
        if (gamepad2.right_stick_y < -0.3 && (up.getCurrentPosition() < arm_upper_lim)) { //left stick -, is going up! (I think it's inverted)
            //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up.setVelocity(gamepad2.right_stick_y * -1300); // When left stick goes up?
            telemetry.addLine("trying to go up ma'am");
            up_true_target_pos = 0;
        } else if (gamepad2.right_stick_y < -0.3 && (up.getCurrentPosition() >= arm_upper_lim)) {
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("upper limit reached", true);
        } else if (!up_zero.isPressed() && gamepad2.right_stick_y > 0.3) { //left stick +, going down
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Left stick goes down
            up.setVelocity(gamepad2.right_stick_y * -1300);
            telemetry.addLine("trying to go down ma'am");

            up_true_target_pos = 0;
        } else if (up_zero.isPressed() && gamepad2.right_stick_y > 0.3) { // Lower limit for up
            telemetry.addData("Lower Limit Reached", up_zero);
            up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            up.setPower(1);
            //use position mode to stay up, as otherwise it would fall down. do some fancy stuff with up_true_target_pos to avoid the issue of it very slightly falling every tick
            if (up_true_target_pos == 0) {
                up.setTargetPosition(up.getCurrentPosition());
                up_true_target_pos = up.getCurrentPosition();
            }
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //make sure the upper and lower limits are actually at the upper and lower limits
        if (up.getCurrentPosition() < 0) {
            up.setTargetPosition(0);
        } else if (up.getCurrentPosition() > arm_upper_lim) {
            up.setTargetPosition(arm_upper_lim);
        }
    }

    public void misumi_slide() {
        // Misumi Slide
        if (gamepad2.dpad_right && !out_zero.isPressed()) { //in
            //use velocity mode to move so it doesn't we all funky with the smoothing of position mode
            out.setPower(-0.8);
            out_true_target_pos = 0;
        } else if (gamepad2.dpad_left && out.getCurrentPosition() < out_max_pos ) { //out
            out.setPower(0.8);
        } else if (gamepad2.dpad_right && out_zero.isPressed()) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("reset out", true);
        } else {
            out.setPower(0);
            out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void intake_claw() {
        // Gamepad2.right_trigger is analog, so we need a comparative statement to use it as a digital button.
        //servo intake control
        if (gamepad2.right_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //NO LONGER NEEDED: find a better solution for this limits so we can actually use them
            servo_intake.setPower(1);
        } else if (gamepad2.right_bumper) { //NO LONGER NEEDED: these limits too.
            servo_intake.setPower(-1);
        } else {
            servo_intake.setPower(0);
        }

        // manual intake rotate location
        if (gamepad2.left_stick_x > 0.1) {
            servo_intake_rotate_location += 0.015;
        }
        if (gamepad2.left_stick_x < -0.1) {
            servo_intake_rotate_location -= 0.015;
        }

        if (servo_intake_rotate_location > 1) {
            servo_intake_rotate_location = 1;
        } else if (servo_intake_rotate_location < 0) {
            servo_intake_rotate_location = 0;
        }

        servo_intake_rotate.setPosition(servo_intake_rotate_location);


        // manual intake wrist location
        if (gamepad2.dpad_down) {
            servo_intake_wrist_location += 0.05;
        }
        if (gamepad2.dpad_up) {
            servo_intake_wrist_location -= 0.05;
        }
        // limits
        if (servo_intake_wrist_location > 1) {
            servo_intake_wrist_location = 1;
        } else if (servo_intake_wrist_location < 0) {
            servo_intake_wrist_location = 0;
        }

        servo_intake_wrist.setPosition(servo_intake_wrist_location);
    }
    public void outtake_claw() {
        //Continuous servo outtake control
        if (gamepad2.left_trigger > 0.8/* && servo_CLAW_position < 1000000000*/) { //NO LONGER NEEDED: find a better solution for this limits so we can actually use them
            servo_outtake.setPower(1);
        } else if (gamepad2.left_bumper) { //NO LONGER NEEDED: these limits too.
            servo_outtake.setPower(-1);
        } else {
            servo_outtake.setPower(0);
        }

        // manual outtake wrist location
        if (gamepad2.y) {
            servo_outtake_wrist_location += 0.03;
        }
        if (gamepad2.a) {
            servo_outtake_wrist_location -= 0.03;
        }

        //reset outtake wrist location in case value is above or below 1 or 0
        if (servo_outtake_wrist_location > 1) {
            servo_outtake_wrist_location = 1;
        } else if (servo_outtake_wrist_location < 0) {
            servo_outtake_wrist_location = 0;
        }

        servo_outtake_wrist.setPosition(servo_outtake_wrist_location);
    }
    public void macros() {
        //Encoder Transfer Method
        if (gamepad2.touchpad_finger_1 && !gamepad2.touchpad_finger_2) { //transfer pos
            servo_outtake_wrist_location = outtake_wrist_pos_transfer;
            servo_intake_wrist_location = intake_wrist_pos_transfer;
            servo_intake_rotate_location = 0.5;
            out.setTargetPosition(out_pos_transfer);
            out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            out.setPower(1);
            up.setTargetPosition(up_pos_transfer1);
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setPower(1);
            if (out.getCurrentPosition() <= 10) {
                up.setTargetPosition(430);
            }
        }
        if (gamepad2.touchpad_finger_2) { //transfer
            servo_outtake.setPower(-1);
            servo_intake.setPower(1);
        }
        if (gamepad2.x) { //goto hanging position
            up.setTargetPosition(up_specimen_hang);
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setPower(1);
            servo_outtake_wrist_location = 0.56;
        }

        if (gamepad2.options) { //reset intake rotate
            servo_intake_rotate_location = 0.47;
            servo_intake_rotate.setPosition(servo_intake_rotate_location);
        }
        if(gamepad2.share) { //viper slide up to avoid touching!
            up.setTargetPosition(400);
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setPower(1);
            servo_outtake_wrist_location = 0.50;
            servo_intake_wrist_location = 0.2;
            servo_intake_rotate_location = 0.5;
        }
        if (gamepad2.circle) { //reset intake wrist and rotate
            servo_intake_wrist_location = 0.7;
            servo_intake_rotate_location = 0.47;
        }
    }
}
