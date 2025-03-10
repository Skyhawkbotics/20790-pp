package pedroPathing.opmode;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "telemetry testing ", group = "MAIN")
public class opmode_MAIN_telelmtry extends OpMode {

    private DcMotorEx up1;

    private TouchSensor up_zero;


    boolean goingdown;

    @Override
    public void start() {
        //PATHING
    }

    @Override
    public void init() {
        up1 = hardwareMap.get(DcMotorEx.class, "up1");
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");
    }

    /**
    Telemetry loop to understand gamepad inputs
     */
    @Override
    public void loop() {
        //telemetry
        telemetry.addData("gamepad2.rightstickx", gamepad2.right_stick_x);
        telemetry.addData("gamepad2.rightsticky", gamepad2.right_stick_y);
        telemetry.addData("gamepad2.left stick y", gamepad2.left_stick_y);
        telemetry.addData("gamepad2.left stick x", gamepad2.left_stick_x);
        telemetry.addData("up1 pos", up1.getCurrentPosition());
        telemetry.addData("triangle", gamepad2.triangle);
        telemetry.addData("left trigger", gamepad2.left_trigger);
        telemetry.addData("up1 pos ", up1.getCurrentPosition());
        telemetry.addData("touch" , up_zero.isPressed());


        telemetry.update();
        if (gamepad2.left_stick_y < -0.5) { // Up
            goingdown = false;
            up1.setTargetPosition(1000);
                up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up1.setPower(1);
                telemetry.addData("up1", true);
                telemetry.addData("up2", true);
        } else if (gamepad2.left_stick_y > 0.5) { // Down
                goingdown = true;
        } else if(gamepad2.left_stick_y == 0) {
            goingdown = false;
            up1.setTargetPosition(up1.getCurrentPosition());
            up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up1.setPower(1);
        }
        if(goingdown) {
            if(!up_zero.isPressed()) {
                up1.setTargetPosition(-1000);
                up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up1.setPower(-1);
            } else if(up_zero.isPressed()) {
                up1.setPower(0);
                up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                goingdown = false;
            }
        }
        if(up_zero.isPressed()) {
            telemetry.addData("touch touch touch", true);
        } else {
            telemetry.addData("no touch", true);
        }
    }
}