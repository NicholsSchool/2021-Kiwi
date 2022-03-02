package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp( name = "Kiwi: TeleOp", group = "Kiwi" )
public class KiwiTele extends OpMode 
{
    KiwiMap robot = new KiwiMap();

    RevBlinkinLedDriver.BlinkinPattern pattern;

    double speed, spinSpeed, angle;
    float lastHeading;

    @Override
    public void init() 
    {    
        robot.init( hardwareMap );
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() 
    {    
        /*
         *   SPEED
         */
        
        // Calculating speed, r, magnitude of vector... r = sqrt( x^2 + y^2 ) ...
        speed = Math.sqrt( Math.pow( gamepad1.left_stick_x, 2 ) + Math.pow( gamepad1.left_stick_y, 2 ) );
        
        // Displaying speed...
        telemetry.addData( "speed", "%1.2f", speed );
        
        
        /*
         *   SPIN SPEED
         */
        
        // Calculating spinSpeed...
        spinSpeed = 0 + gamepad1.right_trigger - gamepad1.left_trigger;
        
        // Displaying spinSpeed...
        telemetry.addData( "spinSpeed", "%1.2f", spinSpeed );
        
        
        /* 
         *   ANGLE
         */
        
        // Calculating angle, Theta, direction of vector... Theta = angle = arctan( y / x ) ...
        angle = ( 180 / Math.PI ) * Math.atan( -gamepad1.left_stick_y / gamepad1.left_stick_x );
        
        // Accounting for arctan's silly Range...
        if( gamepad1.left_stick_x < 0 )
            angle += 180;
        if( angle < 0 )
            angle += 360;
        
        // Displaying angle...
        telemetry.addData( "angle", "%3.2f", angle );
        
        
        /*
         *   ROBOT
         */
         
        telemetry.addData( "Pattern", pattern );
         
        // Blinkin Patterning
        if( gamepad1.a )
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.fromNumber( 1 + new java.util.Random().nextInt( 100 ) ) );
        if( gamepad1.dpad_down )
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.BLACK );
        
        // Movement. Moving, Spinning, and Saucing.
        if( speed > 0 ) 
        {
            if( spinSpeed != 0 )
                robot.sauce( speed, spinSpeed, angle, robot.getHeading() - lastHeading );
            else 
            {
                lastHeading = robot.getHeading();
                robot.move( speed, angle );
            } 
        }
        else if( spinSpeed != 0 ) 
        {
            if( speed > 0 )
                robot.sauce( speed, spinSpeed, angle, robot.getHeading() - lastHeading );
            else 
            {
                lastHeading = robot.getHeading();
                robot.spin( spinSpeed );
            }
        } 
        else
            stop();
        
        // Front-shifting and s'more Blinkin Patterning
        if( gamepad1.x ) 
        {
            robot.setFront( 'x' );
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        if( gamepad1.y ) 
        {
            robot.setFront( 'y' );
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        if( gamepad1.b ) 
        {
            robot.setFront( 'b' );
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        
        
        // Robot 'Anchoring'
        if( gamepad1.left_stick_button )
            robot.anchorless = !robot.anchorless;
        
        telemetry.addData( "ANCHORED", !robot.anchorless );
    }

    @Override
    public void stop() { robot.stop(); }
}
