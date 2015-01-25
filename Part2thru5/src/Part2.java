import lejos.hardware.Button;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;


public class Part2
{	
	public static void main(String[] args)
	{
		SimpleDiffDriveRobot robot = new SimpleDiffDriveRobot();
		
		robot.line();
		robot.rectangle();
		robot.circle();
		robot.figure_eight();
	}
	
	public static class SimpleDiffDriveRobot
	{
		private EncoderMotor motorL;
		private EncoderMotor motorR;
		
		/**
		 * Initialize the robot.
		 */
		public SimpleDiffDriveRobot()
		{
			motorL = new NXTMotor(MotorPort.A);
			motorR = new NXTMotor(MotorPort.B);
		}

		/**
		 * Drive in a straight line
		 */
		public void line()
		{
			resetTacho();
			prompt("Line");
			
			setMotors(50);
			
			motorsForward();
			
			Delay.msDelay(2000);
			
			motorsStop();
		}

		/**
		 * Drive in a rectangle (square)
		 */
		public void rectangle()
		{
			resetTacho();
			prompt("Rectangle");
			
			setMotors(50);
			
			for (int i = 0; i < 4; i++)
			{
				goStraight();
				turnRight();
			}
		}

		/**
		 * Drive in a circle.
		 */
		public void circle() 
		{
			resetTacho();
			prompt("Circle");
			
			motorL.setPower(50);
			motorR.setPower(10);
			
			motorsForward();
			
			Delay.msDelay(8000);
			
			motorsStop();
		}
		
		/**
		 * Drive in a figure eight pattern.
		 */
		public void figure_eight()
		{
			resetTacho();
			prompt("Figure eight");
			
			motorL.setPower(50);
			motorR.setPower(10);
			
			motorsForward();
			
			Delay.msDelay(8000);
			
			motorsStop();

			motorL.setPower(10);
			motorR.setPower(50);
			
			motorsForward();
			
			Delay.msDelay(8000);
			
			motorsStop();
		}
		
		/**
		 * Helper function. Wait for a button press with a message.
		 */
		private void prompt(String msg)
		{
			System.out.println(msg);
			Button.waitForAnyPress();
		}
		
		/**
		 * Helper function. Reset the tachometer on both motors.
		 */
		private void resetTacho()
		{
			motorL.resetTachoCount();
			motorR.resetTachoCount();
		}
		
		/**
		 * Helper function. Set both motors.
		 */
		private void setMotors(int power)
		{
			motorL.setPower(power);
			motorR.setPower(power);
		}
		
		/**
		 * Helper function. Forward both motors.
		 */
		private void motorsForward()
		{
			motorL.forward();
			motorR.forward();
		}
		
		/**
		 * Helper function. Stop both motors.
		 */
		private void motorsStop()
		{
			motorL.stop();
			motorR.stop();
		}

		/**
		 * Helper function. Move in a straight line.
		 */
		private void goStraight()
		{
			motorsForward();
			
			Delay.msDelay(1000);
			
			motorsStop();
		}

		/**
		 * Helper function. Turn right.
		 */
		private void turnRight()
		{
			motorL.forward();
			motorR.backward();
			
			Delay.msDelay(700);
			
			motorsStop();
		}
	}
}
