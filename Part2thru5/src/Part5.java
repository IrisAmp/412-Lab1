/*
 * January 21, 2015
 * Author Christoph Sydora, Brandon Yue
 *
 *  Program for Braitenberg Robot. Uses light sensors to make the robot atract to the light aggressivly or lovingly
 * or move away from light cowardly or explore for other light sources.
 */
 
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.EncoderMotor;
import lejos.robotics.LightDetector;
import lejos.robotics.LightScanner;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;


public class Part5 
{
	public static void main(String[] args)
	{
		Braitenberg robot = new Braitenberg();
		robot.coward();
		robot.aggressive();
		robot.love();
		robot.explore();
	}
	
	/**
	 * 
	 * class for braitenberg vehicle
	 *
	 */
	public static class Braitenberg
	{
		private EncoderMotor motorL;
		private EncoderMotor motorR;
		private SampleProvider lightSensorL;
		private SampleProvider lightSensorR;
		private float[] sampleL;
		private float[] sampleR;
		
		/**
		 * class constructor
		 */
		public Braitenberg()
		{
			motorL = new NXTMotor(MotorPort.A);
			motorR = new NXTMotor(MotorPort.B);
			
			// Init the light sensor.
			Port p1 = LocalEV3.get().getPort("S1");
			SensorModes modes1 = new EV3ColorSensor(p1);
			SampleProvider rawProvider1 = modes1.getMode("Ambient");
			lightSensorL = new MeanFilter(rawProvider1, 5);
			sampleL = new float[lightSensorL.sampleSize()];
			
			Port p2 = LocalEV3.get().getPort("S2");
			SensorModes modes2 = new EV3ColorSensor(p2);
			SampleProvider rawProvider2 = modes2.getMode("Ambient");
			lightSensorR = new MeanFilter(rawProvider2, 5);
			sampleR = new float[lightSensorL.sampleSize()];
		}
		/**
		 * move away from the detected light
		 */
		public void coward()
		{
			System.out.println("Coward.");
			
			motorL.forward();
			motorR.forward();
			
			while (true)
			{
				getSample();
				
				motorL.setPower((int) (sampleL[0] * 100));
				motorR.setPower((int) (sampleR[0] * 100));
				
				if (Button.getButtons() != 0)
				{
					motorL.stop();
					motorR.stop();
					Delay.msDelay(1000);
					break;
				}
			}
		}
		
		/**
		 * move towards light faster as light intensity increases
		 */
		public void aggressive()
		{
			System.out.println("Aggressive.");
			
			motorL.forward();
			motorR.forward();
			
			while (true)
			{
				getSample();
				
				motorL.setPower((int) (sampleR[0] * 100));
				motorR.setPower((int) (sampleL[0] * 100));
				
				if (Button.getButtons() != 0)
				{
					motorL.stop();
					motorR.stop();
					Delay.msDelay(1000);
					break;
				}
			}
		}
		
		/**
		 * function for moving towards light slower as the light intensity increases
		 */
		public void love()
		{
			System.out.println("Love.");
			
			motorL.forward();
			motorR.forward();
			
			while (true)
			{
				getSample();
				
				int rawL = (int) (sampleL[0] * 100);
				int rawR = (int) (sampleR[0] * 100);
				
				if (rawL > 50) rawL = 100 - rawL;
				if (rawL > 50) rawR = 100 - rawR;

				motorL.setPower(rawR);
				motorR.setPower(rawL);
				
				if (Button.getButtons() != 0)
				{
					motorL.stop();
					motorR.stop();
					Delay.msDelay(1000);
					break;
				}
			}
		}
		
		/**
		 * move aggressively towards light until a certain threshold then turn away
		 */
		public void explore()
		{
			System.out.println("Explore.");
			
			motorL.forward();
			motorR.forward();
			
			while (true)
			{
				getSample();
				
				int rawL = (int) (sampleL[0] * 100);
				int rawR = (int) (sampleR[0] * 100);
				
				if (rawL > 50 || rawR > 50)
				{
					turnAway();
				}
				else
				{
					motorL.setPower(rawR);
					motorR.setPower(rawL);
				}
				
				
				if (Button.getButtons() != 0)
				{
					motorL.stop();
					motorR.stop();
					Delay.msDelay(1000);
					break;
				}
			}
		}
		
		/**
		 * turns robot in another direction
		 */
		private void turnAway()
		{
			motorL.setPower(75);
			motorR.setPower(25);
			
			Delay.msDelay(3000);
		}

		/**
		 * gets a sample from the light sensors
		 */
		private void getSample()
		{
			lightSensorL.fetchSample(sampleL, 0);
			lightSensorR.fetchSample(sampleR, 0);
		}
	}
}
