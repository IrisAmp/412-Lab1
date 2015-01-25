import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;


public class Part5 
{
	public static void main(String[] args)
	{
		Braitenberg robot = new Braitenberg();
		//robot.coward();
		//robot.aggressive();
		robot.love();
		//robot.explore();
	}
	
	public static class Braitenberg
	{
		private EncoderMotor motorL;
		private EncoderMotor motorR;
		private SampleProvider lightSensorL;
		private SampleProvider lightSensorR;
		private float[] sampleL;
		private float[] sampleR;
		
		@SuppressWarnings("resource")
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
		
		public void love()
		{
			System.out.println("Love.");
			
			float threshold = 1.5f;
			
			motorL.forward();
			motorR.forward();
			
			while (true)
			{
				getSample();
				
				double rawL = Math.min(sampleL[0] * threshold, 1.0f);
				double rawR = Math.min(sampleR[0] * threshold, 1.0f);
				
				int powerL = (int) (Math.sin(Math.PI * rawL) * 100.0);
				int powerR = (int) (Math.sin(Math.PI * rawR) * 100.0);

				motorL.setPower(powerR);
				motorR.setPower(powerL);
				
				if (Button.getButtons() != 0)
				{
					motorL.stop();
					motorR.stop();
					Delay.msDelay(1000);
					break;
				}
			}
		}
		
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
		
		private void turnAway()
		{
			motorL.setPower(75);
			motorR.setPower(25);
			
			Delay.msDelay(3000);
		}

		private void getSample()
		{
			lightSensorL.fetchSample(sampleL, 0);
			lightSensorR.fetchSample(sampleR, 0);
		}
	}
}
