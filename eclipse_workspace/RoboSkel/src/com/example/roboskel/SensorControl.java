package com.example.roboskel;

import android.annotation.TargetApi;
import android.app.ActionBar;
import android.os.Bundle;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.support.v4.app.FragmentActivity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

public class SensorControl extends FragmentActivity implements SensorEventListener,OnClickListener ,
		ActionBar.OnNavigationListener 
{

	private SensorManager mSensorManager;
	/*Matrixes used to store values to calculate angle at each axis*/
	private float[] gravity,magnetic,azimuth,refAzimuth,currAzimuth;
	/*Matrix that holds canonical values in range [-1,1] of current position using as reference 
	point the one set by the user*/
	private float[] canonicalAz;
	/*Threshold Variable that controls the sensitivity of movement*/ 
	private final float E=4.0f;
	/*Maximum angle(the angle that canonicalizes the value sent to robot)*/
	private final float maxAngle=25.0f;
	/*Difference between last and current value*/
	private float DeltaX,DeltaY,DeltaZ;
	/*Variable used to prevent subsequent break on/off uses*/
	private boolean useBreak;
	/*Sensors*/
	private Sensor accSensor,magnetSensor;
	private boolean initialized,referenceP;
	private TextView x,y,z,acc;
	/*variables used to verify whether user shakes device or not*/
	private float mAccel; /*acceleration apart from gravity*/
	private float mAccelCurrent; /* current acceleration including gravity */
	private float mAccelLast; /*last acceleration including gravity*/
	/*
	 * 
	 */
	private static final String STATE_SELECTED_NAVIGATION_ITEM = "selected_navigation_item";

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_sensor_control);

		x=(TextView)findViewById(R.id.x_axis);
		y=(TextView)findViewById(R.id.y_axis);
		z=(TextView)findViewById(R.id.z_axis);
		acc=(TextView)findViewById(R.id.acceleration);
		//y.setVisibility(TextView.INVISIBLE);
		/*Getting sensors*/
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		accSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		magnetSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
		
		/*Initializing variables*/
		refAzimuth=new float[3];
		gravity=new float[3];
		magnetic=new float [3];
		initialized=false;
		referenceP=false;
		azimuth=new float[3];
		canonicalAz=new float[3];
		useBreak=true;
		
		/*Parameter to keep screen from locking*/
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		/*Send operation code 2 for sensor control*/
		ActiveConnection.getConn().setState(2);
		ActiveConnection.getConn().setSensitivity(0.2f, 0.25f);
		/*Initialize variables used to determine shake(emergency brake of robot)*/
		mAccel = 0.00f;
	    mAccelCurrent = SensorManager.GRAVITY_EARTH;
	    mAccelLast = SensorManager.GRAVITY_EARTH;
		
	    /*set point of reference to calculate angle*/
		Button val=(Button)findViewById(R.id.SetStart);
		val.setOnClickListener(SensorControl.this);
		
		
		/* Set up the action bar to show a dropdown list.*/
		final ActionBar actionBar = getActionBar();
		actionBar.setDisplayShowTitleEnabled(false);
		actionBar.setNavigationMode(ActionBar.NAVIGATION_MODE_LIST);
	}

	@Override
	public void onClick(View v) 
	{
		switch (v.getId()) 
		{
			/*User defines point 0,0,0*/
	        case R.id.SetStart:
	        {
	        	z.setText("0.0");
				/*pitch, rotation around the X axis*/
				x.setText("0.0");
				/*roll, rotation around the Y axis*/
				y.setText("0.0");
				referenceP=true;
				System.arraycopy(azimuth, 0,refAzimuth, 0, 3);
				ActiveConnection.getConn().setPower(true);
				canonicalAz[0]=0.0f;
				canonicalAz[1]=0.0f;
				canonicalAz[2]=0.0f;
				ActiveConnection.getConn().send(canonicalAz);
				break;
	        }
		}
	}
	
	@Override
	public void onSensorChanged(SensorEvent e)
	{
		switch(e.sensor.getType())
		{
			/*get gravity value arrays from Magnet
			 * in order to get orientation(e,w,s,n)*/
			case(Sensor.TYPE_MAGNETIC_FIELD):
			{
				System.arraycopy(e.values, 0, magnetic, 0, 3);
				break;
			}
			/*get gravity value arrays from Accelerometer
			 * define which way is up/down,identify shake event*/
			case(Sensor.TYPE_ACCELEROMETER):
			{
				System.arraycopy(e.values, 0, gravity, 0, 3);
				mAccelLast = mAccelCurrent;
				//Using only Z axis
				mAccelCurrent = (float) Math.sqrt((double) (e.values[2]*e.values[2]+e.values[1]*e.values[1]));
			    //mAccelCurrent = (float) Math.sqrt((double) (e.values[0]*e.values[0] + e.values[1]*e.values[1] + e.values[2]*e.values[2]));
			    float delta = mAccelCurrent - mAccelLast;
			    mAccel = mAccel * 0.8f + delta; // perform low-cut filter
				if(mAccel>7.0 && useBreak)
				{
					useBreak=false;
					Thread t=new Thread(){public void run(){try {
						Thread.sleep(1000);
					} catch (InterruptedException e) {} useBreak=true;}};
					t.start();
					
					if(ActiveConnection.getConn().isPower())
					{
						Toast.makeText(getBaseContext(),"Emergency brake used at : "+mAccel, Toast.LENGTH_SHORT).show();
						ActiveConnection.getConn().setPower(false);
						ActiveConnection.getConn().stop();
					}
					else
					{
						Toast.makeText(getBaseContext(),"Resuming function", Toast.LENGTH_SHORT).show();
						ActiveConnection.getConn().setPower(true);
					}
				}
				break;
			}
		}
		
		if (gravity != null && magnetic != null) 
		{
			/*Rotation matrix and Inclination matrix*/
			float R[] = new float[9];
			float I[] = new float[9];
			/* Compute the inclination matrix I as well as the rotation matrix R 
			transforming a vector from the device 
			coordinate system to the world's coordinate system 
			R and I [Length 9]
			gravity vector expressed in the device's coordinate [Length 3]
			geoMagnetic vector expressed in the device's coordinate[Length 3]
			*/
			if(SensorManager.getRotationMatrix(R, I,gravity, magnetic))
			{
				currAzimuth = new float[3];
				SensorManager.getOrientation(R, currAzimuth);
				if(!initialized)
				{
					DeltaX=0.0f;
					DeltaY=0.0f;
					DeltaZ=0.0f;
					System.arraycopy(currAzimuth, 0,azimuth, 0, 3);
					initialized=true;
				}
				else
				{
					/*Convert to degrees positive in the counter-clockwise direction
					  azimuth, rotation around the Z axis and calculate Delta*/
					DeltaZ=(float)(Math.abs(Math.toDegrees(currAzimuth[0]-azimuth[0])));
					DeltaX=(float)(Math.abs(Math.toDegrees(currAzimuth[1]-azimuth[1])));
					DeltaY=(float)(Math.abs(Math.toDegrees(currAzimuth[2]-azimuth[2])));
					
				}
				/*If the variation of the angle exceeds E(minimum variation)
				in at least one axis*/
				if((DeltaX>E) || (DeltaY>E))// || (DeltaZ>E))
				{	
					System.arraycopy(currAzimuth, 0,azimuth, 0, 3);
					if(!referenceP)
					{
						z.setText("z : "+Float.toString((float)(Math.toDegrees(azimuth[0]))));
						/*pitch, rotation around the X axis*/
						x.setText("x : "+Float.toString((float)(Math.toDegrees(azimuth[1]))));
						/*roll, rotation around the Y axis*/
						y.setText("y : "+Float.toString((float)(Math.toDegrees(azimuth[2]))));
					}
					else
					{
						canonicalAz[0]=(Math.round(((float)cannAz(refAzimuth[0],azimuth[0])/maxAngle)*100.0f))/100.0f;
						canonicalAz[1]=-(Math.round(((float)cannAz(refAzimuth[1],azimuth[1])/maxAngle)*100.0f))/100.0f;
						canonicalAz[2]=-(Math.round(((float)cannAz(refAzimuth[2],azimuth[2])/maxAngle)*100.0f))/100.0f;
						ActiveConnection.getConn().send(canonicalAz);
						
						z.setText("z : "+Float.toString(canonicalAz[0]));
						/*pitch, rotation around the X axis*/
						x.setText("x : "+Float.toString(canonicalAz[1]));
						/*roll, rotation around the Y axis*/
						y.setText("y : "+Float.toString(canonicalAz[2]));
					}
				}
			}
		}
	}
	
	
	private int cannAz(float r,float az)
	{
		/*if(r*az)>0 or |r|+|az|<=180 return az-r*/
		return (int)Math.round(Math.toDegrees(((r*az>0.0f) || (Math.abs(r)+Math.abs(az)<=(float)Math.PI) ? az-r:( (r>0.0f)&&(az<0.0f) ? ((float)2.0*Math.PI-r+az):(az-r-(float)2.0*Math.PI)))));
	}
	
	private int cann2(float ref,float azim)
	{
		int r=(int)Math.round((float)Math.toDegrees((ref)));
		int az=(int)Math.round((float)Math.toDegrees((azim)));
		
		if(r>0)
		{
			if(az>0)
				return az-r;
			else
			{
				if((r-az)<=180)
					return az-r;
				else
					return 360-r+az;
			}
		}
		else if (r<0)
		{
			if (az<0)
				return az-r;
			else
			{
				if((az-r)>=180)
					return az-r-360;
				else
					return az-r;
			}
		}
		else
			return az;
	}
	
	@Override
	protected void onResume() 
	{
		super.onResume();  
		/* Register the sensor listeners */
		mSensorManager.registerListener(this, accSensor,SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, magnetSensor,SensorManager.SENSOR_DELAY_NORMAL);
	}
	
	@Override
	protected void onPause() 
	{
		super.onPause();
		mSensorManager.unregisterListener(this);
		
		ActiveConnection.getConn().setPower(false);
		ActiveConnection.getConn().pause();
		
		finish();
	}
	
	@Override
	public void onAccuracyChanged(Sensor arg0, int arg1) {
		// TODO Auto-generated method stub
		
	}

	
	@TargetApi(Build.VERSION_CODES.ICE_CREAM_SANDWICH)
	private Context getActionBarThemedContextCompat() {
		if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.ICE_CREAM_SANDWICH) {
			return getActionBar().getThemedContext();
		} else {
			return this;
		}
	}

	@Override
	public void onRestoreInstanceState(Bundle savedInstanceState) {
		if (savedInstanceState.containsKey(STATE_SELECTED_NAVIGATION_ITEM)) {
			getActionBar().setSelectedNavigationItem(
					savedInstanceState.getInt(STATE_SELECTED_NAVIGATION_ITEM));
		}
	}

	@Override
	public void onSaveInstanceState(Bundle outState) {
		outState.putInt(STATE_SELECTED_NAVIGATION_ITEM, getActionBar()
				.getSelectedNavigationIndex());
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		getMenuInflater().inflate(R.menu.sensor_control, menu);
		return true;
	}

	@Override
	public boolean onNavigationItemSelected(int position, long id) {
		return true;
	}
	
	@Override
	 public boolean onOptionsItemSelected(MenuItem item) 
	{
		 /*super.onOptionsItemSelected(item);*/
		    	
		 switch(item.getItemId())
		 { case R.id.record:
		    	ActiveConnection.getConn().beginRecording();
			    break;
		    case R.id.stop_recording:
		    	ActiveConnection.getConn().endRecording();
		    	break;
		    case R.id.begin_manuver:
				ActiveConnection.getConn().beginManeuver();
				break;
			case R.id.end_manouver:
				ActiveConnection.getConn().endManeuver();
				break;
		 }
		 
		 return true;
	}
	 
}
