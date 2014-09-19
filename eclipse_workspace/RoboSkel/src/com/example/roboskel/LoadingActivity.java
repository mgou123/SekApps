package com.example.roboskel;

import java.util.Set;

import android.os.Bundle;
import android.app.Activity;
import android.util.Log;
import android.view.WindowManager;
import android.widget.Toast;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;  
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;

public class LoadingActivity extends Activity 
{
	private BluetoothAdapter mBluetoothAdapter;
	boolean connect;
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
		WindowManager.LayoutParams.FLAG_FULLSCREEN);
		setContentView(R.layout.activity_loading);
		Toast.makeText(getBaseContext(), "Loading . . .", Toast.LENGTH_SHORT).show();
		new Thread()
		{
			@Override
		    public void run() 
		    {
				ActiveConnection c=new ActiveConnection();
        		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        		if (!mBluetoothAdapter.isEnabled()) 
        			c.connect("192.168.2.23");
        		else
        		{
        			BluetoothDevice device=findBluetoothDevice();
        			if(device!=null)
        				c.connect(device);
        		
	        		Intent a=new Intent(getApplicationContext(),Main.class);
	        		a.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TASK);
	        		startActivity(a);
        		}
        		/* finish() is called to prevent user from going back to the loading screen */
        		finish();
        	}
		}.start();
	}
	
	/** Scan for visible Bluetooth devices 
	 * @return sec Bluetooth device or Null if it is not found*/
	private BluetoothDevice findBluetoothDevice()
	{
		/*The bluetooth device of robot(null if it could not be found)*/
		BluetoothDevice device=null;
		IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
		registerReceiver(mReceiver, filter);
		
		Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
		/* If there are paired devices */
		if (pairedDevices.size() > 0) {
		    /* Loop through paired devices */
		    for (BluetoothDevice dev : pairedDevices) 
		    {
				/* Add the name and address to an array adapter to show in a ListView */
		        Log.d(dev.getName() + " - " + dev.getAddress(),"A");
		        /* if sec is found return it */
		        if(dev.getAddress().compareTo("00:19:86:00:00:BB")==0)
		        {
		        	device=dev;
		        	break;
		        }
		    }
		}
		return device;
	}
	
	private BroadcastReceiver mReceiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) 
		{
			     String action = intent.getAction();
			     /* When discovery finds a device */
			     if (BluetoothDevice.ACTION_FOUND.equals(action)) 
			     {
			        /* Get the BluetoothDevice object from the Intent */
			        BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
			        /* Add the name and address to an array adapter to show in a ListView */
			        Log.d(device.getName() + "\n" + device.getAddress(),"O");
			      }
		}
	};
	
	@Override
	protected void onDestroy()
	{
		super.onDestroy();
		if(mBluetoothAdapter.isEnabled())
			unregisterReceiver(mReceiver);
	}
}