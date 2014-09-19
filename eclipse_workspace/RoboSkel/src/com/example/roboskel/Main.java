package com.example.roboskel;

import android.os.Bundle;
import android.app.ActionBar;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;	
import android.content.Intent;
import android.view.Menu;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Toast;

public class Main extends Activity 
{
	/* Dialog to display all controls */
	private AlertDialog alert;
	
	@Override
	protected void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		/* Make map full screen */
		getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
		WindowManager.LayoutParams.FLAG_FULLSCREEN);
		setContentView(R.layout.activity_main);
		if(ActiveConnection.isConnected())
			Toast.makeText(getBaseContext(), "Connection Established", Toast.LENGTH_SHORT).show();
		else
			Toast.makeText(getBaseContext(), "Connection Failed", Toast.LENGTH_SHORT).show();
		
		final ActionBar actionBar = getActionBar();
		actionBar.hide();
		Button map=(Button)findViewById(R.id.gotomap);
		Button exit=(Button)findViewById(R.id.exit);
		exit.setOnClickListener(new View.OnClickListener(){
			public void onClick(View v) 
			{
				exitDialog();
			}
		});
		Button control=(Button)findViewById(R.id.controllerSelection);
		control.setOnClickListener(new View.OnClickListener(){
			public void onClick(View v) 
			{
				selectControlType();
			}
		});
		map.setOnClickListener(new View.OnClickListener(){
			public void onClick(View v) 
			{
				Intent a=new Intent(getApplicationContext(),DisplayMap.class);
				startActivity(a);
			}
		});
	}
	
	private void selectControlType()
	{	
		final CharSequence[] items = {"Manual Control", "Simple Sensor Control", "Joystick Control","Sensor Control "};
		 
	   	AlertDialog.Builder builder = new AlertDialog.Builder(this);
	   	builder.setTitle("Select a control type");
	   	builder.setSingleChoiceItems(items, -1, new DialogInterface.OnClickListener() {
	   	    public void onClick(DialogInterface dialog, int item) 
	   	    {
	   	    	Intent a;
	   	    	switch (item)
	   	    	{
	   	    		case 0:
	   	    			a=new Intent(getApplicationContext(),ManualControl.class);
	   					startActivity(a);
	   					break;
	   	    		case 1:
	   	    			a=new Intent(getApplicationContext(),SimpleSensor.class);
	   					startActivity(a);
	   					break;
	   	    		case 2:
	   	    			a=new Intent(getApplicationContext(),CircleControl.class);
	   					startActivity(a);
	   					break;
	   	    		case 3:
	   	    			a=new Intent(getApplicationContext(),SensorControl.class);
	   					startActivity(a);
	   					break;
	   	    	}
	   	    	alert.cancel();
	   	    }
	   	});
	   	alert = builder.create();
	   	alert.show();
	}

	private void exitDialog()
	{

		 AlertDialog.Builder alert_box=new AlertDialog.Builder(this);
		 alert_box.setTitle("	Warning");
		 alert_box.setIcon(R.drawable.warning);
		 alert_box.setMessage("Do you really want to exit?");
		 alert_box.setPositiveButton("Yes",new DialogInterface.OnClickListener() {
		 @Override
		 public void onClick(DialogInterface dialog, int which) 
		 {
			 /* Exit Application */
			 finish();}
		 });
		 alert_box.setNegativeButton("No", new DialogInterface.OnClickListener() {
		 @Override
		 public void onClick(DialogInterface dialog, int which) 
		 {}
		 });
		 alert_box.show();
	}
	
	@Override
	public void onBackPressed() 
	{
		exitDialog();
	}
	
	@Override
	protected void onDestroy()
	{
		super.onDestroy();
		ActiveConnection.CloseConnection();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		return true;
	}

}
