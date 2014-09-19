package com.example.roboskel;


import java.io.IOException;
import java.net.URI;

import org.apache.http.HttpResponse;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;

import com.camera.simplemjpeg.MjpegInputStream;
import com.camera.simplemjpeg.MjpegView;

import android.annotation.TargetApi;
import android.app.ActionBar;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.support.v4.app.FragmentActivity;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

public class CircleControl extends FragmentActivity implements OnTouchListener,
		ActionBar.OnNavigationListener 
{
	
	/*Matrix that holds canonical values in range [-1,1] of current position using as reference 
	point the one set by the user*/
	private float[] canonicalAz;
	/*Variable used to determine if robot is in the JoySticks range*/
	private boolean inCircle,wasInCircle;
	private TextView xy,center;
	/*Joystick attributes*/
	int centerX,centerY,radious;
	private Button horn;
	float x,y,spin;
	/* variables used to stream */
	private static final boolean DEBUG=true;
    private static final String TAG = "MJPEG";
	private int width = 320;
    private int height = 240;
    private MjpegView mv = null;
    private String URL;
    private final Handler handler = new Handler();	
    /* ******************************/
	private static final String STATE_SELECTED_NAVIGATION_ITEM = "selected_navigation_item";

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
	    WindowManager.LayoutParams.FLAG_FULLSCREEN);
		setContentView(R.layout.activity_circle_control);
		/* if sec is not streaming initiate streaming */
		if(!ActiveConnection.getConn().isStreaming())
			ActiveConnection.getConn().stream(false);
		/* get view for streaming */
		mv = (MjpegView) findViewById(R.id.mv);
		if(mv != null){
        	mv.setResolution(width, height);
        	URL=mv.getUrl(getSharedPreferences("SAVED_VALUES", MODE_PRIVATE));
        }
		
		mv.setVisibility(View.INVISIBLE);
		
		/*Initializing variables*/
		canonicalAz=new float[3];
		ActiveConnection.getConn().setPower(true);
		ActiveConnection.getConn().setOnPause(false);
		wasInCircle=false;
		
		DisplayMetrics p=new DisplayMetrics();
		this.getWindowManager().getDefaultDisplay().getMetrics(p);
		centerX = p.widthPixels/2;
		centerY = p.heightPixels/2;
		/*radious=(int)((float)(p.widthPixels/3))*/
		radious=230;
		
		spin=(float)Math.cos(Math.PI/9.0f);
		
		/*Parameter to keep screen from locking*/
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		/*Send operation code 2 for sensor control*/
		ActiveConnection.getConn().setState(2);
		/*Define movement starting point ( [-1,-x],[x,1] )*/
		ActiveConnection.getConn().setSensitivity(0.0f, 0.0f);
		ImageView img=(ImageView)findViewById(R.id.circleImg);
		img.setOnTouchListener(CircleControl.this);
		
		horn=(Button)findViewById(R.id.horn);
		horn.setOnClickListener(new OnClickListener()
		{public void onClick(View v){ActiveConnection.getConn().horn();}});
		
		center=(TextView)findViewById(R.id.dimensions);
		
		center.setText("Center| x : "+p.widthPixels/2 +" y : "+p.heightPixels/2);
		xy=(TextView)findViewById(R.id.coordinates);
		xy.setText("Front/Back : 0 Left/Right : 0");
		
		/* Set up the action bar */
		final ActionBar actionBar = getActionBar();
		actionBar.setDisplayShowTitleEnabled(false);
		actionBar.setIcon(R.drawable.circle);
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
	public boolean onTouch(View v,MotionEvent e)
	{
		x=e.getRawX();
		y=e.getRawY();
		
		inCircle=(Math.pow(centerX-x,2)+Math.pow(centerY-y, 2)<=radious*radious);
		center.setText(" x : = "+(x-centerX)+" y : = "+(y-centerY));
		canonicalAz[2]=-Math.round(((float)(y-centerY)/radious)*100.0f)/100.0f;
		canonicalAz[1]=-Math.round(((float)(x-centerX)/radious)*100.0f)/100.0f;
		
		if(Math.abs(canonicalAz[2])>1.0f)
			canonicalAz[2]=1;
		if(Math.abs(canonicalAz[1])>1.0f)
			canonicalAz[1]=1;
		
		switch(e.getAction())
		{
			case(MotionEvent.ACTION_DOWN):
				if(inCircle)
				{
					/* if users touches the arc defined below the robot rotates*/
					if(Math.abs((x-centerX)/(y-centerY))/Math.PI>=spin)
					{
						canonicalAz[2]=0.0f;
						ActiveConnection.getConn().send(canonicalAz);
					}
					else
						ActiveConnection.getConn().send(canonicalAz);
					
					wasInCircle=true;
					xy.setText("Front/Back : "+canonicalAz[2]+" Left/Right : "+canonicalAz[1]);
				}
				break;
			case(MotionEvent.ACTION_MOVE):
				if(wasInCircle)
				{
					if(Math.abs((x-centerX)/(y-centerY))/Math.PI>=spin)
					{
						canonicalAz[2]=0.0f;
						ActiveConnection.getConn().send(canonicalAz);
					}
					else
						ActiveConnection.getConn().send(canonicalAz);
					xy.setText("Front/Back : "+canonicalAz[2]+" Left/Right : "+canonicalAz[1]);
				}
				break;
			case(MotionEvent.ACTION_UP):
				ActiveConnection.getConn().stop();
				wasInCircle=false;
				xy.setText("Front/Back : 0 Left/Right : 0");
				break;
			case(MotionEvent.ACTION_POINTER_DOWN):
				ActiveConnection.getConn().stop();
		}
		return true;
	}
	
	@Override
	protected void onPause() 
	{
		super.onPause();
		
		ActiveConnection.getConn().setPower(false);
		ActiveConnection.getConn().pause();
		if(DEBUG) Log.d(TAG,"onPause()");
        super.onPause();
        if(mv!=null){
        	if(mv.isStreaming()){
		        mv.stopPlayback();
        	}
        }
	}

	@Override
	public void onRestoreInstanceState(Bundle savedInstanceState) {
		/* Restore the previously serialized current dropdown position. */
		if (savedInstanceState.containsKey(STATE_SELECTED_NAVIGATION_ITEM)) {
			getActionBar().setSelectedNavigationItem(
					savedInstanceState.getInt(STATE_SELECTED_NAVIGATION_ITEM));
		}
	}

	@Override
	public void onSaveInstanceState(Bundle outState) {
		/* Serialize the current dropdown position. */
		outState.putInt(STATE_SELECTED_NAVIGATION_ITEM, getActionBar()
				.getSelectedNavigationIndex());
	}
	
	@Override
	public boolean onNavigationItemSelected(int position, long id) {
		return true;
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		/* Inflate the menu; this adds items to the action bar if it is present.*/
		getMenuInflater().inflate(R.menu.circle_control, menu);
		return true;
	}

	@Override
	public void onDestroy() 
	{
	    if(DEBUG) Log.d(TAG,"onDestroy()");
	    	
	    if(mv!=null)
	    	mv.freeCameraMemory();
	    super.onDestroy();
	    finish();
	}

	@Override
	 public boolean onOptionsItemSelected(MenuItem item) 
	{
		 /*super.onOptionsItemSelected(item); */
		    	
		 switch(item.getItemId())
		 {
		    case R.id.record:
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
			case R.id.camera:
				/* if the view is not visible */
				if(!mv.isShown())
				{
		     		new DoRead().execute(URL);
		     		mv.setVisibility(View.VISIBLE);
		     	}
			    else
			    {
			    	mv.stopPlayback();
			    	mv.setVisibility(View.INVISIBLE);
			    }
				break;
		 }
		 
		 return true;
	}

	public void setImageError(){
    	handler.post(new Runnable() {
    		@Override
    		public void run() {
    			Log.e("Image Error","");
    			return;
    		}
    	});
    }
    private class DoRead extends AsyncTask<String, Void, MjpegInputStream> {
        protected MjpegInputStream doInBackground(String... url) {
            //TODO: if camera has authentication deal with it and don't just not work
            HttpResponse res = null;         
            DefaultHttpClient httpclient = new DefaultHttpClient(); 
            HttpParams httpParams = httpclient.getParams();
            HttpConnectionParams.setConnectionTimeout(httpParams, 5*1000);
            HttpConnectionParams.setSoTimeout(httpParams, 5*1000);
            if(DEBUG) Log.d(TAG, "1. Sending http request");
            try {
                res = httpclient.execute(new HttpGet(URI.create(url[0])));
                if(DEBUG) Log.d(TAG, "2. Request finished, status = " + res.getStatusLine().getStatusCode());
                if(res.getStatusLine().getStatusCode()==401){
                    //You must turn off camera User Access Control before this will work
                    return null;
                }
                return new MjpegInputStream(res.getEntity().getContent());  
            } catch (ClientProtocolException e) {
            	if(DEBUG){
	                e.printStackTrace();
	                Log.d(TAG, "Request failed-ClientProtocolException", e);
            	}
                //Error connecting to camera
            } catch (IOException e) {
            	if(DEBUG){
	                e.printStackTrace();
	                Log.d(TAG, "Request failed-IOException", e);
            	}
                //Error connecting to camera
            }
            return null;
        }

        protected void onPostExecute(MjpegInputStream result) {
            mv.setSource(result);
            if(result!=null){
            	result.setSkip(1);
            	setTitle(R.string.app_name);
            }else{
            	Log.e("Disconnected","");
            	//setTitle(R.string.title_disconnected);
            }
            mv.setDisplayMode(MjpegView.SIZE_BEST_FIT);
            mv.showFps(false);
        }
    }
    
    public class RestartApp extends AsyncTask<Void, Void, Void> {
        protected Void doInBackground(Void... v) {
        	CircleControl.this.finish();
            return null;
        }

        protected void onPostExecute(Void v) {
        	startActivity((new Intent(CircleControl.this,CircleControl.class)));
        }
    }

}
