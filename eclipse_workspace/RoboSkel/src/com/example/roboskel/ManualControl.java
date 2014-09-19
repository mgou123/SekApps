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
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.ImageButton;

public class ManualControl extends FragmentActivity implements OnTouchListener,
		ActionBar.OnNavigationListener {

	private static final String STATE_SELECTED_NAVIGATION_ITEM = "selected_navigation_item";
	/* variables to determine which button is pressed */
	private boolean b1,b2,b3,b4;
	/* portrait/landscape */
	int orientation;
	final int portrait=0;
	final int landscape=1;
	/* variables used to stream */
	private static final boolean DEBUG=true;
    private static final String TAG = "MJPEG";
	private int width = 320;
    private int height = 240;
    private MjpegView mv = null;
    private String URL;
    private final Handler handler = new Handler();	
    /* ******************************/
	@Override
	protected void onCreate(Bundle savedInstanceState) 
	{
		super.onCreate(savedInstanceState);
		ActiveConnection.getConn().stream(false);
		setContentView(R.layout.activity_manual_control);
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
		/*initialize buttons not pressed*/
		b1=b2=b3=b4=false;
		orientation=getResources().getConfiguration().orientation;
		Listener();
		
		ActionBar actionBar = getActionBar();
		actionBar.setDisplayShowTitleEnabled(false);
		actionBar.setIcon(R.drawable.up);
	}
	
	private void Listener()
	{
		ImageButton up,back,right,left;
		if(orientation==portrait)
		{
			up=(ImageButton)findViewById(R.id.up);
			back=(ImageButton)findViewById(R.id.back);
			right=(ImageButton)findViewById(R.id.right);
			left=(ImageButton)findViewById(R.id.left);
		}
		else
		{
			left=(ImageButton)findViewById(R.id.up);
			up=(ImageButton)findViewById(R.id.right);
			right=(ImageButton)findViewById(R.id.back);
			back=(ImageButton)findViewById(R.id.left);
		}
		/*Send to the robot the operation code 1 which is the code for 
		 *the manualControl mode */
    	ActiveConnection.getConn().setState(1);
		
		up.setOnTouchListener(ManualControl.this);
		left.setOnTouchListener(ManualControl.this);
		back.setOnTouchListener(ManualControl.this);
		right.setOnTouchListener(ManualControl.this);
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
	public boolean onTouch(View v,MotionEvent event) 
	{
		if(event.getAction() == MotionEvent.ACTION_DOWN)
		{	
			/*Send signal depending on the button pressed*/
			switch (v.getId()) 
			{
			     case R.id.up:
			    	 b1=true;
			    	 ActiveConnection.getConn().send(1);
			         break;
			     case R.id.left:
			    	 b2=true;
			    	 ActiveConnection.getConn().send(2);
			    	 break;
			     case R.id.back:
			    	 b3=true;
			    	 ActiveConnection.getConn().send(3);
			    	 break;
			     case R.id.right:
			    	 b4=true;
			    	 ActiveConnection.getConn().send(4);
			    	 break;
			}
		}else if(event.getAction() == MotionEvent.ACTION_UP)
		{
			b1=b2=b3=b4=false;
	        ActiveConnection.getConn().send(5);
		}
		
		if (b1)
	        ActiveConnection.getConn().send(1);
		else if(b2)
	        ActiveConnection.getConn().send(2);
		else if(b3)
	        ActiveConnection.getConn().send(3);
		else if(b4)
	        ActiveConnection.getConn().send(4);
		
		return true;
	}
	
	@Override
	public void onBackPressed() 
	{
        ActiveConnection.getConn().send(0);
		finish();
	}
	
	 @Override
	 public boolean onOptionsItemSelected(MenuItem item) {
		 //super.onOptionsItemSelected(item);
		    	
		 switch(item.getItemId())
		 {
			 case R.id.camera:
					/* if the view is not visible */
					if(!mv.isShown())
					{
						/* Initiate stream */
				    	if(!ActiveConnection.getConn().isStreaming())
				    		ActiveConnection.getConn().stream(false);
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
	 
	@Override
	public void onWindowFocusChanged(boolean hasFocus) {
	    super.onWindowFocusChanged(hasFocus);
	}

	@Override
	   public void onPause() {
	    	if(DEBUG) Log.d(TAG,"onPause()");
	        super.onPause();
	        if(mv!=null){
	        	if(mv.isStreaming()){
			        mv.stopPlayback();
	        	}
	        }
	    }
	
	@Override
	protected void onStop() 
	{
		super.onStop();
	}
	   @Override
	   public void onDestroy() {
	    	if(DEBUG) Log.d(TAG,"onDestroy()");
	    	
	    	if(mv!=null){
	    		mv.freeCameraMemory();
	    	}
	        super.onDestroy();
	        finish();
	    }
	

	@Override
	public void onRestoreInstanceState(Bundle savedInstanceState) {
		/* Restore the previously serialized current dropdown position.*/
		if (savedInstanceState.containsKey(STATE_SELECTED_NAVIGATION_ITEM)) {
			getActionBar().setSelectedNavigationItem(
					savedInstanceState.getInt(STATE_SELECTED_NAVIGATION_ITEM));
		}
	}

	@Override
	public void onSaveInstanceState(Bundle outState) {
		/* Serialize the current dropdown position.*/
		outState.putInt(STATE_SELECTED_NAVIGATION_ITEM, getActionBar()
				.getSelectedNavigationIndex());
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		/* Inflate the menu; this adds items to the action bar if it is present.*/
		getMenuInflater().inflate(R.menu.manual_control, menu);
		return true;
	}

	@Override
	public boolean onNavigationItemSelected(int position, long id) {
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
        	ManualControl.this.finish();
            return null;
        }

        protected void onPostExecute(Void v) {
        	startActivity((new Intent(ManualControl.this,ManualControl.class)));
        }
    }

}
