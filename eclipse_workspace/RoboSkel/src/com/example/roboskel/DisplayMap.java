package com.example.roboskel;

import android.annotation.TargetApi;
import android.app.ActionBar;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.content.Context;
import android.content.Intent;
import android.content.res.Configuration;
import android.os.Build;
import com.actionbarsherlock.view.MenuItem;
import java.io.File;
import java.io.IOException;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;
import com.actionbarsherlock.app.SherlockActivity;
import com.camera.simplemjpeg.MjpegInputStream;
import com.camera.simplemjpeg.MjpegView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Toast;
import java.io.FileInputStream;
import java.net.URI;
import org.apache.http.HttpResponse;
import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;

import com.example.touch.TouchImageView;


public class DisplayMap extends SherlockActivity
{
	private TouchImageView img;
	private static final boolean DEBUG=true;
    private static final String TAG = "MJPEG";

    private MjpegView mv = null;
    private String URL;
    
    private int width = 320;
    private int height = 240;
    
	final Handler handler = new Handler();	
	
	@Override
	protected void onCreate(Bundle savedInstanceState) 
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_display_map);
		/* if sec is not streaming initiate streaming */
		if(!ActiveConnection.getConn().isStreaming())
			ActiveConnection.getConn().stream(false);
		mv = (MjpegView) findViewById(R.id.mv);
		if(mv != null){
        	mv.setResolution(width, height);
        	URL=mv.getUrl(getSharedPreferences("SAVED_VALUES", MODE_PRIVATE));
        }
		mv.setVisibility(View.INVISIBLE);
		/* If map does not exist download it */
		File file = new File(ActiveConnection.getConn().getFloorMap());
		if(!file.exists())
			ActiveConnection.getConn().receiveMap();
		/* Make map full screen */
		getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
		WindowManager.LayoutParams.FLAG_FULLSCREEN);
		/*Parameter to keep screen from locking*/
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		 /*Display the map */
    	img = (TouchImageView) findViewById(R.id.img);
    	img.setImageBitmap(getOriginalMap());
		/* Set up the action bar to show a dropdown list. */
		ActionBar actionBar = getActionBar();
		actionBar.setDisplayShowTitleEnabled(false);
		actionBar.setIcon(R.drawable.mapsandroid);
	}
	
	/** Method used to retrieve map from sd card */
	private Bitmap getOriginalMap()
	{
		File f = new File(ActiveConnection.getConn().getFloorMap());
		Bitmap b;
    	BitmapFactory.Options options=new BitmapFactory.Options();
    	options.inSampleSize = 2;
    	options.inPreferredConfig=Bitmap.Config.ARGB_4444;
    	try{
    		b=BitmapFactory.decodeStream(new FileInputStream(f), null, options);
        }catch(Exception e){b=null;Log.d("Could not fetch bitmap from disk"," ");}
    	return b;
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
    public boolean onCreateOptionsMenu(com.actionbarsherlock.view.Menu menu) {
        getSupportMenuInflater().inflate(R.menu.display_map, menu);
        return super.onCreateOptionsMenu(menu);
    }
	
	@Override
	public boolean onOptionsItemSelected(MenuItem item) 
	{
		 /*super.onOptionsItemSelected(item);*/
		    	
		 switch(item.getItemId())
		 {
		    case R.id.action_settings:
		    	img.clearMap();
			    break;
			case R.id.navigation:
				if(ActiveConnection.getConn().isNavigating())
				{
					item.setIcon(R.drawable.action_map);
					Toast.makeText(getBaseContext(), "Stopped navigation", Toast.LENGTH_SHORT).show();
					ActiveConnection.getConn().setNavigating(false);
				}
				else
				{
			    	/* start view refresh*/
					img.Navigate();
					item.setIcon(R.drawable.ic_action_map_navigating);
				}
			    break;
			case R.id.save_map:
			    Toast.makeText(getBaseContext(), "Saving Map", Toast.LENGTH_SHORT).show();
			    ActiveConnection.getConn().saveMap();
			    break;
			case R.id.gamepad:
			    Toast.makeText(getBaseContext(), "You selected Gamepad", Toast.LENGTH_SHORT).show();
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
			case R.id.set_destination:
				ActiveConnection.getConn().setDest(true);
			    Toast.makeText(getBaseContext(), "Set Destination", Toast.LENGTH_SHORT).show();
			    break;
			case R.id.set_location:
				ActiveConnection.getConn().setStart(true);
			    Toast.makeText(getBaseContext(), "Set Location", Toast.LENGTH_SHORT).show();
			    break;
			case R.id.download_map:
				receiveMap();
				break;
			case R.id.begin_manuver:
				ActiveConnection.getConn().beginManeuverMap();
				break;
			case R.id.end_manouver:
				ActiveConnection.getConn().endManeuverMap();
				break;
		    }
		    return true;
	}
	
	/** Downloads and displays the map */
	private void receiveMap()
	{
    	/*Receive the Map */
		ActiveConnection.getConn().receiveMap();
		Toast.makeText(getBaseContext(), "Downloading Map", Toast.LENGTH_SHORT).show();
    	img.setImageBitmap(getOriginalMap());
	}
	
	@Override
	public void onConfigurationChanged(Configuration newConfig) {
	  super.onConfigurationChanged(newConfig);
	  setContentView(R.layout.activity_display_map);
	}
	
	@Override
	public void onPause() 
	{
	   	if(DEBUG) Log.d(TAG,"onPause()");
	    super.onPause();
	    if(mv!=null)
	    {
	       	if(mv.isStreaming())
	       		mv.stopPlayback();
	    }
	}
	
	@Override
	public void onDestroy() 
	{
	    if(DEBUG) Log.d(TAG,"onDestroy()");
	    if(mv!=null)
	    	mv.freeCameraMemory();
	    /* Cancel streaming */
	    super.onDestroy();
	}
	
	@Override
	protected void onStop() 
	{
		super.onStop();
		ActiveConnection.getConn().exitMap();
		finish();
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
        	DisplayMap.this.finish();
            return null;
        }

        protected void onPostExecute(Void v) {
        	startActivity((new Intent(DisplayMap.this,DisplayMap.class)));
        }
    }
}


