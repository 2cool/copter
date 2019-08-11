package cc.dewdrop.ffplayer;


/**
 * Created by igor on 8/19/2016.
 */




import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.ContextMenu;
import android.view.Display;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.ScaleGestureDetector;
import android.view.SubMenu;
import android.view.View;
import android.view.WindowManager;
import android.widget.EditText;
import android.widget.LinearLayout;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.lang.reflect.Method;


public class Map extends Activity {
    static public DrawMap drawmap=null;
    static public Bitmap blank;



    public static float [] screenMetrics;
    static public Programmer prog=new Programmer();
    static public boolean openMenu=false;
    @Override
    protected  void onStart() {
        super.onStart();
    }



    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            hideSystemUI();
        }
    }

    private void hideSystemUI() {
        // Enables regular immersive mode.
        // For "lean back" mode, remove SYSTEM_UI_FLAG_IMMERSIVE.
        // Or for "sticky immersive," replace it with SYSTEM_UI_FLAG_IMMERSIVE_STICKY
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_IMMERSIVE
                        // Set the content to appear under the system bars so that the
                        // content doesn't resize when the system bars hide and show.
                        | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                        | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        // Hide the nav bar and status bar
                        | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_FULLSCREEN);
    }



    float[] get_screen_size_in_pixels(){
        float [] screenXY=new float[4];
        final DisplayMetrics metrics = new DisplayMetrics();
        Display display = getWindowManager().getDefaultDisplay();
        Method mGetRawH = null, mGetRawW = null;
        try {


            display.getRealMetrics(metrics);
            screenXY[0] = metrics.widthPixels;
            screenXY[1] = metrics.heightPixels;
            screenXY[2] = metrics.xdpi;
            screenXY[3] = metrics.ydpi;


        } catch (Exception e3) {
            e3.printStackTrace();
        }
        return screenXY;
    }

    float scale=1;
    private class ScaleListener extends ScaleGestureDetector.SimpleOnScaleGestureListener {
        @Override
        public boolean onScale(ScaleGestureDetector scaleGestureDetector){
            scale*=scaleGestureDetector.getScaleFactor();
                Log.d("SCALE",Float.toString(scale));
                if (scale>=1.3) {
                    DrawMap.addZoom(1);
                    scale=1;
                }
                else
                    if (scale<=0.7) {
                        DrawMap.addZoom(-1);
                        scale=1;
                    }
            return true;
        }
    }

    @SuppressLint("WrongViewCast")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
     //   setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        blank= BitmapFactory.decodeStream(this.getResources().openRawResource(R.raw.blank));
        SharedPreferences settings = getPreferences(MODE_PRIVATE);
        screenMetrics=get_screen_size_in_pixels();
        DrawMap.mScaleGestureDetector = new ScaleGestureDetector(this, new ScaleListener());
        DrawMap.zoom=settings.getInt("zoom",3);
        DrawMap.screenP.x=settings.getInt("screenPX",0);
        DrawMap.screenP.y=settings.getInt("screenPY",0);
        DrawMap.type=settings.getInt("mapName",7);
        String s=settings.getString("program1","");
        if (s.length()>10)
           Programmer.load(s);

        drawmap = new DrawMap(Map.this);
        drawmap.setBackgroundColor(Color.WHITE);

        setContentView(drawmap);
        cont=this;
        registerForContextMenu(drawmap);
        drawmap.setLongClickable(false);

    }

    void save(){
        DrawMap.threadRun=false;
        SharedPreferences settings = getPreferences(MODE_PRIVATE);
        SharedPreferences.Editor editor = settings.edit();
        editor.putInt("zoom",DrawMap.zoom);
        editor.putInt("screenPX",DrawMap.screenP.x);
        editor.putInt("screenPY",DrawMap.screenP.y);
        editor.putInt("mapName",DrawMap.type);
        editor.putString("program1",Programmer.get());
        editor.commit();

    }

    static public Context cont;
    @Override
    protected void onPause(){
        super.onPause();
        save();
    }
    @Override
    protected void onDestroy() {
        super.onDestroy();
        save();

    }

    static String[] progName=new String[100];
    static int progs=0;
    static String[] logName=new String[1000];
    static int logs=0;
    final int PROGS_I=20;
    final int MAX_STR_IN_LIST =100;
    final int LOGS_I=1000;

    void add2menu_prog_loading(Menu menu){

        SubMenu fileMenu = menu.addSubMenu("Load prog");

        File f = new File("/sdcard/RC/PROGS");
        if (f.exists()==false) {
            File folder = new File("/sdcard/RC/PROGS");
            folder.mkdirs();
        }
        File file[] = f.listFiles();
        progs=0;
        for (int i=0; i<file.length; i++) {
            String t[]=file[i].toString().split("/");
            if (t[t.length-1].endsWith(".prog")) {
                String ts=t[t.length - 1];
                ts=ts.substring(0,ts.length()-5);
                fileMenu.add(0, progs+PROGS_I, 0, ts);
                progName[progs]=t[t.length - 1];
                progs++;
                if (progs>= MAX_STR_IN_LIST)
                    break;
            }

        }
    }

    void add2menu_log_loading(Menu menu){
        SubMenu fileMenu = menu.addSubMenu( "Load log");

        File f = new File("/sdcard/RC");
        if (f.exists()==false) {
            File folder = new File("/sdcard/RC");
            folder.mkdirs();
        }
        File file[] = f.listFiles();
        logs=0;
        for (int i=0; i<file.length; i++) {
            String t[]=file[i].toString().split("/");
            if (t[t.length-1].endsWith(".log")) {
                String ts=t[t.length - 1];
                ts=ts.substring(0,ts.length()-4);
                fileMenu.add(0, logs+1000, 0, ts);
                logName[logs]=t[t.length - 1];
                logs++;
                if (logs>= MAX_STR_IN_LIST)
                    break;
            }

        }
    }

    public void onCreateContextMenu(ContextMenu menu, View v,
                                    ContextMenu.ContextMenuInfo menuInfo) {
        super.onCreateContextMenu(menu, v, menuInfo);
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.map_menu, menu);
        add2menu_prog_loading(menu);
        add2menu_log_loading(menu);
    }

    @Override
    public boolean onContextItemSelected(MenuItem item){
        int id = item.getItemId();
        if (id>=PROGS_I && id <PROGS_I+ MAX_STR_IN_LIST){
            try {
                FileInputStream stream = new FileInputStream("/sdcard/RC/PROGS/"+progName[id-PROGS_I]);
                int len=stream.available();
                byte buf[]=new byte[len];
                try {
                    stream.read(buf);
                    Programmer.load(new String(buf));

                } finally {
                    stream.close();
                }
            }
            catch (Exception e) {
            }
        }
        if (id>=LOGS_I ){
            try {
                FileInputStream stream = new FileInputStream("/sdcard/RC/"+logName[id-LOGS_I]);
                int len=stream.available();
                byte buf[]=new byte[len];
                try {
                    int rlen=stream.read(buf);
                    LogReader.load(new String(buf));
                } finally {
                    stream.close();
                }
            }
            catch (Exception e) {
            }
        }
        switch(id) {

            case R.id.GOOGLE_MAP:{
                DrawMap.type=0;
                break;
            }
            case R.id.OPEN_CYCLE_MAP:{
                DrawMap.type=1;
                break;
            }
            case R.id.MICROSOFT_EARTH:{
                DrawMap.type=7;
                break;
            }
            case R.id.SAVE_PROG:{
                AlertDialog.Builder alertDialog = new AlertDialog.Builder(Map.this);
                alertDialog.setTitle("File name");
                final EditText input = new EditText(Map.this);
                LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                        LinearLayout.LayoutParams.MATCH_PARENT,
                        LinearLayout.LayoutParams.MATCH_PARENT);
                input.setLayoutParams(lp);
                alertDialog.setView(input);
                alertDialog.setPositiveButton("YES",
                        new DialogInterface.OnClickListener() {
                            @TargetApi(Build.VERSION_CODES.HONEYCOMB)
                            public void onClick(DialogInterface dialog, int which) {
                                String fn = input.getText().toString();
                                try {
                                    FileOutputStream stream = new FileOutputStream("/sdcard/RC/PROGS/"+fn+".prog");
                                    try {
                                        stream.write(Programmer.get().getBytes());
                                    } finally {
                                        stream.close();
                                        invalidateOptionsMenu();
                                    }
                                }
                                catch (Exception e) {

                                }
                            }
                        });
                alertDialog.setNegativeButton("NO",
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int which) {
                                dialog.cancel();
                            }
                        });
                alertDialog.show();
                break;
            }
            case R.id.EXIT:{
                super.finish();
                break;
            }
        }
        return super.onContextItemSelected(item);
    }



    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_VOLUME_DOWN) {
            //Log.i("KEY","DOWN "+Integer.toString(keyCode));
            DrawMap.addZoom(1);
            return true;
        }
        if (keyCode == KeyEvent.KEYCODE_VOLUME_UP) {
            //Log.i("KEY","DOWN "+Integer.toString(keyCode));
            DrawMap.addZoom(-1);
            return true;
        }

        if (keyCode == KeyEvent.KEYCODE_BACK)
            DrawView.turn2MainScreen();

        return super.onKeyDown(keyCode, event);
    }






}
