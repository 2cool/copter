package cc.dewdrop.ffplayer;
import android.annotation.TargetApi;
import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.os.Build;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

/**
 * Created by igor on 8/26/2016.
 */


public class MapEdit extends Activity {


    static public boolean active=false;
    private EditText e_Prog, eTimer,eSpeed,eVSpeed,eAltitude,eCamAng,eZoom;
    static private SeekBar sTimer, sSpeed, sVSpeed,sAltitude,sCamAng, sProg,sZoom;
    static int inListener=0;


    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            hideSystemUI();
        }
    }
    @TargetApi(Build.VERSION_CODES.HONEYCOMB)

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
                        //  | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        // Hide the nav bar and status bar
                        | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_FULLSCREEN);
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    static public  String getNum(double num, int n){
        n++;
        if (Math.abs(num)<0.1)
            num=0;
        String s=Double.toString(num);
        int dotI=s.indexOf('.');
        if (s.length()-dotI > n){
            return s.substring(0,dotI+n);
        }else
            return s;
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    protected void update(int index) {
        GeoDot d = Programmer.get_dot(index);
        Log.d("UPADD", " " + DrawMap.selectedDot);
        // d=Programmer.get_current_dot_();
        if (d != null) {
            //  eCamAng.setText(Integer.toString((int)d.cam_ang));
            //90 -  -10
            sCamAng.setProgress((int) d.cam_ang + 10);
            eCamAng.setText(getNum(d.cam_ang, 2));
            e_Prog.setText(Programmer.prog_names[d.action_]);
            sProg.setProgress(d.action_);
            eTimer.setText(getNum(d.timer_, 2));
            sTimer.setProgress((int) d.timer_);
            eSpeed.setText(""+d.speed);
            sSpeed.setProgress((int) d.speed);
          //  //double zsk = ((d.speedZ > 0) ? (d.speedZ / Programmer.maxUpSpeed) : (d.speedZ / Programmer.maxDownSpeed));
            eVSpeed.setText(getNum(d.speedZ, 2));
            sVSpeed.setProgress((int) d.speedZ);
            eAltitude.setText(getNum(d.alt, 2));
            sAltitude.setProgress((int) (d.alt));
            eZoom.setText(""+d.cam_zoom);
            sZoom.setProgress(d.cam_zoom);

        }
    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    @Override
    protected void onCreate(Bundle savedInstanceState) {

        active=true;
        super.onCreate(savedInstanceState);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
        setContentView(R.layout.dot_edit);


        ((TextView)findViewById(R.id.textViewDistance)).setText(((int)Programmer.distance)+" m.");
        eTimer=findViewById(R.id.editTextTimer);
        eTimer.setEnabled(false);
        sTimer=findViewById(R.id.seekBarTimer);
        // sTimer.setEnabled(false);

        eCamAng=findViewById(R.id.editTextCamAng);
        eCamAng.setEnabled(false);
        sCamAng=findViewById(R.id.seekBarCamAng);
        sProg =findViewById(R.id.s_prog);
        e_Prog =findViewById(R.id.e_prog);
        e_Prog.setEnabled(false);
        eSpeed=findViewById(R.id.editTextSpeed);

        eSpeed.setEnabled(false);
        sSpeed=findViewById(R.id.seekBarSpeed);
        sSpeed.setProgress(100);
        eVSpeed=findViewById(R.id.editTextVerSpeed);
        eVSpeed.setEnabled(false);
        sVSpeed=findViewById(R.id.seekBarVerSpeed);
        sVSpeed.setProgress(100);
        eAltitude=findViewById(R.id.editTextAltitude);
        eAltitude.setEnabled(false);
        sAltitude=findViewById(R.id.seekBarAltitude);
        eZoom=findViewById(R.id.e_zoom);
        eZoom.setEnabled(false);
        sZoom=findViewById(R.id.s_zoom);


        update(DrawMap.selectedDot);

        //---------------------------------------------------------------------
        sZoom.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser) {
                // TODO Auto-generated method stub
                if (++inListener==1) {
                    Programmer.dot[DrawMap.selectedDot].cam_zoom=Programmer.cam_zoom=progress;
                    update(DrawMap.selectedDot);
                }
                inListener--;
            }
        });
//---------------------------------------------------------------------
        sTimer.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser) {
                // TODO Auto-generated method stub
                if (++inListener==1) {
                    if (Programmer.dot[DrawMap.selectedDot].action_!=GeoDot.PHOTO_360) {
                        Programmer.dot[DrawMap.selectedDot].timer_=progress;
                        update(DrawMap.selectedDot);
                    }
                }
                inListener--;
            }
        });
//---------------------------------------------------------------------
        sAltitude.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser) {
                // TODO Auto-generated method stub
                if (++inListener==1) {
                    Programmer.dot[DrawMap.selectedDot].alt=Programmer.altitude=progress;
                    double old_alt=Programmer.dot[DrawMap.selectedDot].alt;
                    Programmer.dot[DrawMap.selectedDot].dAlt-=old_alt-progress;
                    update(DrawMap.selectedDot);
                }
                inListener--;
            }
        });
//--------------------------------------------------------------------------------------
        sProg.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser) {
                // TODO Auto-generated method stub
                if (++inListener==1) {
                    Programmer.dot[DrawMap.selectedDot].action_=progress;
                    if (progress==GeoDot.PHOTO_360) {
                        if (Programmer.dot[DrawMap.selectedDot].timer_ < 120) {
                            Programmer.dot[DrawMap.selectedDot].timer_ = 120;
                        }
                    }else{
                        if (progress>=GeoDot.PHOTO && progress<=GeoDot.STOP_VIDEO){
                            if (Programmer.dot[DrawMap.selectedDot].timer_< 5) {
                                Programmer.dot[DrawMap.selectedDot].timer_ = 5;
                            }
                        }else{
                            Programmer.dot[DrawMap.selectedDot].timer_ = 0;
                        }
                    }
                    update(DrawMap.selectedDot);
                }
                inListener--;
            }
        });
//--------------------------------------------------------------------------------------
        sCamAng.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser) {
                // TODO Auto-generated method stub
                if (++inListener==1) {
                    Programmer.dot[DrawMap.selectedDot].cam_ang=Programmer.cam_ang=progress-10;
                    update(DrawMap.selectedDot);
                }
                inListener--;
            }
        });
//--------------------------------------------------------------------------------------
        sSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser) {
                // TODO Auto-generated method stub
                if (++inListener==1) {
                    if (progress==0)
                        progress++;
                    Programmer.dot[DrawMap.selectedDot].speed=Programmer.speed_=progress;
                    update(DrawMap.selectedDot);
                }
                inListener--;

            }
        });
//--------------------------------------------------------------------------------------
        sVSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}
            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser) {

                // TODO Auto-generated method stub
                if (++inListener==1) {
                    if (progress==0)
                        progress++;
                    Programmer.dot[DrawMap.selectedDot].speedZ=Programmer.speedZ_=progress;
                    update(DrawMap.selectedDot);
                }
                inListener--;

            }
        });
//--------------------------------------------------------------------------------------

    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    @Override
    protected void onPause() {
        super.onPause();
        active=false;
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
