package cc.dewdrop.ffplayer;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import cc.dewdrop.ffplayer.utils.FFUtils;

public class Settings extends Activity implements AdapterView.OnItemSelectedListener,SeekBar.OnSeekBarChangeListener{

    String _null="";


    final int seek_bar_default_progress=50;
    final double seek_bar_default_change=0.2;
    /*
    "name,max,progress in percent,change in percent" or only "name"
     */


    double def_change[][]={
            {0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2},
            {0.2,0.2,0.2,0.2,0.2,0.2,1,0.2,0.2,0.2},
            {0.2,0.2,20,0.2,0.2,0.2,10,4,0.2,0.2},
            {0.2,0.2,0.2,0.2,0.2,0.2,1,2,1,1},
            {0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2},
            {10,10,10,10,0.2,0.2,0.2,0.2,0.2,0.2},
            {5,5,1,0.2,0.2,0.2,0.2,0.2,0.2,0.2},
            {20,20,10,1,0.2,0.2,0.2,0.2,0.2,0.2}

    };


    String a[][]={
            // balance
            {"P_R_KP","P_R_KI","P_R_KD","P_R_rateIMAX","YAW_KP","YAW_KI","YAW_KD","YAW_rate_IMAX","MAX_ANGLE",_null},
            // Z stab
            {"alt to speed", "speed_kP","speed_kI","speed_kD","MAX_SPEED_P","MAX_SPEED_M","min_stab_speed","FILTR",_null,_null},
            // XY stab
            {"pos_KP","speed_KD","max_wind_ang","dist to speed", "speed_kP","acc_KD","max_speed","allowance",_null,_null},
            // secure
            {"high_to_lift_2_home","max_throttle","min_throttle","min_hight","lift at start","battery cap","min_hor_acur",_null,_null, _null},//,"gimbP_Z","gimbR_Z"},
            // mpu
            {"Q_XY","R_XY","Q_Z","R_Z",_null,_null,_null,_null,_null,_null},
            //compas
            {"yaw_correction","calibr_index","phone_yaw_corr",_null, _null,_null,_null,_null,_null,_null},
            //rest
            {"start_sim800 need restart","ppp_inet need restart","telegram need restart",_null,_null,_null,_null,_null,_null,_null},
            //DEBUG
            {"debug 1","debug 2","debug 3","debug 4","debug 5","debug 6","debug 7","debug 8","debug 9","debug 0"}

    };

    TextView[] textV=new TextView[20];
    SeekBar []seek_bar=new SeekBar[10];
    Button set;

    protected static int menu_n=0;


    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {

        if (keyCode == KeyEvent.KEYCODE_VOLUME_UP) {
            finish();
            return true;
        }

        return super.onKeyDown(keyCode, event);
    }


    protected void download_settings(int num, boolean upload_anyway){
      //  String sn=""+num;
       // SharedPreferences settings = getPreferences(MODE_PRIVATE);
       // Log.i("UPS", "SET_EDIT_TEXT");
        if (upload_anyway || Telemetry.n_settings!=num ||  Telemetry.settings[0]==Commander.NO_DATA) {

            Commander.upload_settings = num;//Integer.parseInt(n.getText().toString());
            //   Commander.button="UP"+Integer.parseInt(n.getText().toString());
           // Log.i("UPS", "DOWNLOADING SETINGS");
            Telemetry.n_settings=-2;
            boolean uploaded=false;
            for (int i=0; i<5; i++) {
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                   // Log.i("UPS", "sleep Error");
                    e.printStackTrace();
                }
                if (Telemetry.n_settings!= -2){
                   // Log.i("UPS", "DOWNLOADING DONE");
                    if ( Telemetry.n_settings != num){//Integer.parseInt(n.getText().toString()) ){
                       // Log.i("UPS", "DOWNLOADING ERROR");
                        uploaded=false;
                    }else {
                        uploaded=true;
                        break;
                    }
                }
            }
            if (uploaded==false)
                for (int i=0; i<10; i++)
                    Telemetry.settings[i] = Commander.NO_DATA;
        }


      //  if (flag)
        //    n.setText("" + num);
    }

    @Override
    protected void onStop(){
        super.onStop();
        DrawView.turn2MainScreen();
    }

    @Override
    protected  void onStart(){
        super.onStart();
        update();

    }


    void reset(View view){
        update();
    }

    void update(){
       try {
           download_settings(menu_n,true);
         //  Log.i("UPS", "UPDATE in");
           int i = 0;
           for (; i < 10; i++) {

               String f[] = a[menu_n][i].split(",");
               textV[i + 10].setText(f[0]);
               if (Telemetry.settings[i] != Commander.NO_DATA) {
                   textV[i].setText(Float.toString(Telemetry.settings[i]));
                   textV[i + 10].setEnabled(true);
                   textV[i].setEnabled(true);
                   seek_bar[i].setEnabled(true);
                   if (f.length >= 3) {
                       double max = Float.parseFloat(f[1]);
                       double progress = max * 0.01 * Float.parseFloat(f[2]);
                       seek_bar[i].setMax((int) max);
                       seek_bar[i].setProgress((int) progress);
                   } else {
                       seek_bar[i].setMax(seek_bar_default_progress * 2);
                       seek_bar[i].setProgress(seek_bar_default_progress);
                   }
               } else {

                   textV[i].setText("");
                   textV[i + 10].setEnabled(false);
                   textV[i].setEnabled(false);
                   seek_bar[i].setEnabled(false);
               }
           }

//------------------------------------------


           set.setEnabled(Telemetry.n_settings>=0);
          // Log.i("UPS", "UPDATE out");
       } catch (Exception ex) {
           Thread.currentThread().interrupt();
          // Log.i("UPS", "UPDATE ERROR  EXCEPTION "+ex.toString());
       }
    }



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        try {




            super.onCreate(savedInstanceState);

            setContentView(R.layout.settings_scrolling);


            set = findViewById(R.id.upload);

            Spinner spinner = findViewById(R.id.spinner1);
            ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,R.array.settings_list, android.R.layout.simple_spinner_item);
            adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            spinner.setAdapter(adapter);
            spinner.setOnItemSelectedListener(this);
            spinner.setSelection(menu_n);

            int i=0;
            (seek_bar[i++] = findViewById(R.id.sb0)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb1)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb2)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb3)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb4)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb5)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb6)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb7)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb8)).setOnSeekBarChangeListener(this);
            (seek_bar[i++] = findViewById(R.id.sb9)).setOnSeekBarChangeListener(this);

            i=0;
            textV[i++]=findViewById(R.id.ek0);
            textV[i++]=findViewById(R.id.ek1);
            textV[i++]=findViewById(R.id.ek2);
            textV[i++]=findViewById(R.id.ek3);
            textV[i++]=findViewById(R.id.ek4);
            textV[i++]=findViewById(R.id.ek5);
            textV[i++]=findViewById(R.id.ek6);
            textV[i++]=findViewById(R.id.ek7);
            textV[i++]=findViewById(R.id.ek8);
            textV[i++]=findViewById(R.id.ek9);
//------------------------------------------
            textV[i++]=findViewById(R.id.tv0);
            textV[i++]=findViewById(R.id.tv1);
            textV[i++]=findViewById(R.id.tv2);
            textV[i++]=findViewById(R.id.tv3);
            textV[i++]=findViewById(R.id.tv4);
            textV[i++]=findViewById(R.id.tv5);
            textV[i++]=findViewById(R.id.tv6);
            textV[i++]=findViewById(R.id.tv7);
            textV[i++]=findViewById(R.id.tv8);
            textV[i++]=findViewById(R.id.tv9);

          //  update();
           // download_settings(menu_n,true);

        }catch (Exception e){
           // Log.i("ERROR","settings "+e.toString());
        }
    }


    public void   update_phone_settings(int index) {
        if (index==5) {
            MainActivity.yaw_correction = Float.parseFloat(textV[2].getText().toString());
            if (MainActivity.yaw_correction>45)
                MainActivity.yaw_correction=45;
            else if (MainActivity.yaw_correction<-45)
                MainActivity.yaw_correction=-45;
            Disk.save_settings("yaw_correction",textV[2].getText().toString());
        }

    }
    public void upload_settings(View view) {
        if (Telemetry.n_settings!= -2) {
          update_phone_settings(Telemetry.n_settings);
           // Log.i("SETT1", "UPLOADING SETINGS");
            Commander.n = menu_n;
            for (int i = 0; i < 10; i++) {
                if (textV[i].getText().toString().length() > 0)
                    Commander.sets[i] = Float.parseFloat(textV[i].getText().toString());
                else
                    Commander.sets[i] = 1;
            }
            Commander.settings = true;
           // Log.i("SETT1", "setting=true");
            // download_settings(menu_n,true);
            //finish();
            update();
        }
    }

    @Override
    public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
        menu_n=position;
        update();
       // download_settings(menu_n,false);
    }


    @Override
    public void onNothingSelected(AdapterView<?> adapterView) {

    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {

        double dk=seek_bar_default_progress;
        int i=seekBar.getId()-seek_bar[0].getId();
        double f = Telemetry.settings[i];
        double maxChange=def_change[menu_n][i];//              seek_bar_default_change;

        if (maxChange>1){
            if (progress!=(int)dk) {
                double p = progress;
                f += (p - dk) / dk * maxChange;
               // f=Math.ceil(f);
            }
        }else {

            String sf[] = a[menu_n][i].split(",");
            if (sf.length >= 4) {
                dk = 0.5 * Float.parseFloat(sf[1]);
                maxChange = 0.01 * Float.parseFloat(sf[3]);
            }

            double k = maxChange * (dk - progress) / dk;
            f -= f * k;
            // textV[i].setText(Float.toString((float)(f)));

        }
        textV[i].setText(String.format("%.5f", f).replace(',', '.'));
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {

    }
}
