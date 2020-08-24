package cc.dewdrop.ffplayer;
import android.content.Context;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import cc.dewdrop.ffplayer.myTools.BatteryMonitor;
import cc.dewdrop.ffplayer.myTools.BeepHelper;
import cc.dewdrop.ffplayer.myTools.Camera_pitch_cntr;
import cc.dewdrop.ffplayer.myTools.FlightTextInfo;
import cc.dewdrop.ffplayer.myTools.Img_button;
import cc.dewdrop.ffplayer.myTools.Joystick;
import cc.dewdrop.ffplayer.myTools.Monitor;
import cc.dewdrop.ffplayer.myTools.Square_Cells;

public class DrawView extends View {



    final static float THROTTLE_K =0.5f;
    static public double wrap_180(double x) {return (x < -180 ? x+360 : (x > 180 ? x - 360: x));}
    public static ScaleGestureDetector mScaleGestureDetector;
    final float max_manual_thr_index = 5;
    final static int  viewMain=0;
    final static int  viewMenu=1;
    public static Rect za_cntrl;
    static private Paint gray_opaq;
    static public Camera_pitch_cntr cam_p_c;
    static public Paint black;
    static public float sm[];
    BatteryMonitor batMon;
    private static Joystick j_left,j_right;
    static public Img_button yaw_off, desc_off, pitch_off, roll_off,menu;
    static public Img_button pitch_roll_off,go_to_home;
    static public Img_button showMap,showSettings,hold_alt,smart_ctrl,extra_buttons;
    static public Img_button[]motors_on=new Img_button[2];

    static public Img_button control_type_acc_, head_less_;
    static public  float maxAngle=35;
    static public  float jleftK=1;
    static private float maxAngleChangerStep=-10;
    Monitor monitor;
    static private int screen=viewMain;
    FlightTextInfo ftx;
    static public Img_button exitMenu,exitProg,reboot,shutdown,comp_calibr,comp_m_calibr,gps_on_off;
    static public Img_button  fpv_,vrc_,photo_,do_prog;

    static Paint green_c = new Paint();

    static public boolean is_on_screen_the_menu(){
        return screen==viewMenu;
    }
    static public void turn2MainScreen(){
        screen=viewMain;
    }

    Square_Cells sc;

    static float heading=0;

    private static int old_tel_counter=0;




    String constStrLen(final String in, final int len){
        if (in.length()>len)
            return in.substring(0,len);
        else if (in.length()<len)
            return in+"0000000000".substring(0,len-in.length());
        return in;
    }

    private static long message_time=0;
    float v_speed =0;

    boolean old_hold_on=false;
    private  void updateControls(){
        if (MainActivity.altHoldF())
            j_left.setLabel(Integer.toString((int) (100.0 * jleftK)));
        else
            j_left.setLabel("+");
        motors_on[0].enabled(Commander.link);
        motors_on[1].enabled(Commander.link);
        boolean fpvf=Commander.link && ((MainActivity.control_bits&MainActivity.WIFI_CAMERA_FOUND)>0);
        fpv_.enabled(fpvf);
        photo_.enabled(fpvf);
        vrc_.enabled(fpvf);
        do_prog.enabled(MainActivity.prog_is_loaded());
        ftx.p[ftx.LOC]=constStrLen(Double.toString(Telemetry.lat),8)
                +"  "+constStrLen(Double.toString(Telemetry.lon),8) + " | -"
                + (Telemetry.status&127)
                + " -" + ((Telemetry.status>>7)&127) + (((MainActivity.control_bits&MainActivity.INET_OK)>0)?" w":" ");
        ftx.p[ftx._2HM]="2h: "+Integer.toString((int)Telemetry.dist)+"  h:"+Integer.toString(Telemetry.r_accuracy_hor_pos)+"v:"+Integer.toString(Telemetry.r_acuracy_ver_pos);
        ftx.p[ftx.THR]=constStrLen(Double.toString(Telemetry.realThrottle),4);
        ftx.p[ftx.VIBR]=constStrLen(Double.toString(Telemetry.vibration/1000),5)+ " ,CPU "+Telemetry.cpu_temp+" C°";
        ftx.p[ftx.BAT]=constStrLen(Telemetry.batery,3);
        ftx.p[ftx.YAW]=Integer.toString((int)Telemetry.heading);
        ftx.p[ftx.CAM_ANG]=Integer.toString(Telemetry.gimbalPitch);
        ftx.p[ftx.CAM_ZOOM]=Integer.toString(Commander.fpv_zoom-1);
        ftx.p[ftx.CUR]=Integer.toString((int)(Telemetry.current*Telemetry.batVolt*0.00001f))+"W "+Integer.toString((int)(Telemetry.battery_consumption ));
        if (Telemetry.on_power_time==0)
            ftx.p[ftx.M_ON_T]="00:00:00";
        else {
            double sec= 0.001 * Telemetry.on_power_time;
            int h=(int)Math.floor(sec/3600);
            String sh=((h<10)?"0":"")+h;
            sec-=h*3600;
            int m=(int)Math.floor(sec/60);
            String sm=((m<10)?"0":"")+m;
            sec-=m*60;
            int s=(int)sec;
            String ss=((s<10)?"0":"")+s;
            ftx.p[ftx.M_ON_T] = ""+sh+":"+sm+":"+ss;
        }
        v_speed =(float)Telemetry.v_speed;
        ftx.p[ftx.VSPEED]=constStrLen(Double.toString(v_speed),5);

        if (Telemetry.messages!=null) {
            message_time=System.currentTimeMillis();
            ftx.p[ftx.MESGE] = Telemetry.messages;
            Telemetry.messages = null;
        }else if (message_time>0 && System.currentTimeMillis()-message_time>5000){
            ftx.p[ftx.MESGE]="";
            message_time=0;
        }
        if (old_tel_counter < Telemetry.get_counter()) {
            old_tel_counter=Telemetry.get_counter();

            cam_p_c.gimbal_pitch_add(0,0,Commander.fpv_zoom);//update
            if (motors_on[0].is_pressed()==motors_on[1].is_pressed()) {
                motors_on[0].set(MainActivity.motorsOnF());
                motors_on[1].set(MainActivity.motorsOnF());


            }
            go_to_home.set(MainActivity.toHomeF());
//------------------------------------------------------------------------------------------------------


            smart_ctrl.set(MainActivity.smartCntrF());
            hold_alt.set(MainActivity.altHoldF());

          //  if ((MainActivity.command_bits_& MainActivity.Z_STAB) == 0)
          //      j_left.set_return_back_Y(MainActivity.altHoldF());



            if (old_hold_on != hold_alt.is_pressed()) {
                old_hold_on=hold_alt.is_pressed();
              //  if (MainActivity.toHomeF()==false && MainActivity.progF()==false) {
                //    MainActivity.altHold();
                    j_left.set_return_back_Y(hold_alt.is_pressed());
                    desc_off.set(false);
                    j_left.set_block_Y(false);
                    if (hold_alt.is_pressed() == false)
                        j_left.setJosticY( (max_manual_thr_index * (THROTTLE_K - Telemetry.corectThrottle())));
                    else
                        j_left.setJosticY(0);
              //  }
            }













            do_prog.set(MainActivity.progF());
        }
    }





    void menu_DrawView(final Context context){

        sc=new Square_Cells(9,0,0.3f,sm);
        int nX=sc.getMaxX();
        int nY=sc.getMaxY();

        exitMenu =new Img_button(sc.getRect(0,0),
                context.getResources().getDrawable(R.drawable.left),
                context.getResources().getDrawable(R.drawable.left),false);

        exitProg=new Img_button(sc.getRect(0,1),
                context.getResources().getDrawable(R.drawable.exit),
                context.getResources().getDrawable(R.drawable.exit),false);

        reboot=new Img_button(sc.getRect(nX-1,1),
                context.getResources().getDrawable(R.drawable.reboot),
                context.getResources().getDrawable(R.drawable.reboot),false);

        shutdown=new Img_button(sc.getRect(nX-1,0),
                context.getResources().getDrawable(R.drawable.shutdown),
                context.getResources().getDrawable(R.drawable.shutdown),false);
        comp_calibr=new Img_button(sc.getRect(nX/2,0),
                context.getResources().getDrawable(R.drawable.compass_on),
                context.getResources().getDrawable(R.drawable.compass_on),true);
        comp_m_calibr=new Img_button(sc.getRect(nX/2,1),
                context.getResources().getDrawable(R.drawable.comp_m_c),
                context.getResources().getDrawable(R.drawable.comp_m_c),true);
        gps_on_off=new Img_button(sc.getRect(nX/2,2),
                context.getResources().getDrawable(R.drawable.gps_off),
                context.getResources().getDrawable(R.drawable.gps),true);



        showMap =new Img_button(sc.getRect(nX/2,3),
                context.getResources().getDrawable(R.drawable.route),
                context.getResources().getDrawable(R.drawable.route),false);
        showSettings=new Img_button(sc.getRect(1,0),
                context.getResources().getDrawable(R.drawable.settings),
                context.getResources().getDrawable(R.drawable.settings),false);

    }
    void main_DrawView(final Context context){




        float bR=sm[2]*1.2f;
        float thumb=sm[1]/sm[3];
        float k=0.1f;
        //  j_left=new Joystick((int)(sm[2]*k),(int)(sm[1]-sm[2]*(1+k)),(int)sm[2],true,false,white);
        //  j_right=new Joystick((int)(sm[0]-sm[2]*(1+k)),(int)(sm[1]-sm[2]*(1+k)),(int)sm[2],true,true,white);
        final float monSize=bR/2.5f;


        sc=new Square_Cells(13,0,0.3f,sm);
        int nX=sc.getMaxX();
        int nY=sc.getMaxY();

        za_cntrl=new Rect((int)(sm[0]/2-sm[0]/7),(int)(sm[2]/2.4),(int)(sm[0]/2+sm[0]/7),(int)(sm[1]));
        gray_opaq =new Paint();
        gray_opaq.setColor(Color.GRAY);
        //gray_opaq.setAlpha(30);
        gray_opaq.setStyle(Paint.Style.STROKE);
        gray_opaq.setStrokeWidth(1);
        cam_p_c=new Camera_pitch_cntr();



        ftx=new FlightTextInfo(
                za_cntrl,
                true,
                true,
                true,
                true,
                true,
                true,
                true,
                true,
                true,
                true,
                true,
                -1);
        ftx.setPaint(ftx.MESGE,0xff00ff00);
        monitor =new Monitor((int)(sm[0]/2),(int)(sm[1]-monSize/2),(int)(monSize),
                BitmapFactory.decodeResource(getResources(), R.drawable.angle),
                BitmapFactory.decodeResource(getResources(), R.drawable.cmps));

        batMon=new BatteryMonitor((float)((sm[0]-monSize)/2),(int)(sm[1]-monSize*1.7),monSize);

        final int  jbuttonsY=(int)(nY-bR/(sm[1]/nY))-1;
        j_left=new Joystick(bR*k,sm[1]-bR*(1+k),bR,true,true,false,false,green_c);
        j_right=new Joystick(sm[0]-bR*(1+k),sm[1]-bR*(1+k),bR,true,true,false,false,green_c);
        j_right.setLabel(Integer.toString((int)(100.0*maxAngle/45)));
        j_left.setLabel(Integer.toString((int)(100.0*jleftK)));
        ///  Rect r=new Rect(100,100,100+(int)(sm[2]/3f),100+(int)(sm[2]/3f));


        final int bs=(int)(sm[2]/3);


        pitch_off =new Img_button(sc.getRect(nX-2,jbuttonsY),
                context.getResources().getDrawable(R.drawable.x_on),
                context.getResources().getDrawable(R.drawable.x_off),true);

        pitch_roll_off =new Img_button(sc.getRect(nX-1,jbuttonsY),
                context.getResources().getDrawable(R.drawable.rotate_on),
                context.getResources().getDrawable(R.drawable.rotate_off),true);


        roll_off =new Img_button(sc.getRect(nX-1,jbuttonsY),
                context.getResources().getDrawable(R.drawable.y_on),
                context.getResources().getDrawable(R.drawable.y_off),true);



        yaw_off =new Img_button(sc.getRect(1,jbuttonsY),
                context.getResources().getDrawable(R.drawable.x_on),
                context.getResources().getDrawable(R.drawable.x_off),true);

        desc_off =new Img_button(sc.getRect(0,jbuttonsY),
                context.getResources().getDrawable(R.drawable.y_on),
                context.getResources().getDrawable(R.drawable.y_off),true);



        control_type_acc_ =new Img_button(sc.getRect(nX-2,0),
                context.getResources().getDrawable(R.drawable.touch),
                context.getResources().getDrawable(R.drawable.gyro),true);


        head_less_ =new Img_button(sc.getRect(1,0),
                context.getResources().getDrawable(R.drawable.touch),
                context.getResources().getDrawable(R.drawable.compass_on),true);
        head_less_.enabled(MainActivity.magnetometerWork);
        

        fpv_ =new Img_button(sc.getRect(4,0),
                context.getResources().getDrawable(R.drawable.fpv_off),
                context.getResources().getDrawable(R.drawable.fpv),true);

        vrc_ =new Img_button(sc.getRect(5,0),
                context.getResources().getDrawable(R.drawable.vrc_off),
                context.getResources().getDrawable(R.drawable.vrc_on),true);
        photo_ =new Img_button(sc.getRect(6,0),
                context.getResources().getDrawable(R.drawable.photo),
                context.getResources().getDrawable(R.drawable.photo),true);

        menu=new Img_button(sc.getRect(7,0),
                context.getResources().getDrawable(R.drawable.menu),
                context.getResources().getDrawable(R.drawable.menu),true);



        motors_on[0]=new Img_button(sc.getRect(0,0),
                context.getResources().getDrawable(R.drawable.green),
                context.getResources().getDrawable(R.drawable.red),true);
        //on_off[0].enabled(false);

        motors_on[1]=new Img_button(sc.getRect(nX-1,0),
                context.getResources().getDrawable(R.drawable.green),
                context.getResources().getDrawable(R.drawable.red),true);
        //on_off[0].enabled(false);

        hold_alt=new Img_button(sc.getRect(2,0),
                context.getResources().getDrawable(R.drawable.alt_hold_offt),
                context.getResources().getDrawable(R.drawable.alt_hold_on),true);
        hold_alt.set(true);

        smart_ctrl=new Img_button(sc.getRect(nX-3,0),
                context.getResources().getDrawable(R.drawable.smart_off),
                context.getResources().getDrawable(R.drawable.smart_on),true);
        smart_ctrl.set(true);
        extra_buttons=new Img_button(sc.getRect(8,0),
                context.getResources().getDrawable(R.drawable.circle),
                context.getResources().getDrawable(R.drawable.extra_buttons),true);
       // extra_buttons.set(true);
        go_to_home =new Img_button(sc.getRect(9,0),
                context.getResources().getDrawable(R.drawable.go2home_off),
                context.getResources().getDrawable(R.drawable.go2home),false);

        do_prog =new Img_button(sc.getRect(3,0),
                context.getResources().getDrawable(R.drawable.route),
                context.getResources().getDrawable(R.drawable.prog),true);
        do_prog.enabled(false);



    }
    //--------------------------------------------------------------------------------------------
    public DrawView(Context context) {
        super(context);

        black=new Paint();
        black.setColor(Color.BLACK);

        sm=MainActivity.screenMetrics;

        green_c.setColor(Color.GREEN);
        green_c.setStrokeWidth(sm[2]/100);
        green_c.setAlpha(255);

        main_DrawView(context);
        menu_DrawView(context);


    }


    static public void fpv_stop(){
        if (Commander.fpv_addr!=0){
            Log.d("FPV", "STOP");

            MainActivity.stopVideo();
            Commander.fpv_addr = 0;
            Commander.fpv_port = 0;
            Commander.fpv = true;
        }
    }


    static public void fpv_start_stop(){
        if (fpv_.is_pressed() && Commander.fpv_addr==0) {
            String myIP=Net.getIpAddress();
            Commander.fpv_addr=(byte)Integer.parseInt(myIP.substring(myIP.lastIndexOf('.')+1));
            Commander.fpv_port=5544;
            Commander.fpv_zoom=1;
            Commander.fpv=true;
            Log.d("FPV","START");
            MainActivity.startFPV_Video();

        }else {
            fpv_stop();
        }
    }
    private void fpv_photo_video(final MotionEvent event){
        vrc_.onTouchEvent(event);
        if (vrcf != vrc_.is_pressed()) {
            vrcf = vrc_.is_pressed();
            MainActivity.stopVideo();
            Commander.fpv_code = (short) ((vrcf) ? 513 : 514);
            Commander.fpv = true;


            Thread thread = new Thread() {
                @Override
                public void run() {
                    try {
                        Thread.sleep(3000);
                    } catch (InterruptedException ex) {
                        Thread.currentThread().interrupt();
                    }
                    MainActivity.startFPV_Video();
                }
            };
            thread.start();
        }
        photo_.onTouchEvent(event);
        if (photo_.getStat() == 3) {
            Commander.fpv_code = 769;
            Commander.fpv = true;
           // Commander.fpv_zoom=1;

        }
    }

    static boolean vrcf=false;
    static public boolean thumbed = false;


    void yaw_controls(final MotionEvent event){
        head_less_.onTouchEvent(event);
        yaw_off.onTouchEvent(event);
        if (head_less_.getStat()==3){
            yaw_off.set(head_less_.is_pressed());
            j_left.set_block_X(yaw_off.is_pressed());
            if (head_less_.is_pressed()){
                heading=Telemetry.heading-MainActivity.get_yaw();
            }else {
              //  Commander.heading=heading = MainActivity.yaw;
                Commander.heading=heading=Telemetry.heading;
                Commander.headingOffset=0;
            }
          //  heading=(float)(Telemetry.heading-MainActivity.yaw);
           // Commander.headingOffset=0;
        }
        if (yaw_off.getStat()==3){
            j_left.set_block_X(yaw_off.is_pressed());

        }
    }
    void pitch_roll_controls(final MotionEvent event){
        control_type_acc_.onTouchEvent(event);
        if (control_type_acc_.getStat()==3){
            if (control_type_acc_.is_pressed()){
                pitch_roll_off.set(true);
                j_right.set_block_X(true);
                j_right.set_block_Y(true);
            }else{
                pitch_off.set(false);
                roll_off.set(false);
                j_right.set_block_X(false);
                j_right.set_block_Y(false);
            }
        }


        if (control_type_acc_.is_pressed()){
            pitch_roll_off.onTouchEvent(event);
            if (pitch_roll_off.getStat()==3){
                j_right.set_block_X(pitch_roll_off.is_pressed());
                j_right.set_block_Y(pitch_roll_off.is_pressed());
            }
        }else{
            pitch_off.onTouchEvent(event);
            if (pitch_off.getStat()==3)
                j_right.set_block_X(pitch_off.is_pressed());
            roll_off.onTouchEvent(event);
            if (roll_off.getStat()==3)
                j_right.set_block_Y(roll_off.is_pressed());
        }
    }
    void motors_control(final MotionEvent event){
        motors_on[0].onTouchEvent(event);
        motors_on[1].onTouchEvent(event);
        if ( motors_on[0].is_pressed()==motors_on[1].is_pressed() && MainActivity.motorsOnF()!=motors_on[0].is_pressed() &&
                ((motors_on[0].getStat())==3 || (motors_on[1].getStat())==3)) {
            MainActivity.start_stop();

            yaw_off.set(true);
            j_left.set_block_X(true);
            //Log.d("PWR","PWR+"+Integer.toString(stat1)+" "+Integer.toString(stat2));
        }
    }

    int test4sqere(final MotionEvent event,final Rect za_cntrl){
        //int actionMask = event.getActionMasked();
        int inSqcnt=0;
        for (int i=0; i<event.getPointerCount(); i++) {
            int index = event.findPointerIndex(event.getPointerId(i));
            // int index = event.getActionIndex();
            final float gx = event.getX(index);
            final float gy = event.getY(index);
            inSqcnt += (gx >= za_cntrl.left && gx <= za_cntrl.right &&
                    gy >= za_cntrl.top && gy <= za_cntrl.bottom)?1:0;
        }
        return inSqcnt;
    }

    boolean main_onTouchEvent(final MotionEvent event){

        extra_buttons.onTouchEvent(event);
        menu.onTouchEvent(event);
        if (menu.getStat()==3)
            screen=viewMenu;

        fpv_.onTouchEvent(event);
        if (fpv_.getStat()==3)
            fpv_start_stop();

        fpv_photo_video(event);

        motors_control(event);

        int isc=test4sqere(event,za_cntrl);
        if (isc>1) {
            cam_p_c.reset();
            mScaleGestureDetector.onTouchEvent(event);
        }else
        if (isc==1)
              cam_p_c.onTouchEvent(event, Commander.fpv_zoom);


        yaw_controls(event);
        pitch_roll_controls(event);

        smart_ctrl.onTouchEvent(event);



        if (smart_ctrl.getStat()==3)
            if (MainActivity.toHomeF()==false && MainActivity.progF()==false)
                MainActivity.smartCtrl();

        hold_alt.onTouchEvent(event);
        if (hold_alt.getStat()==3) {
            if (MainActivity.toHomeF()==false && MainActivity.progF()==false) {
                MainActivity.altHold();
                j_left.set_return_back_Y(hold_alt.is_pressed());
                desc_off.set(false);
                j_left.set_block_Y(false);
                old_hold_on=hold_alt.is_pressed();
                if (hold_alt.is_pressed() == false)
                    j_left.setJosticY( (max_manual_thr_index * (THROTTLE_K - Telemetry.corectThrottle())));
                else
                    j_left.setJosticY(0);
            }
        }

        desc_off.onTouchEvent(event);
        if (desc_off.getStat()==3)
            j_left.set_block_Y(desc_off.is_pressed());
        go_to_home.onTouchEvent(event);
        if (go_to_home.getStat()==3) {
            MainActivity.toHome();
            if (head_less_.is_pressed()) {
                Commander.headingOffset = (float) (Telemetry.heading - MainActivity.get_yaw());
              //  Commander.heading = heading = (float) MainActivity.yaw;
            }else{
                Commander.heading = heading=Telemetry.heading;
                Commander.headingOffset=0;
            }
        }

        j_left.onTouchEvent(event);
        j_right.onTouchEvent(event);
        if (j_left.double_touch){
            if ((MainActivity.control_bits & MainActivity.Z_STAB) !=0) {
                BeepHelper.beep();
                jleftK -= 0.25;
                if (jleftK <= 0)
                    jleftK = 1;

            }
            j_left.double_touch=false;
        }
		if (j_right.double_touch){


            BeepHelper.beep();
            if (maxAngle==15)
                maxAngleChangerStep=10;
            else
            if (maxAngle==45)
                maxAngleChangerStep=-10;
            maxAngle+=maxAngleChangerStep;

            j_right.setLabel(Integer.toString((int)(100.0*maxAngle/45)));
            //Log.d("TELE", " OK " + maxAngle);

            j_right.double_touch=false;
        }
        do_prog.onTouchEvent(event);
        if (do_prog.getStat()==3){
            MainActivity.Prog();
        }

        // invalidate();
        return true;
    }


    boolean menu_onTouchEvent(final MotionEvent event){
        exitMenu.onTouchEvent(event);
        if (exitMenu.getStat()==3) {
            screen = viewMain;
            return true;
        }
        showMap.onTouchEvent(event);
        exitProg.onTouchEvent(event);
        reboot.onTouchEvent(event);
        shutdown.onTouchEvent(event);
        comp_calibr.onTouchEvent(event);
        comp_calibr.onTouchEvent(event);
        comp_m_calibr.onTouchEvent(event);
        gps_on_off.onTouchEvent(event);
        showSettings.onTouchEvent(event);




        return true;
    }









    @Override
    public boolean onTouchEvent(MotionEvent event) {
        boolean ret = true;
        old_tel_counter=Telemetry.get_counter()+1000;
        switch(screen) {
            case viewMain:
                ret = main_onTouchEvent(event);
                break;
            case viewMenu:
                ret = menu_onTouchEvent(event);
                break;
        }
        old_tel_counter=Telemetry.get_counter()+3;
        MainActivity.update=true;
        return ret;
    }

    static int i = 0;
    float angle;

    //-------------------------------------------------------------------------------------------
    void setMonitor(){
        monitor.setSpeed(Telemetry.speed);
        monitor.setRoll(-Telemetry.roll);
        monitor.setPitch(Telemetry.pitch);
        monitor.setYaw(Telemetry.heading);
        monitor.setHeight(Telemetry._alt);
    }
    //-------------------------------------------------------------------------------------------








    double old_yaw=0;
    double ma_pitch=0,ma_roll=0;
    long old_time=0;
    void main_onDraw(final Canvas c){

        updateControls();

        if (MainActivity.motorsOnF()==false)
            heading=(float)Telemetry.heading;

        extra_buttons.paint(c);

        old_yaw=MainActivity.get_yaw();
        ma_pitch+=(MainActivity.pitch / maxAngle - ma_pitch)*1;
        ma_roll+=(MainActivity.roll  / maxAngle - ma_roll)*1;

        ////////////////////////////////////////////////////////////////////////////////////////////

        long now=System.currentTimeMillis();
        double dt=0.001*(now-old_time);
        old_time=now;
        if (dt>0.1)
            dt=0.1;

        if (control_type_acc_.is_pressed()) {
            j_right.setJosticY((float) (ma_pitch));
            j_right.setJosticX((float) (ma_roll));
        }
        if ((MainActivity.control_bits&MainActivity.MOTORS_ON)!=0){

            if (head_less_.is_pressed()){   //old control type


                //отключить флаг управления всегда включен
                Commander.heading=(float)MainActivity.get_yaw();
                heading += 45 * j_left.get_JX() * dt;
                heading=(float)wrap_180(heading);
                heading=Commander.headingOffset = (yaw_off.is_pressed())?0: heading;


                //Log.d("COMM", heading +" "+j_left.getX()+ "dt="+dt);

            }else{                          //new control type
                if (j_left.get_JX()!=0)
                    Commander.heading =  Telemetry.heading; //ok
                Commander.headingOffset =  j_left.get_JX()*90;
            }
        }else{
            Commander.headingOffset=0;
            Commander.heading=heading=Telemetry.heading;
        }


        batMon.setVoltage(0.25f*Telemetry.batVolt);



        if (hold_alt.is_pressed()){
            Commander.throttle=0.5f+j_left.get_neg_Y()/2;
        }else {
            Commander.throttle = THROTTLE_K + j_left.get_neg_Y() / max_manual_thr_index;

        }

      //  Log.d("JLEFT",Double.toString(j_left.getY()));

        Commander.roll = j_right.get_JX() * maxAngle;
        Commander.pitch = -j_right.get_neg_Y() * maxAngle;




        setMonitor();
        monitor.paint(c);

        yaw_off.paint(c);
        desc_off.paint(c);

        if (control_type_acc_.is_pressed())
            pitch_roll_off.paint(c);
        else {
            pitch_off.paint(c);
            roll_off.paint(c);
        }

        head_less_.paint(c);
        menu.paint(c);
        control_type_acc_.paint(c);
        j_left.paint(c);
        j_right.paint(c);
        motors_on[0].paint(c);
        motors_on[1].paint(c);
        batMon.paint(c);
        fpv_.paint(c);
        smart_ctrl.paint(c);
        hold_alt.paint(c);
        go_to_home.paint(c);
      //  if (fpv.is_pressed()) {
            vrc_.paint(c);
            photo_.paint(c);
       // }
        do_prog.paint(c);



        c.drawRect(za_cntrl,gray_opaq);

        if (extra_buttons.is_pressed() || message_time>0)
            ftx.paint(c);

    }
    void menu_onDraw(final Canvas c){
        showMap.paint(c);
        exitMenu.paint(c);
        exitProg.paint(c);
        reboot.paint(c);
        shutdown.paint(c);
        comp_calibr.paint(c);
        comp_calibr.paint(c);
        comp_m_calibr.paint(c);
        gps_on_off.paint(c);
        showSettings.paint(c);
    }


    public void onDraw(Canvas c) {





        super.onDraw(c);
        //

        if (fpv_.is_pressed()==false)
            c.drawRect(0,0,sm[0],sm[1],black);

        switch(screen) {
            case viewMain:
                main_onDraw(c);
                break;
            case viewMenu:
                menu_onDraw(c);
                break;
        }


    }
}




