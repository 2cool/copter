package cc.dewdrop.ffplayer;

import android.util.Log;

public class Telemetry {
    private static int  telemetry_couter=0;
    public static int get_counter(){return telemetry_couter;}
    static public int batVolt,current;
    static public boolean maxTelemetry=false;
    static private boolean hom_pos_is_loaded=false;
    static private int old_motors_status=-1;
    static private long oldMsgTimer;
    static private int motorsONtimer=0;
    static public String motors_on_timer="00:00";
    static private long oldTimeMS=0;
    static int messageCode;
    static public int satilites=0;
    static public double lat=47.9;
    static public double lon=33.2523;
    static public int r_accuracy_hor_pos=99,r_acuracy_ver_pos=99;//когда включаем смартконтрол мощность меняестя. (исправить)
    static public double roll=0,pitch=0,yaw=0;
    static public double autoLat=0,autoLon=0;
    static private double oldlat,oldlon;
    static public double dist=0,speed=0,v_speed=0,alt_time=0 ,speed_time=0,alt_speed;
    static public String messages=null;
    static public float heading=0,	battery_consumption=0,vibration=0;
    static public int status=0;
    //	static public String message="message";
    static float motorsTh[];

    static public String lat(){return Double.toString(lat);}

    static double ap_pitch,ap_roll,realThrottle;  //autopilot
    static private  double ap_throttle=0;
    static int gimbalPitch;
    static float _alt=0,old_alt1=-100000;
    static float  relAlt=0;
    static String batery="";
    static final int MIN_VOLT =1280;
    static final int VOLT_50  =1520;
    static boolean F_MIN_VOLT=false;
    static int min_volt=0;
    static boolean F_VOLT50 =false;
    static public double oldAlt=11110,oldLat=0,oldLon=0,oldHeading=110;
    //static public int no_gps_data_counter=100;
    static final long DELTA_TIME_4_MSG = 20;
    static final double GRAD2RAD = 0.01745329251994329576923690768489;


    static public boolean motorsWasOn=false;
    //counter,lat,lon,gpsHeight,bat1,bat2,bat3,pressure,temperature,heading*18000/PI,roll,pitch,messageCode,motorTh0,motorTh1,motorTh2,motorTh3,(bat0+200)/100



    static public float settings[]=new float[10];
    static public int n_settings;
    static private int old_control_bits=0;












    static public void init(){
        //Log.i("TTZZ","INIT TELEM");
        for (int i=0; i<10; i++)
            settings[i]=Commander.NO_DATA;

        old_control_bits=satilites=0;
        r_accuracy_hor_pos=0;

        ap_throttle=0.5f;
        batery="";
        F_MIN_VOLT=false;
        F_VOLT50 =false;
        ap_roll=ap_pitch=roll=pitch=0;
        oldMsgTimer=System.currentTimeMillis()+DELTA_TIME_4_MSG;
        battery_consumption=0;
    }




    static public float corectThrottle(){

        if (ap_throttle < 0.5)
            ap_throttle = 0.5;
        if (ap_throttle > 0.7)
            ap_throttle = 0.7;
        return (float)ap_throttle;//*z;


    }


    static public void readMessages(String msg){
        int i;
        if ((i=msg.indexOf("UPS",0))>=0){
            //Log.i("UPS","read mess "+msg);
            parse_settings(msg, i+3);
            //	i=msg.lastIndexOf(",",i);
        }else {
            messages = msg;
            Log.d("ERRMSG",msg);
        }


    }

    static private void parse_settings(String str, int b){
        // Log.i("UPS","parse: "+str.substring(b));
        String v[]=str.substring(b).split(",");
        int n=Integer.parseInt(v[0]);
        if (n<0){
            n_settings=-1;
            return;
        }
        int i=1;
        for (; i<v.length;i++){
            settings[i-1]=Float.parseFloat(v[i]);
        }
        for (; i<=10; i++){
            settings[i-1]=Commander.NO_DATA;
        }
        n_settings=n;
    }









    static long loaduint64t(byte buf[],int i){
        long vall=       (long)load_uint8(buf,i);
        vall+=    0x100L*(long)load_uint8(buf,i+1);
        vall+=  0x10000L*(long)load_uint8(buf,i+2);
        vall+=0x1000000L*(long)buf[i+3];

        vall+=0x100000000L*(long)buf[i+4];
        vall+=0x10000000000L*(long)buf[i+5];
        vall+=0x1000000000000L*(long)buf[i+6];
        vall+=0x100000000000000L*(long)buf[i+7];
        return vall;
    }




    static int load_int32(byte buf[],int i){
        int vall=       load_uint8(buf,i);
        vall+=    0x100*load_uint8(buf,i+1);
        vall+=  0x10000*load_uint8(buf,i+2);
        vall+=0x1000000*(int)buf[i+3];
        return vall;
    }
    static int load_int16(byte buf[],int i){
        int vall=load_uint8(buf,i);
        int valh=buf[i+1];
        return vall+valh*256;
    }
    static int load_uint8(byte buf[],int i){
        int vall=buf[i];
        vall&=255;
        //if (vall<0)
        //	vall=0-vall;
        return vall;
    }
    //////////////////////////////////////////////////////////////
    static int old_alt=0;








    static public double dist(double lat1,double lon1, double lat2, double lon2){
        final double R = 6371e3; // metres
        double φ1 = lat1*GRAD2RAD;
        double φ2 = lat2*GRAD2RAD;
        double Δφ = (lat2-lat1)*GRAD2RAD;
        double Δλ = (lon2-lon1)*GRAD2RAD;

        double a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
                Math.cos(φ1) * Math.cos(φ2) *
                        Math.sin(Δλ/2) * Math.sin(Δλ/2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

        double d = R * c;
        return d;
    }
    static int old_index=-1;
    static int log_s=0;
    static int block_cnt=0;










    static final int  mask=63;
    static byte [][]log_buffer=new byte[mask+1][1024];
    static int readedI=0;
    static int writedI=0;
    static boolean logThread_f=true;
    static boolean thread_started=false;
    static void startlogThread(){
        if (thread_started==true)
            return;
        thread_started=true;
        Thread thread = new Thread() {
            @Override
            public void run() {
                while (logThread_f==true){
                    if (writedI<readedI){
                        int adr=writedI&mask;
                        int mes_len=log_buffer[adr][0]&255;
                        mes_len +=256*log_buffer[adr][1];
                        Disk.write2Log(log_buffer[adr],0,mes_len);
                        writedI++;
                    }else {
                        try {
                            Thread.sleep(5);
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }

            }
        };
        thread.start();
    }

    static final int MPU_EMU=0;
    static final int MPU_SENS=1;
    static final int HMC_BASE=2;
    static final int HMC_SENS=3;
    static final int HMC_EMU=4;
    static final int GPS_SENS=5;
    static final int TELE = 6;
    static final int COMM = 7;
    static final int EMU = 8;
    static final int AUTO = 9;
    static final int BAL = 10;

    static float M_PI=3.14159265358979323846f;



    static double RAD2GRAD =57.29578;


    static float qw,qx,qy,qz;

    static public double copysign(double x, double y){
        x=Math.abs(x);
        if (y<0)
            x=-x;
        return x;
    }
    static public void toEulerianAngle()
    {
        // roll (x-axis rotation)
        double sinr = +2.0 * (qw * qx + qy * qz);
        double cosr = +1.0 - 2.0 * (qx * qx + qy * qy);

        roll =RAD2GRAD * Math.atan2(sinr, cosr);

        // pitch (y-axis rotation)
        double sinp = +2.0 * (qw * qy - qz * qx);
        if (Math.abs(sinp) >= 1)
            pitch = RAD2GRAD * copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = RAD2GRAD * Math.asin(sinp);

        // yaw (z-axis rotation)
        double siny = +2.0 * (qw * qz + qx * qy);
        double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
        yaw = RAD2GRAD * Math.atan2(siny, cosy);
    }



    static public void mpu_parser(byte buf[],int n){
        int j=n;
        long time =loaduint64t(buf, j);
        j+=8;

        int g[]= new int[3];
        int a[]= new int [3];
        int q[] = new int[4];


        g[0] = load_int16(buf, j);
        j+=2;
        g[1] = load_int16(buf, j);
        j+=2;
        g[2] = load_int16(buf, j);
        j+=2;

        a[0] = load_int16(buf, j);
        j+=2;
        a[1] = load_int16(buf, j);
        j+=2;
        a[2] = load_int16(buf, j);
        j+=2;


        q[0]=load_int32(buf,j);
        j+=4;
        q[1]=load_int32(buf,j);
        j+=4;
        q[2]=load_int32(buf,j);
        j+=4;
        q[3]=load_int32(buf,j);
        j+=4;


        qw = 1.5259e-5f*(float)q[0] / 16384.0f;
        qx = 1.5259e-5f*(float)q[1] / 16384.0f;
        qy = 1.5259e-5f*(float)q[2] / 16384.0f;
        qz = 1.5259e-5f*(float)q[3] / 16384.0f;

        toEulerianAngle();

    }

    static public void barom_parser(byte buf[],int n){

    }

    static public void bal_parser(byte buf[],int i){
        int bank_counter=load_int32(buf,i);
        i+=4;
        i+=16;
        ap_roll=(float)load_int16(buf,i)/16.0;
        i+=2;
        ap_pitch=(float)load_int16(buf,i)/16.0;
        //	i+=2;




    }
    static public void parser(byte buf[],int i){

        int f_len=i+load_int16(buf,i);
        i+=4;
        while(i<f_len) {
            int b = buf[i++];
            int len = load_uint8(buf, i);
            i++;
            if (len == 0) {
                len = load_int16(buf, i);
                i += 2;
            }
            switch (b) {

                case MPU_SENS: {
                    mpu_parser(buf,i);
                    break;
                }
                case HMC_BASE: {

                    break;
                }
                case BAL: {
                    bal_parser(buf,i);
                    return;
                }
            }
            i += len;
        }

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    static public void bufferReader_(byte buf[],int buf_len){
        //   Log.d("BUFREAD","bufRead");
        int i=0;

        //if (buf_len<=4)
        //	return;
        telemetry_couter++;
        MainActivity.control_bits=load_int32(buf,i);

        i+=4;


        realThrottle  = load_int16(buf,i);
        i+=2;
        realThrottle=(realThrottle-1000)*0.001f;


        int ilat=load_int32(buf,i);
        i+=4;
        lat=0.0000001*(double)ilat;
        int ilon=load_int32(buf,i);
        i+=4;
        lon=0.0000001*(double)ilon;
        //Log.i("DKDKD",Double.toString(lat)+ " "+Double.toHexString(lon));
        //---------------------------


        if (hom_pos_is_loaded == false) {
            if (MainActivity.motorsOnF()){
                Disk.loadLatLonAlt("/sdcard/RC/start_location.save",false);
                oldlat=lat;
                oldlon=lon;
               // Log.d("LOAD","LOAD "+autoLat+", "+autoLon);
            }
        }
        hom_pos_is_loaded = true;
        if (MainActivity.motorsOnF() && old_motors_status==1) {
            autoLat = oldlat = lat;
            autoLon = oldlon = lon;
            Disk.saveLatLonAlt("/sdcard/RC/start_location.save", lat, lon, 0);
           // Log.d("LOAD", "SAVE " + autoLat + ", " + autoLon);
        }

        old_motors_status = MainActivity.motorsOnF()?2:1;







        long timems = System.currentTimeMillis();
        if ((timems-oldTimeMS)>=1000) {
            if (oldTimeMS == 0) {
                oldTimeMS = System.currentTimeMillis();
            }else {
                if (MainActivity.motorsOnF()) {
                    motorsONtimer += (int) ((timems - oldTimeMS) / 1000);
                    int minutes=(int)Math.ceil(motorsONtimer/60);
                    int seconds=motorsONtimer-minutes*60;
                    motors_on_timer=Integer.toString(minutes)+":"+((seconds<10)?"0":"")+Integer.toString(seconds);
                }
                oldTimeMS = timems;
            }
        }



        if (MainActivity.motorsOnF()){
            //вічисляем растояние до старта
            dist=(autoLat==0 || autoLon==0)?0:dist(autoLat,autoLon,lat,lon);
          //  Log.d("LOAD","2HOME "+dist+ " "+autoLat + " " + autoLon);
            double dDist=dist(oldlat,oldlon,lat,lon);
            oldlat=lat;
            oldlon=lon;
            long t=System.currentTimeMillis();
            double dt=0.001*(double)(t-speed_time);
            speed_time=t;
            speed+=(dDist/dt-speed)*dt;

        }
        r_accuracy_hor_pos = buf[i++];
        r_acuracy_ver_pos = buf[i++];
        _alt =load_int16(buf,i);
        i+=2;
        _alt*=0.1;

        if (old_alt1==-100000)
            old_alt1=_alt;
        double dalt=_alt-old_alt1;
        old_alt1=_alt;
        long t=System.currentTimeMillis();
        double dt=0.001*(double)(t-alt_time);
        alt_time=t;
        if (dt>0)
            v_speed+=(dalt/dt-v_speed)*0.05;
        if ((MainActivity.control_bits&MainActivity.MOTORS_ON)==MainActivity.MOTORS_ON){
            relAlt=_alt;
        }
        pitch=buf[i++];
        roll=buf[i++];

        ap_pitch=buf[i++];
        ap_roll=buf[i++];

        //i+=4;

        batVolt=load_int16(buf,i);
        i+=2;
        //int b0= 256+load_uint8(buf,i++);
        //int b1= 256+load_uint8(buf,i++);
        //int b2= 256+load_uint8(buf,i++);

        F_MIN_VOLT=batVolt<=MIN_VOLT;

        batery=Integer.toString(batVolt/4);
        battery_consumption=load_int16(buf,i);
        i+=2;
        current=load_int16(buf,i);
        i+=2;
        vibration=load_int16(buf,i);
        i+=2;
        //loadBUF16(i, full_power*10);
        //	loadBUF16(i, Mpu.vibration * 1000);

        gimbalPitch= buf[i++];
        heading=load_int16(buf,i);
        heading/=182.0;
        i+=2;




        status=load_int32(buf,i);
        i+=4;
       // Log.i("STRR","str= "+Integer.toString(status));

        //тут читаем сообщения если они кому нужни так как у нас все данніе риходят и муд
        int mes_len=load_int16(buf,i);
        i+=2;
        if ( mes_len >0 ) {
            readMessages(new String(buf, i, mes_len));
        }
        i+=mes_len;
        mes_len=load_int16(buf,i);
        //parser(buf,i);
        i+=2;
        block_cnt=0;
        while (mes_len>0){
            System.arraycopy(buf, i-2,log_buffer[readedI&mask], 0,  mes_len+2);
            readedI++;

            block_cnt++;
            log_s+=mes_len;


            i+=mes_len;
            mes_len=load_int16(buf,i);

            i+=2;
        }

        //if (block_cnt>1)
        //Log.i("DISK","W... "+Integer.toString(log_s)+"  blocks  "+Integer.toString(block_cnt));

        float z = (float) Math.sqrt( (1 - pitch * pitch*GRAD2RAD*GRAD2RAD ) * (1 - roll * roll * GRAD2RAD*GRAD2RAD) );
        //ap_throttle=realThrottle*z;
        if ((MainActivity.control_bits&MainActivity.MOTORS_ON)==0){
            ap_throttle=realThrottle*z;
        }else {
            ap_throttle += (realThrottle * z - ap_throttle) * 0.03;
        }

/*
	//	msg+=",";
		final long cttt=System.currentTimeMillis();
		if (oldMsgTimer<=cttt) {
			oldMsgTimer+=DELTA_TIME_4_MSG;
			if (msg.matches("TELE,,,,,,,,,,,,,,"))
				Disk.write("n\n");
			else
				Disk.write(msg+"\n");
			while (oldMsgTimer<=cttt){
				oldMsgTimer+=DELTA_TIME_4_MSG;
				Disk.write("t\n");
			}

			oldMsgTimer=cttt;
		}
		readMessages(new String(buf,i,buf_len-i));

		float z = (float) Math.sqrt( (1 - pitch * pitch*GRAD2RAD*GRAD2RAD ) * (1 - roll * roll * GRAD2RAD*GRAD2RAD) );
		//ap_throttle=realThrottle*z;
		if ((MainActivity.control_bits&MainActivity.MOTORS_ON)==0){
			ap_throttle=realThrottle*z;
		}else {
			ap_throttle += (realThrottle * z - ap_throttle) * 0.03;
		}
		*/
    }




}
