package cc.dewdrop.ffplayer;


import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

public class Disk {

    private static FileOutputStream fos=null;
    private static File myFile=null;
    private static String filename=null;

    private static FileOutputStream bb_fos=null;
    private static File black_box_file=null;
    /* Checks if external storage is available for read and write */
    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }

    /* Checks if external storage is available to at least read */
    public boolean isExternalStorageReadable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state) ||
                Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
            return true;
        }
        return false;
    }

    //-------------------------------------------
    public static String getLOG_FNAME(){

        int cnt=999;
        try {
            InputStream is = new FileInputStream("/sdcard/RC/counter.txt");
            BufferedReader buf = new BufferedReader(new InputStreamReader(is));
            String s="";
            s = buf.readLine();
            cnt=Integer.parseInt(s);

            is.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        try{
            File file;

            file = new File(Environment.getExternalStorageDirectory(),"RC/counter.txt");
            file.createNewFile();
            //  boolean deleted = file.delete();

            OutputStream os = new FileOutputStream("/sdcard/RC/counter.txt");


            String t=Integer.toString(cnt+1);
            os.write(t.getBytes());
            os.close();

        } catch (Exception e) {
            e.printStackTrace();
        }

        return "RC/"+Integer.toString(cnt)+".log";
    }

    //------------------------------------------
    public static int write2Log(byte b[],int offset,int len){


        try {

            {
                final File file = new File("/sdcard/RC");
                if (!file.exists()) {
                    if (file.mkdir()) {
                        //System.out.println("Directory is created!");
                    } else {
                        System.out.println("Failed to create directory!");
                    }
                }
            }



            if (myFile==null) {
                filename=getLOG_FNAME();
                myFile = new File(Environment.getExternalStorageDirectory(), filename);
                if (!myFile.exists())
                    myFile.createNewFile();
                fos = new FileOutputStream(myFile , false);

            }

            fos.write(b,offset,len);
            fos.flush();

        } catch (Exception e) {
            e.printStackTrace();
            return 1;
        }

        return 0;
    }



    public static void save_location_(String fname, double lat, double lon, double alt){
        if (Telemetry.lat==0 || Telemetry.lon==0 || Telemetry.r_accuracy_hor_pos<20)
            return;
        // final File file = new File("/sdcard/RC/start_location.save");
        try {
            OutputStream os = new FileOutputStream(fname);//"/sdcard/RC/start_location.save");

            String t=lat+","+lon+","+(int)alt;
            os.write(t.getBytes());
            os.close();


            os = new FileOutputStream(fname+"_all.txt",true);
            DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
            Date date = new Date();
            String tt= dateFormat.format(date);
            tt+=" "+t+"\n";
            os.write(tt.getBytes());
            os.close();

            // Log.d("SAVEE","save_loc");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static double _lat, _lon, _alt;

    public static boolean load_location_(String fname){
        final File file = new File(fname);//"/sdcard/RC/start_location.save");
        if (!file.exists()) {
            return false;
        }

        InputStream is= null;
        try {
            is = new FileInputStream(file);

            BufferedReader buf = new BufferedReader(new InputStreamReader(is));
            String line = null;

            line = buf.readLine();

            if (line==null || line.length()<10)
                return false;

            String s[]=line.split(",");
            _lat =Double.parseDouble(s[0]);
            _lon =Double.parseDouble(s[1]);
            _alt =Double.parseDouble(s[2]);
            is.close();

        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }

        return true;


        // autoLat=0,autoLon=0;
    }
    public static void create_folders(){
        File folder = new File("/sdcard/RC");
        folder.mkdirs();
        folder = new File("/sdcard/RC/PROGS");
        folder.mkdir();
    }

    public static void save_settings(String name, String val){
        try{
            final File file = new File("/sdcard/RC/settings.txt");
            InputStream is=new FileInputStream(file);
            BufferedReader buf = new BufferedReader(new InputStreamReader(is));
            String out="";
            while(true) {
                String line = buf.readLine();
                if (line != null) {
                    String setting[] = line.split(",");
                    if (setting[0].endsWith(name)) {
                        setting[1] = val;
                        line = setting[0] + "," + setting[1];
                        Log.d("SAVEE",setting[0]+"="+setting[1]);
                    }
                    out += line + "\n";
                }else
                    break;
            }
            buf.close();
            is.close();
            FileOutputStream stream = new FileOutputStream("/sdcard/RC/settings.txt");
            stream.write(out.getBytes());
            stream.close();

        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
    }

    public static void get_Settings(){
        try {

            File f = new File("/sdcard/RC");
            if (f.exists()==false)
                create_folders();


            final File file = new File("/sdcard/RC/settings.txt");
            if ( file.exists()==false) {
                try {
                    FileOutputStream stream = new FileOutputStream("/sdcard/RC/settings.txt");
                    stream.write("yaw_correction,35\n".getBytes());
                    stream.close();
                }
                catch (Exception e) {

                }
            }

            InputStream is=new FileInputStream(file);
            BufferedReader buf = new BufferedReader(new InputStreamReader(is));
            String line = buf.readLine();
            while (line!=null) {
                String par[] = line.split(",");
                read_one_settings(par);
                line = buf.readLine();
            }

            buf.close();
            is.close();

        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
    }
    public static boolean read_one_settings(String par[]){
        if (par[0].endsWith("yaw_correction")){
            MainActivity.yaw_correction=Float.parseFloat(par[1]);
            Log.d("SAVEE","yaw_correction="+Float.parseFloat(par[1]));
            return true;
        }else
            return false;
    }
    public static String[] getIP(String myIP){

        try {
            String out[]=new String[10];
            int outI=0;
            String ip=myIP.substring(0,myIP.lastIndexOf('.'));

            File f = new File("/sdcard/RC");
            if (f.exists()==false)
                create_folders();

            final File file = new File("/sdcard/RC/ip.set");
            if ( file.exists()==false) {
                try {
                    FileOutputStream stream = new FileOutputStream("/sdcard/RC/ip.set");
                    stream.write("192.168.100.112:9876\n".getBytes());
                    stream.close();
                    stream = new FileOutputStream("/sdcard/RC/counter.txt");
                    stream.write("9999\n".getBytes());
                    stream.close();



                }
                catch (Exception e) {

                }
            }

            InputStream is=new FileInputStream(file);
            BufferedReader buf = new BufferedReader(new InputStreamReader(is));
            String line;
            int n;

            do {
                line = buf.readLine();
                if (line==null || line.length()<10)
                    break;
                n = line.lastIndexOf(ip);
                if (n==0) {
                    out[outI++]=line;
                }
            }while(true);
            buf.close();
            is.close();


            return out;

        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }

    }
    public static int save_black_box_log(String log){

        try {

            {
                final File file = new File("/sdcard/RC/BLACK_BOX/");
                if (!file.exists()) {
                    if (file.mkdir()) {
                        //System.out.println("Directory is created!");
                    } else {
                        System.out.println("Failed to create directory!");
                    }
                }
            }



            if (black_box_file==null) {
                Calendar c = Calendar.getInstance();
                String filename = ""+c.getTime();
                filename=filename.replace(':','_').replace(' ','_');
                filename="/sdcard/RC/BLACK_BOX/"+filename+"_black_box.log";
               // Log.d("DISK_SAVE1",filename);
                black_box_file = new File( filename);
                black_box_file.createNewFile();
               // Log.d("DISK_SAVE1","createNewFile ");
                bb_fos = new FileOutputStream(black_box_file , true);
               // Log.d("DISK_SAVE1","FileOutputStream ");
            }
            byte[] data = log.getBytes();
            try {
               // Log.d("DISK_SAVE","saved  1");
                bb_fos.write(data);

            } catch (Exception e) {
                e.printStackTrace();
                Log.d("DISK_SAVE1","saved err1 "+e.getMessage());
                return 1;
            }

        } catch (Exception e) {
            e.printStackTrace();
            Log.d("DISK_SAVE1","saved err2 "+e.getMessage());
            return 1;
        }
        return 0;
    }

    public static int createOrOpen_(){
        try {

            {
                final File file = new File("/sdcard/RC");
                if (!file.exists()) {
                    if (file.mkdir()) {
                        //System.out.println("Directory is created!");
                    } else {
                        System.out.println("Failed to create directory!");
                    }
                }
            }

            Calendar c = Calendar.getInstance();
            filename = ""+c.getTime();
            filename=filename.replace(':','_').replace(' ','_');
            filename="RC/"+filename+".log";

            if (myFile==null) {
                myFile = new File(Environment.getExternalStorageDirectory(), filename);
                if (!myFile.exists())
                    myFile.createNewFile();
                fos = new FileOutputStream(myFile , false);

            }


        } catch (Exception e) {
            e.printStackTrace();
            return 1;
        }
        return 0;
    }

    public static int close_(){
        try {
            Disk.save_location_(
                    "/sdcard/RC/lostCon_location.save",
                    Telemetry.lat,
                    Telemetry.lon,
                    Telemetry._alt);
          //  Log.i("MSGG", "disk close");

            if (bb_fos!=null){
                bb_fos.flush();
                bb_fos.close();
            }

            if (fos!=null) {
                fos.flush();
                fos.close();
                if (myFile!=null && myFile.exists() &&  myFile.length()==0){
                    myFile.delete();
                }
            }
            fos=null;
            myFile=null;
        }catch (Exception e){
          //  Log.i("MSGG", "exception");
            e.printStackTrace();
            return 1;
        }
        return 0;
    }
    public static int write_(String string) {
        if (fos==null)
            // if (createOrOpen()!=0)
            return 1;

        byte[] data = string.getBytes();
        try {

            fos.write(data);

        } catch (Exception e) {
            e.printStackTrace();
            return 1;
        }


        return 0;
    }









}
