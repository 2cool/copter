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
import java.util.Calendar;

public class Disk {

    private static FileOutputStream fos=null;
    private static File myFile=null;
    private static String filename=null;

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



    public static void saveLatLonAlt(String fname,double lat, double lon, double alt){
        // final File file = new File("/sdcard/RC/start_location.save");
        try {
            OutputStream os = new FileOutputStream(fname);//"/sdcard/RC/start_location.save");

            String t=Double.toString(lat)+","+Double.toString(lon)+","+Double.toString(alt);
            os.write(t.getBytes());

            os.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public static boolean loadLatLonAlt(String fname,boolean to_telemetry){
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
            if (to_telemetry){
                Telemetry.lat=Double.parseDouble(s[0]);
                Telemetry.lon=Double.parseDouble(s[1]);
                Telemetry._alt=(float)Double.parseDouble(s[2]);
            }else {
                Telemetry.autoLat = Double.parseDouble(s[0]);
                Telemetry.autoLon = Double.parseDouble(s[1]);
                //   Telemetry.autoLat=Double.parseDouble(s[0]);
            }

            is.close();

        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }

        return true;


        // autoLat=0,autoLon=0;
    }
    public static String[] getIP(String myIP){

        try {
            String out[]=new String[10];
            int outI=0;
            String ip=myIP.substring(0,myIP.lastIndexOf('.'));

            File f = new File("/sdcard/RC");
            if (f.exists()==false) {
                File folder = new File("/sdcard/RC");
                folder.mkdirs();
                folder = new File("/sdcard/RC/PROGS");
                folder.mkdir();
            }

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

            Log.i("MSGG", "disk close");


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
            Log.i("MSGG", "exception");
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
