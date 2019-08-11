package cc.dewdrop.ffplayer;

import android.os.Environment;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

/**
 * Created by igor on 28.09.2016.
 */

public class LogReader {



    static int offset=0;

    static String kml="";

    static String getNext(String s){
        if (offset==-1)
            return null;
        int end=s.indexOf('\n',offset);
        String ret;
        if (end==-1){
            ret=s.substring(offset);
            offset=-1;
        }else {
            ret =  s.substring(offset, end);
            offset = end+1;
        }
        return ret;

    }
    static String getFirst(String s){
        offset=0;
        cnt=0;
        kml="<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">\n" +
                "<Document>\n" +
                "\t<name>track.kml</name>\n" +
                "\t\n" +
                "\t<Placemark>\n" +
                "\t\t<name>track</name>\n" +
                "\t\t<styleUrl>#inline1</styleUrl>\n" +
                "\t\t<LineString>\n" +
                "\t\t\t<tessellate>1</tessellate>\n" +
                "\t\t\t<coordinates>";
        int end=s.indexOf('\n',offset);
        String ret=(end!=-1)?s.substring(offset,end):s.substring(offset);
        offset=end;
        return ret;
    }
    static double lat,lon,alt;
    static boolean newLatLon=false;
    static int cnt=0;
    static  void addLogString(String s){
        newLatLon=false;
        if (s.startsWith("TELE,",0)) {
            String fds[] = s.split(",");
            if (fds.length >= 15) {
                if (fds[2].length() == 9) {
                    newLatLon = true;
                    lat = 0.0000001 * Double.parseDouble(fds[2]);
                }
                if (fds[3].length() == 9) {
                    newLatLon = true;
                    lon = 0.0000001 * Double.parseDouble(fds[3]);
                }
                if (newLatLon) {
                    if (cnt>0)
                        kml+=",";
                    kml+=Double.toString(lon)+","+Double.toString(lat)+",0";
                    cnt++;
                }
            }
        }

    }
    //33.18911182217912,47.97026925768822,0
//TELE,1000,479483797,333811305,1,1,-195,1,-3,,,424,422,429,-164
    static public int load(String str){
        String s=getFirst(str);

        addLogString(s);
        while ((s=getNext(str))!=null) {
            addLogString(s);
        }
        kml+="</coordinates>\n" +
                "\t\t</LineString>\n" +
                "\t</Placemark>\n" +
                "</Document>\n" +
                "</kml>";
        File myFile = new File(Environment.getExternalStorageDirectory(), "RC/log.kml");
        if (myFile.exists()){
            myFile.delete();
        }
        if (!myFile.exists())
            try {
                myFile.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        try {
            FileOutputStream fos = new FileOutputStream(myFile , false);
            fos.write(kml.getBytes());
            fos.flush();
            fos.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return 0;
    }
}
