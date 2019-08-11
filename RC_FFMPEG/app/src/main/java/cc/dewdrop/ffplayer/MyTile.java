package cc.dewdrop.ffplayer;

import android.graphics.Bitmap;

/**
 * Created by igor on 8/19/2016.
 */
public class MyTile {
    public Bitmap img;
    public int   size;
    public MyTile(){
        img=null;
        size=0;
    }
    public void flush(){
        if (img!=null){
            //img.flush();
            img=null;
            size=0;
        }
    }

}