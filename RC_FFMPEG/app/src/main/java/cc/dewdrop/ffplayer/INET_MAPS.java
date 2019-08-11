package cc.dewdrop.ffplayer;

import android.graphics.Bitmap;

/**
 * Created by igor on 8/19/2016.
 */
public interface INET_MAPS {
    //public int cnt=0;
    public String png=".png",jpg=".jpg";
    public String getReferer();
    public String getName();
    public String getExt();
    public Bitmap getIcon();
    public String getUrl(final int x,final int y,final int z);
    public void   OkVersion();
    public void   incVersion();
    public void   loadVersion();
    public void   saveVersion();
}
