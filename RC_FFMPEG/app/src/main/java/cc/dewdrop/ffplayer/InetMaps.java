package cc.dewdrop.ffplayer;



import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.net.URL;
import java.net.URLConnection;
import java.util.Stack;

/**
 * Created by igor on 8/19/2016.
 */
class TILE{
    public int x, y, z,hash,error=0;
    public String fn;
    public boolean inProcess=false;

    public TILE(final TILE t){
        inProcess=t.inProcess;
        x=t.x;
        y=t.y;
        z=t.z;
        hash=t.hash;
        fn=t.fn;
        error=t.error;
    }
    public TILE(int x, int y, int z, int hash,String fn){
        this.x=x;
        this.y=y;
        this.z=z;
        this.hash=hash;
        this.fn=fn;//new String(fn);
    }

}

/////////////////////////////////////////////////////////////////////////////////////////

//http://toolserver.org/tiles/hikebike/14/9309/5676.png
class HikeBike implements INET_MAPS{
    public int cnt=0;
    public String getReferer(){return null;}
    public String getName(){return "HikeBike";}
    public String getExt(){return png;}
    public Bitmap getIcon(){return null;}
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        String url="http://toolserver.org/tiles/hikebike/"+Integer.toString(z)+"/" + Integer.toString(x)+"/"+Integer.toString(y)+png;
        return url;
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}



class GoogleMap implements INET_MAPS{
    public int cnt=0;
    public String getReferer(){return null;}
    public String getName(){return "googleMaps";}
    public String getExt(){return png;}
    public Bitmap getIcon(){return null;}//MyI.googleIcon;}
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        String url="http://mt" + Integer.toString(cnt & 1)+".google.com/vt/lyrs=m@146000000&hl=ru&x=";
        url+=Integer.toString(x)+"&y="+Integer.toString(y)+"&z="+Integer.toString(z)+"&s="+"Galileo".substring(0, cnt&7);
        return url;
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}
/////////////////////////////////////////////////////////////////////////////////////////
//   http://khm0.google.com/kh/v=104&src=app&x=304&y=178&z=9&s=Ga
//https://khms0.google.com.ua/kh/v=128&src=app&x=604&y=354&z=10&s=Galile
//   https://khms0.google.com/kh/v=129&src=app&x=9728&y=5700&z=14&s=Gali
//   https://khms1.google.com/kh/v=150&src=app&x=607&y=355&z=10&s=
class GoogleEarth implements INET_MAPS{
    private int cnt;

    public String getReferer(){return null;}
    public String getName(){return "googleEarth";}
    public String getExt(){return jpg;}
    public Bitmap getIcon(){return null;}// MyI.googleIcon;}
    static private int verI=151;
    static private int ver=verI;


    public String getUrl(final int x,final int y,final int z){
        cnt++;
        String url="https://khms"+Integer.toString(cnt&1)+".google.com/kh/v="+Integer.toString(ver)+"&src=app&x=";
        url+=Integer.toString(x)+"&s=&y="+Integer.toString(y)+"&z="+Integer.toString(z)+"&s="+"Galileo".substring(0, cnt&7);
        System.out.println(ver);
        return url;
    }
    @Override
    public void   incVersion(){

    }
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){

    }
}
/////////////////////////////////////////////////////////////////////////////////////////
class GoogleTerrian implements INET_MAPS{
    public int cnt;
    public String getReferer(){return null;}
    public String getName(){return "GoogleTerrian";}
    public String getExt(){return jpg;}
    public Bitmap getIcon(){return null;}// MyI.googleIcon;}
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        String url="http://mt"+Integer.toString(cnt&1)+".google.com/vt/lyrs=t@127,r@160000000&hl=en&src=api&x=";
        url+=Integer.toString(x)+"&y="+Integer.toString(y)+"&z="+Integer.toString(z)+"&s="+"Galileo".substring(0, cnt&7);
        return url;
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}
/////////////////////////////////////////////////////////////////////////////////////////
class MicrosoftEarthAndLabels implements INET_MAPS{
    public int cnt;
    public String getReferer(){return null;}
    public String getName(){return "Bing Bird's eye";}
    public String getExt(){return jpg;}
    public Bitmap getIcon(){return null;}// MyI.bingIcon;}
    public String getUrl(final int xx,final int yy,final int z){
        int x=xx;
        int y=yy;
        cnt++;
        if (z == 0)
            return new GoogleEarth().getUrl(x, y, z);
        String s = "";
        for (int i = 0; i < z; i++)
        {
            s = Integer.toString(((y & 1) * 2) + (x & 1)) + s;
            x >>= 1;
            y >>= 1;
        }
        return "http://ecn.t" + Integer.toString(cnt&3) + ".tiles.virtualearth.net/tiles/h" + s + ".jpeg?g=668&mkt=en-us&n=z";
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}
//----------------------------------------------------------------------------------------------
class MicrosoftMap implements INET_MAPS{
    public int cnt;
    @Override
    public String getReferer(){return null;}
    @Override
    public String getName(){return "Bing Road";}
    @Override
    public String getExt(){return jpg;}
    @Override
    public Bitmap getIcon(){return null;}// MyI.bingIcon;}
    @Override
    public String getUrl(final int xx,final int yy,final int z){
        int x=xx;
        int y=yy;
        cnt++;
        if (z == 0)
            return new GoogleEarth().getUrl(x, y, z);
        String s = "";
        for (int i = 0; i < z; i++)
        {
            s = Integer.toString(((y & 1) * 2) + (x & 1)) + s;
            x >>= 1;
            y >>= 1;
        }
        //http://ecn.t3.tiles.virtualearth.net/tiles/r1123?g=784&mkt=en-us&lbl=l1&stl=h&shading=hill&n=z
        return "http://ecn.t" + Integer.toString(cnt&3) + ".tiles.virtualearth.net/tiles/r" + s + "?g=784&mkt=en-us&lbl=l1&stl=h&shading=hill&n=z";
    }
    @Override
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}


/////////////////////////////////////////////////////////////////////////////////////////
class TopoKrym implements INET_MAPS{
    public int cnt;
    public String getReferer(){return "http://mapim.com.ua/crimea/interesnye-mesta-v-krymu/map";}
    public String getName(){return "TopoKrym";}
    public String getExt(){return jpg;}
    public Bitmap getIcon(){return null;}// MyI.openStreetIcon;}
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        String s=Integer.toString((cnt&7)+1);
        return "http://gmap"+s+".mapim.com.ua/images/l"+Integer.toString(z)+"/"+Integer.toString(x)+"_"+Integer.toString(y)+"_"+Integer.toString(z)+jpg;
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}
/////////////////////////////////////////////////////////////////////////////////////////

//OutDoors
class OutDoors implements INET_MAPS{
    public int cnt;
    public String getReferer(){return null;}
    public String getName(){return "OutDoors";}
    public String getExt(){return png;}
    public Bitmap getIcon(){return null;}// MyI.openStreetIcon;}
    final char[]abca={'a','b','c','a'};
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        //http://b.tile.thunderforest.com/cycle/15/19421/11404.png
        return "http://"+abca[cnt&3]+".tile.thunderforest.com/cycle/"+Integer.toString(z)+"/"+Integer.toString(x)+"/"+Integer.toString(y)+png;
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}




class OpenCycleMap implements INET_MAPS{
    public int cnt;
    public String getReferer(){return null;}
    public String getName(){return "OpenCycleMap";}
    public String getExt(){return png;}
    public Bitmap getIcon(){return null;}// MyI.openStreetIcon;}
    final char[]abca={'a','b','c','a'};
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        //http://b.tile.opencyclemap.org/cycle/6/39/26.png
        return "http://"+abca[cnt&3]+".tile.opencyclemap.org/cycle/"+Integer.toString(z)+"/"+Integer.toString(x)+"/"+Integer.toString(y)+png;
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}
/////////////////////////////////////////////////////////////////////////////////////////
class OpenStreetMap implements INET_MAPS{
    public int cnt;
    public String getReferer(){return null;}
    public String getName(){return "OpenStreetMap";}
    public String getExt(){return png;}
    public Bitmap getIcon(){return null;}// MyI.openStreetIcon;}
    final char[]abca={'a','b','c','a'};
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        return "http://"+abca[cnt&3]+".tile.openstreetmap.org/"+Integer.toString(z)+"/"+Integer.toString(x)+"/"+Integer.toString(y)+png;
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}
/////////////////////////////////////////////////////////////////////////////////////////
class YahooMap implements INET_MAPS{
    public int cnt;
    public String getReferer(){return null;}
    public String getName(){return "YahooMap";}
    public String getExt(){return png;}
    public Bitmap getIcon(){return null;}// MyI.yahooIcon;}
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        int s;
        if (z>1){
            s=1<<(z-1);
            s--;
        }else
            s=y<<1;
        //http://maps2.yimg.com/hx/tl?b=1&v=4.3&.intl=en&x=3&y=1&z=5&r=1
        return "http://maps"+Integer.toString(1+(cnt&1))+".yimg.com/hx/tl?b=1&v=4.3&.intl=en&x="+Integer.toString(x)+"&y="+Integer.toString(s-y)+"&z="+Integer.toString(z+1)+"&r=1";
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}
/////////////////////////////////////////////////////////////////////////////////////////
class YahooSatelitte implements INET_MAPS{
    public int cnt;
    public String getReferer(){return null;}
    public String getName(){return "YahooSatelitte";}
    public String getExt(){return jpg;}
    public Bitmap getIcon(){return null;}//MyI.yahooIcon;}
    public String getUrl(final int x,final int y,final int z){
        cnt++;
        int s;
        if (z>1){
            s=1<<(z-1);
            s--;
        }else
            s=y<<1;
        //http://maps2.yimg.com/hx/tl?b=1&v=4.3&.intl=en&x=3&y=1&z=5&r=1
        return "http://maps"+Integer.toString(1+(cnt&1))+".yimg.com/ae/ximg?v=1.9&t=a&s=256&.intl=en&x="+Integer.toString(x)+"&y="+Integer.toString(s-y)+"&z="+Integer.toString(z+1)+"&r=1";
    }
    public void   incVersion(){}
    public void   loadVersion(){}
    public void   saveVersion(){}
    public void   OkVersion(){}
}

/////////////////////////////////////////////////////////////////////////////////////////




public class InetMaps {
    static final private GoogleMap 			gm=new GoogleMap();
    static final private GoogleEarth 		ge=new GoogleEarth();
    static final private GoogleTerrian 		gt=new GoogleTerrian();
    static final private MicrosoftEarthAndLabels 	bbe=new MicrosoftEarthAndLabels();
    static final private TopoKrym			tk=new TopoKrym();
    static final private OpenCycleMap		ocm=new OpenCycleMap();
    static final private HikeBike                   hb=new HikeBike();
    static final private OpenStreetMap		osm=new OpenStreetMap();
    static final private YahooMap			ym=new YahooMap();
    static final private YahooSatelitte		ys=new YahooSatelitte();
    static final private MicrosoftMap		mm=new MicrosoftMap();
    static final public INET_MAPS[] im={gm,ocm,hb,osm,mm,ym,ge,bbe,ys,gt,tk};

    static public  int type=1;
    public boolean pause=false;
    static public int getType(final String name){
        int ret=-1;
        for (int i=0; i<im.length; i++){
            if (im[i].getName().equals(name)){
                ret=i;
                break;
            }
        }
        return ret;
    }

    //public final Image []ICONS={MyI.googleIcon,MyI.googleIcon,MyI.openStreetIcon,MyI.openStreetIcon,MyI.openStreetIcon,MyI.bingIcon,MyI.googleIcon,MyI.yahooIcon};
//public final String []DIR={"GoogleMap","GoogleEarth","Topo_Krym","OpenCycleMap", "OpenStreetMap","Bing Bird's eye","GoogleTerian","Yahoo"};
//public final String []EXT={png,jpg,jpg,png,png,jpg,jpg,png};
/////////////////////////////////////////////////////////////////////////////////////////
    private int inetIn=0;
    private int loadErrors=0;
    private byte[] downLoadTile(int x, int y, int z){

        InputStream is = null;

        try{
            try {
                String url=im[type].getUrl(x, y, z);
                URL c = new URL(url);
                //  System.out.println(url);

                URLConnection myC = c.openConnection();
                myC.setRequestProperty( "User-Agent","Mozilla/5.0 (Windows NT 6.1; WOW64; rv:9.0.1) Gecko/20100101 Firefox/9.0.1" );
                if (im[type].getReferer()!=null)
                    myC.setRequestProperty("Referer",im[type].getReferer());
                myC.connect();
                is=myC.getInputStream();
                int lenF =(int)myC.getContentLength();
                int len=lenF;
                if (len>0){
                    im[type].OkVersion();
                    byte []buf=new byte[len];
                    int off=0,readed=0;
                    do{
                        readed=is.read(buf, off, len);
                        inetIn+=readed;
                        if (readed==len)
                            return buf;
                        len-=readed;
                        off+=readed;
                        //System.out.println("READED="+readed);
                    }while (readed>0);

                    // return Image.createImage(buf, 0, len);

                }

            }finally{
                loadErrors++;
                im[type].incVersion();

                if (is!=null)
                    is.close();

            }
        } catch (Exception ex) {
            System.out.println("DOWNLOAD ERROR");
            // Logger.getLogger(InetMaps.class.getName()).log(Level.SEVERE, null, ex);
        }

        return null;
    }

    static private final int MAX_FILES_IN_DIR_SH = 10;

    /////////////////////////////////////////////////////////////////////////////////////////
    static private final String extention=".bni";//for back yard navigator

    static private String getTileFName(final int type, final int x, final int y, final int z){
        File fc;
        try {
            try{
                String fn="mnt/sdcard/Tiles/";
                fc = new File(fn);
                if (! fc.exists())
                    fc.mkdir();

                fn+=im[type].getName()+"/";
                fc = new File(fn);
                if (! fc.exists())
                    fc.mkdir();

                fn+=Integer.toString(z)+"/";
                fc = new File(fn);
                if (! fc.exists())
                    fc.mkdir();

                fn+=Integer.toString(x)+"/";
                fc = new File(fn);
                if (! fc.exists())
                    fc.mkdir();

                fn+=Integer.toString(y)+extention;

                return fn;
            }finally{

            }
        } catch (Exception ex) {}
        return null;
    }

    /*
    static public String getTileFName__(final int type, final int x, final int y, final int z){
         File fc;
        try {
            try{
                String fn=MobNavJFrame.userHome+"/Tiles/";
                fc = new File(fn);
                if (! fc.exists())
                    fc.mkdir();

                fn+=im[type].getName()+"/";
                fc = new File(fn);
                if (! fc.exists())
                    fc.mkdir();

                fn+=Integer.toString(z)+"/";
                fc = new File(fn);
                if (! fc.exists())
                    fc.mkdir();

                fn+=Integer.toString(x>>MAX_FILES_IN_DIR_SH)+"_"+Integer.toString(y>>MAX_FILES_IN_DIR_SH)+"/";
                fc =  new File(fn);
                if (! fc.exists())
                    fc.mkdir();
                fc=null;
           return fn+=Integer.toString(z)+"_"+Integer.toString(x)+"_"+Integer.toString(y)+im[type].getExt();
            }finally{

            }
        } catch (Exception ex) {}

        return null;

    }
    */
    private String getTileFName(final int x, final int y, final int z){
        return getTileFName(type,x,y,z);
    }
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

    private final Object stMon=new Integer(0);
    private Stack st=new Stack();
    private boolean []downloaderRunning={false,false,false,false,false,false,false,false};


    /////////////////////////////////////////////////////////////////////////////////////////
    private int GetStSize(){
        synchronized(stMon){
            return st.size();
        }
    }
/////////////////////////////////////////////////////////////////////////////////////////

    private  UPD_MON hostUpdMon=null;
    private int errors=0;
    public InetMaps(final UPD_MON o){
        hostUpdMon=o;
    }
    private int thrRunning=0;
    private void update(){

        new Thread(new Runnable() {
            @Override
            public void run() {
                // MobNavDeskView
                synchronized(hostUpdMon){
                    hostUpdMon.notify();
                    System.out.println("I_UPDATE_");
                }
            }}).start();
    }


    private int cnt=0;
    private final int MAX_DOWNLOAD_ERRORS=3;
    boolean running=true;
    private void stop(){running=false;}
    private void StartDownloader(final int thr){
        thrRunning++;
        new Thread(new Runnable() {
            @Override
            public void run() {

                TILE tb=null;
                while (running){
                    while (pause)
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException ex) {}

                    synchronized(stMon){
                        int size=st.size();
                        if (size==0){
                            downloaderRunning[thr]=false;
                            System.out.println("dd "+thrRunning);
                            if (--thrRunning==0)
                                update();
                            return;
                        }else{
                            int i=0;
                            for (; i<size; i++){
                                if (((TILE)st.elementAt(i)).inProcess==false){
                                    ((TILE)st.elementAt(i)).inProcess=true;
                                    tb=new TILE((TILE)st.elementAt(i));
                                    break;
                                }
                            }if (tb==null){
                                downloaderRunning[thr]=false;
                                --thrRunning;
                                return;
                            }
                        }
                    }
                    File fc=null;
                    FileOutputStream out=null;
                    boolean removeFromST=true,  exist=false;

                    try{
                        fc=new File(tb.fn);
                        exist=fc.exists();

                        //  if (exist){
                        //  fc.close();
                        //  }
                    } catch (Exception ex) {}

                    if (exist==false &&   tb.z==hostUpdMon.zoom){
                        byte []b=downLoadTile(tb.x,tb.y,tb.z);
                        if (b!=null  ){
                            try {
                                try{
                                    out=new FileOutputStream(fc);
                                    out.write(b);
                                    out.close();
                                    out=null;
                                    cnt++;
                                    //System.out.println("OK "+tb.fn);
                                }finally{
                                    if (out!=null)
                                        out.close();
                                }
                                // System.out.println("Thread="+thr+" Write "+tb.fn);
                            } catch (Exception ex) {
                                removeFromST=true;
                                System.out.println("Thread="+thr+" ERROR "+tb.fn+ex.toString());
                            }
                        }else{
                            // System.out.println("ERROR "+tb.fn);
                            removeFromST=false;
                        }

                    }
                    synchronized(stMon){
                        try{
                            // int size=st.size();
                            for (int i=0; i<st.size(); i++)
                                if ( tb.fn.endsWith(((TILE)st.elementAt(i)).fn) ){
                                    if (removeFromST){
                                        st.removeElementAt(i);
                                        break;
                                    }else{
                                        if ( ++((TILE)st.elementAt(i)).error>MAX_DOWNLOAD_ERRORS    ){
                                            st.removeElementAt(i);
                                            //MyCanvas.SetErrorText("No TIle");
                                            errors++;
                                            break;
                                        }else{
                                            ((TILE)st.elementAt(i)).inProcess=false;
                                            System.out.println("ERROR "+tb.fn+" # "+((TILE)st.elementAt(i)).error);
                                            //removeFromST=true;
                                        }
                                    }
                                }
                        } catch (Exception ex) {
                            //MyCanvas.SetErrorText("No TIle+");
                        }
                    }
                }

            }}).start();
    }
    /////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//public Image out_of_map=null;
    private boolean tileExistOrLoad(final int x, int y, final int z){

        File fc;
        final String fn=getTileFName(x,y,z);
        boolean exist;
        try{
            fc=new File(fn);
            exist=fc.exists();

        } catch (Exception ex) {exist=false;System.out.println("TILE EXIST"+ex.toString());}

        if (exist)
            return true;
        else
            loadTile(x,y,z,fn);
        return false;
    }
    public MyTile getTile(final int x, int y, final int z){
        if (x<0 || x>=(1<<z) || y<0 || y>=(1<<z)){
            MyTile mt=new MyTile();
            // mt.img=new BufferedImage(256 ,256,BufferedImage.TYPE_INT_RGB);
            mt.img=Map.blank;

            mt.size=1000;
            return mt;
        }
        File fc=null;
        final String fn=getTileFName(x,y,z);
        boolean exist;
        try{
            fc=new File(fn);
            exist=fc.exists();

        } catch (Exception ex) {exist=false;System.out.println("TILE EXIST"+ex.toString());}

        if (exist)
            return getTile(fc);
        else
            loadTile(x,y,z,fn);
        return new MyTile();
    }
    // ------------------------------------------------------------------------
    private MyTile getTile(final File fc){
        InputStream in;
        try{
            MyTile t=new MyTile();
            in=new FileInputStream(fc);
            byte []b=new byte[in.available()];
            t.size=in.read(b);
            in = new ByteArrayInputStream(b);






            t.img=BitmapFactory.decodeStream(in);

/*

            t.img=Bitmap.createBitmap(256,256,Bitmap.Config.ARGB_8888);
            t.img = BitmapFactory.decodeStream(in).copy(Bitmap.Config.ARGB_8888, true);
            try {
                for (int y = 0; y < t.img.getHeight(); y++)
                   // int y=0;
                    for (int x = 0; x < t.img.getWidth(); x++)
                        t.img.setPixel(x, y, t.img.getPixel(x,y)^0xffffff);
                // t.img = ImageIO.read(in);
            }catch (Exception ex){}

  */
            return t;
        }catch (Exception ex){System.out.println("Load Tile Error "+ex.toString());return null;}
    }
    //---------------------------------------------------------------------------
    private void loadTile(final int x, int y, final int z, String fn){
        synchronized(stMon){
            int i,size=st.size();
            final TILE nt=new TILE(x,y,z,0,fn);
            for (i=0; i<size; i++)
                if (((TILE)st.elementAt(i)).fn.endsWith(fn)){
                    break;
                }
            if (i==size)
                st.push(nt);

            for (int threadN=0; threadN<downloaderRunning.length; threadN++)
                if (downloaderRunning[threadN]==false){
                    downloaderRunning[threadN]=true;
                    StartDownloader(threadN);
                    break;
                }
        }

    }
/////////////////////////////////////////////////////////////////////////////////////////



}
