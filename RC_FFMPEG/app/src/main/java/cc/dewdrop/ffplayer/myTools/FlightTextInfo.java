package cc.dewdrop.ffplayer.myTools;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;

public class FlightTextInfo {

    public static final int LOC=0,_2HM=1,THR=2,CUR=3,VIBR=4,BAT=5,YAW=6,CAM_ANG=7,CAM_ZOOM=8,VSPEED=9,M_ON_T=10, MESGE=11;
    final private String name[]={"","2HM: ","THR: ","CUR: ","VBR: ","BAT: ","YAW:","CAM: ","ZOOM: ","VSP: ","TIM:",  "MSG:"};
    final private String unt[]={""," m",""," mAh",""," v"," ang"," ang",""," m/s"," s",  ""};

    Rect r;
    final int FIELDS=12;
    private Paint[] paint = new Paint[FIELDS];
    public boolean f[]=new boolean[FIELDS];
    public String p[]={"","","","","","","","","","","",""};
    public void setPaint(int n, int c){
        paint[n].setColor(c);
    }
    public FlightTextInfo(Rect r,
                          boolean loc,
                          boolean _2hom,
                          boolean thr,
                          boolean current,
                          boolean vibr,
                          boolean bat,
                          boolean yaw,
                          boolean cam_ang,
                          boolean cam_zoom,
                          boolean vSpeed,
                          boolean motors_on_time,
                          int txt_color){
        this.r=r;
        f[LOC]=loc;
        f[_2HM]=_2hom;
        f[THR]=thr;
        f[CUR]=current;
        f[VIBR]=vibr;
        f[BAT]=bat;
        f[YAW]=yaw;
        f[CAM_ANG]=cam_ang;
        f[CAM_ZOOM]=cam_zoom;
        f[VSPEED]=vSpeed;
        f[M_ON_T]=motors_on_time;
        f[MESGE]=true;

        Paint txt=new Paint();
        txt.setColor(txt_color);
        final int fsx=(int)(0.7*(r.right-r.left));
        final int fsy=(int)(0.7*(r.bottom-r.top));
        final String str="0000000  000000000";
        Rect b=new Rect();
        for (int s=15; s<300;s++) {
            txt.setTextSize(s);
            txt.getTextBounds(str,0,str.length(),b);
            if (fsx<=(b.right-b.left) || fsy<=(b.bottom-b.top)*FIELDS*1.5) {
                break;
            }
        }
        txt.setTextSize(25);
        Paint txt2=new Paint(txt);
        txt2.setColor(Color.GREEN);
        for (int i=0; i<FIELDS; i++) {
            if (i==1)
                paint[i] = new Paint(txt2);
            else
                paint[i]=new Paint(txt);
           // paint[i].setColor(txt_color);
        }

    }
    public void paint(Canvas c){

        Rect b=new Rect();
        paint[0].getTextBounds("Text",0,4,b);
        int h=(int)(1.5*(b.bottom-b.top));
        int x=r.left;
        int y=r.top+h;
        for (int i=0; i<FIELDS;i++ ){
            if (f[i]){
                c.drawText(name[i]+p[i]+unt[i],x,y,paint[i]);
                y+=h;
            }
        }
    }
}
