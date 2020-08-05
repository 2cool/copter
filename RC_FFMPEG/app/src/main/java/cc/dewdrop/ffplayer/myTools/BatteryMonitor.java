package cc.dewdrop.ffplayer.myTools;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

public class BatteryMonitor {


    float x,  y,  sizeX,  sizeY;
    float border;
    Paint white,green;
    float charge;
    public BatteryMonitor(float x_, float y_, float size){
        x=x_;
        y=y_;
        sizeX=size;
        border=size*0.02f;
        sizeY=size*0.2f;
        white=new Paint();
        white.setColor(0xff007700);
        white.setStyle(Paint.Style.STROKE);
        green=new Paint();
        green.setColor(0xff007700);

        charge=(sizeX-border-border)*1;
    }
    public void setVoltage(float volt){
        /*
        long ttt=System.currentTimeMillis();
        ttt/=30;
        ttt&=511;
        ttt=511-ttt;
        volt=ttt;
*/

        if (volt>422)
            volt=422;
        if (volt<300)
            volt=300;
        float bat = (volt-300)/122;
        if (bat>1)
            bat=1;
        if (bat>=0.5){
            green.setColor(Color.rgb((int)(255*(2-bat*2)),255,0));
        }else{
            green.setColor(Color.rgb(255,(int)(255*(bat*2)),0));
        }


       /* if (bat>0.5)
            green.setColor(Color.GREEN);
        else if (bat>0.3)
            green.setColor(Color.rgb(200,200,0));
        else
            green.setColor(Color.RED);
*/

        charge=(sizeX-border-border)*bat;
        green.setAlpha(125);
    }

    public void paint(Canvas c){
        c.drawRect(x,y,(x+sizeX),(y+sizeY),white);
        float l=(sizeX-border-border);
        c.drawRect(x+border,y+border,x+charge,y+sizeY-border,green);

    }
}
