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
        white.setColor(Color.WHITE);
        white.setStyle(Paint.Style.STROKE);
        green=new Paint();
        green.setColor(Color.GREEN);

        charge=(sizeX-border-border)*1;
    }
    public void setVoltage(float volt){

        if (volt>422)
            volt=422;
        if (volt<300)
            volt=300;
        float bat = (volt-300)/122;
        charge=(sizeX-border-border)*bat;
        if (bat>0.5)
            green.setColor(Color.GREEN);
        else if (bat>0.3)
            green.setColor(Color.rgb(200,200,0));
        else
            green.setColor(Color.RED);
        green.setAlpha(89);
    }

    public void paint(Canvas c){
        c.drawRect(x,y,(x+sizeX),(y+sizeY),white);
        c.drawRect(x+border,y+border,x+charge,y+sizeY-border,green);
    }
}
