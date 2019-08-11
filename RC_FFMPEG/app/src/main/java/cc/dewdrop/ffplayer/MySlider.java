package cc.dewdrop.ffplayer;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;
import android.view.MotionEvent;

/**
 * Created by igor on 8/21/2016.
 */
public class MySlider {
    private Paint red= new Paint(Paint.ANTI_ALIAS_FLAG);
    private Paint redAlpha= new Paint(Paint.ANTI_ALIAS_FLAG);

    private int x, y,len;
    private double pos;
    double getPos(){return pos;}
    public MySlider(final int x, final int y,final int len, final double pos){
        this.x=x;
        this.y=y;
        this.pos=pos;
        this.len=len;
        red.setColor(Color.RED);
        redAlpha.setColor(Color.RED);
        redAlpha.setAlpha(100);
        red.setTextSize(20);



    }
    private boolean sliderON=false;
    public boolean onTouchEvent(MotionEvent event) {
        int pIndex = event.getActionIndex();
        //p=new Point[event.getPointerCount()];
        // get pointer ID
        int pointerId = event.getPointerId(pIndex);
        // get masked (not specific to a pointer) action
        int maskedAction = event.getActionMasked();


        int px=  (int)event.getX(pIndex);
        int py= (int)event.getY(pIndex);

        boolean onSlider= ((x-50)<=px && (x+len+50)>=px && (y-50)<=py && (y+50)>=py);


        switch (maskedAction) {
            case MotionEvent.ACTION_DOWN:
            case MotionEvent.ACTION_POINTER_DOWN:
                sliderON=onSlider;
                if (onSlider) {
                    if (px < x)
                        px = x;
                    if (px > x + len)
                        px = x + len;
                    pos = (double) (px - x) / (double) len;

                    return true;
                }

            case MotionEvent.ACTION_MOVE:
                if (sliderON){
                    if (px<x)
                        px=x;
                    if (px>x+len)
                        px=x+len;
                    pos = (double) (px-x) / (double) len;

                    return true;
                }
                break;
            default:

                sliderON=false;
        }




        return false;
    }



    public boolean clicked(final float cx, final float cy){
        int dx=Math.abs(x-(int)cx);
        int dy=Math.abs(y-(int)cy);
        int r2=dx*dx+dy*dy;
        return false;//r2<=r*r;
    }
    public void draw(final Canvas c){

        c.drawRect(x,y-2,x+(int)(pos*(double)len),y+4,red);
        c.drawLine(x,y,x+len,y,red);
        // Log.i("MAP","pos"+pos);
        c.drawCircle(x+(int)(pos*(double)len),y,10,red);
        c.drawCircle(x+(int)(pos*(double)len),y,40,redAlpha);
        if (sliderON)
            c.drawCircle(x+(int)(pos*(double)len),y,50,redAlpha);
    }



}
