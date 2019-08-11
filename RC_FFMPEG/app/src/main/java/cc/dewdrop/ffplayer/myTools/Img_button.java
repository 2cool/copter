package cc.dewdrop.ffplayer.myTools;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.drawable.Drawable;
import android.util.Log;
import android.view.MotionEvent;

public class Img_button {
    private Drawable imageOn,imageOff;
    private boolean pressed;
    private boolean toggle;
    private int status;
    private boolean enabled=true;
    private long time=0,last_time=0;
    private Paint white;
    public boolean is_pressed(){

        return pressed;
    }
    public int getStat(){
        int stat=status;
        if (status==3)
            status=0;
        return stat;
    }
    public void set(boolean b){
        status=0;
        pressed=b;
    }
    Rect r;
    private int id;
    public void enabled(boolean b){
        enabled=b;
        if (b==false) {
            status=0;
            imageOn.setAlpha(60);
            imageOff.setAlpha(60);
        }else{
            imageOn.setAlpha( 255);
            imageOff.setAlpha(255);
        }
    }
    public Img_button(Rect r_, Drawable imgOff_, Drawable imgOn_, boolean toggle_){

        white=new Paint();
        white.setColor(Color.GRAY);
        white.setAlpha(100);
        imageOn= imgOn_.mutate().getConstantState().newDrawable();
        imageOff= imgOff_.mutate().getConstantState().newDrawable();

        r=r_;
        imageOn.setBounds(r);
        imageOff.setBounds(r);

        toggle=toggle_;
        pressed=false;
        id=-1;

    }

    public boolean onTouchEvent(MotionEvent event) {



        if (enabled==false || System.currentTimeMillis()-last_time<300)
            return true;

        boolean ret=false;
        // событие
        int actionMask = event.getActionMasked();
        // индекс касания
        int pointerIndex = event.getActionIndex();
        // число касаний
        //  int pointerCount = event.getPointerCount();

        //  index=event.findPointerIndex(pointerIndex);
        final float gx = event.getX(pointerIndex);
        final float gy = event.getY(pointerIndex);
        switch (actionMask) {
            case MotionEvent.ACTION_DOWN: // первое касание
            case MotionEvent.ACTION_POINTER_DOWN: // последующие касания
            {

                if  (id<0){
                    if ( gx>=r.left && gx<=r.right && gy>=r.top && gy<=r.bottom) {
                        time=System.currentTimeMillis();
                        status=1;
                        id= event.getPointerId(pointerIndex);
                        ret=true;
                        Log.d("IMBNT","down");
                    }


                }
                break;
            }
            case MotionEvent.ACTION_UP: // прерывание последнего касания
            case MotionEvent.ACTION_POINTER_UP: // прерывания касаний


                if (id==event.getPointerId(pointerIndex)) {
                    if (gx >= r.left && gx <= r.right && gy >= r.top && gy <= r.bottom) {
                        last_time=System.currentTimeMillis();
                        if (toggle)
                            pressed^=true;
                        id=-1;
                        status=3;
                        ret=true;
                        Log.d("BUTTON","up");
                    }else
                        status=0;

                }
                break;
            case MotionEvent.ACTION_MOVE: // движение

                if (id==event.getPointerId(pointerIndex)) {
                    if (!(gx >= r.left && gx <= r.right && gy >= r.top && gy <= r.bottom)) {
                        id=-1;
                        status=0;
                        ret=true;
                    }else
                        status=2;

                }
                break;
        }
        return ret;

    }

    public void paint(Canvas c) {
        if (time>0 && System.currentTimeMillis()-time>500){
            time=0;
            status=0;
            id=-1;

        }


        (pressed?imageOn:imageOff).draw(c);
        if (status>0)
            c.drawCircle(r.left+(r.right-r.left)/2,r.top+(r.bottom-r.top)/2,(r.bottom-r.top)/2,white);

    }
}
