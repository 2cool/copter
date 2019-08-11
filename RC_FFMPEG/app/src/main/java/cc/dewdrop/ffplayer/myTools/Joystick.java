package cc.dewdrop.ffplayer.myTools;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.view.MotionEvent;

public class Joystick {
    private float def_jx,def_jy;
    private Paint white;
    private boolean return_backX,return_backY, block_X, block_Y;
    private float x,y,size;
    private String label="";
    private float trackX,trackY,old_posX,old_posY,shiftX,shiftY;
    private float jx,jy;
    private int index;
    public float getX(){return jx;}
    public float get_neg_Y(){return -jy;}
    public void setLabel(String s){label=s;}
    public float setJosticX(float x){

        jx=(block_X)?0: Math.min(1,Math.max(-1,x));

        return jx;
    }
    public float setJosticY(float y)
    {
        jy=(block_Y)?0: Math.min(1,Math.max(-1,y));
        old_posY=this.y+(y+1)*size*0.5f;
        return jy;
    }

    private boolean setJX(float xp){
        float t_jx=(xp-x - size*0.5f)/(size*0.5f);
        if (t_jx<=1 && t_jx>=-1){
            jx=t_jx;

            return true;
        }else
            return false;
    }
    private boolean setJY(float yp){
        float t_jy=(yp-y-size*0.5f)/(size*0.5f);
        if (t_jy<=1 && t_jy>=-1){
            jy= t_jy;
            return true;
        }else
            return false;
    }
    private void end(){
        index=-1;
        if (return_backX) {
            jx=0;
            old_posX=x+size*0.5f;
        }else
            old_posX-=shiftX;
        if (return_backY) {
            jy=0;
            old_posY=y+size*0.5f;
        }else
            old_posY-=shiftY;


        shiftX=shiftY=0;

    }
    private int findPointerIndex(MotionEvent event){
        float min=10e24f;
        int index=-1;
        for (int i=0; i< event.getPointerCount(); i++){
            final float px = event.getX(i);
            final float py = event.getY(i);
            final float dx=px-trackX;
            final float dy=py-trackY;
            final float dist2=dx*dx+dy*dy;
            if (dist2<min){
                index=i;
                min=dist2;
            }
        }
        if (min>10000)
            index=-1;
        return index;

    }


    public boolean onTouchEvent(MotionEvent event) {
        boolean ret=false;
        // событие
        int actionMask = event.getActionMasked();
        // индекс касания
        int pointerIndex = event.getActionIndex();
        // число касаний
        //  int pointerCount = event.getPointerCount();
        switch (actionMask) {
            case MotionEvent.ACTION_DOWN: // первое касание
            case MotionEvent.ACTION_POINTER_DOWN: // последующие касания
            {
                final float gx = event.getX(pointerIndex);
                final float gy = event.getY(pointerIndex);
                if (index<=0 && gx >= x && gx <= x + size && gy >= y && gy <= y + size) {
                    end();
                    index=pointerIndex;
                    trackX=gx;
                    trackY=gy;
                    if (!block_X){
                        shiftX = gx - old_posX;//??
                        old_posX=gx;
                    }
                    if (!block_Y) {
                        shiftY = gy - old_posY;
                        old_posY = gy;
                    }
                    ret=true;
                }
                break;
            }
            case MotionEvent.ACTION_UP: // прерывание последнего касания
                end();
                break;
            case MotionEvent.ACTION_POINTER_UP: // прерывания касаний
                if (index==event.getActionIndex()){
                    end();
                    ret=true;
                }
                break;
            case MotionEvent.ACTION_MOVE: // движение
                if (index>=0)
                    index=findPointerIndex(event);
                if (index >= 0) {
                    trackX = event.getX(index);
                    trackY = event.getY(index);
                    if (!block_X) {

                        if (setJX(trackX - shiftX)) {
                            old_posX = trackX;
                        }
                    }


                   if (!block_Y) {

                       if (setJY(trackY - shiftY))
                           old_posY = trackY;
                   }

                }else{
                    end();
                }
                break;
        }
        if (block_X){
            jx=def_jx;

        }
        if (block_Y){
            jy=def_jy;

        }

        //  Log.d("JOSTIC",Float.toString(jx)+" , "+Float.toString(jy));
        return ret;

    }

    Paint cc;

    public void set_return_back_X(boolean b){
        if (b!=return_backX) {
            return_backX = b;
            if (return_backX && index == -1)
                end();
        }
    }
    public void set_return_back_Y(boolean b){
        if (b!=return_backY) {
            return_backY = b;
            if (return_backY && index == -1)
                end();
        }
    }
    public void set_block_X(boolean b){
        block_X=b;
        def_jx=jx;
    }
    public void set_block_Y(boolean b){
        block_Y=b;
        def_jy=jy;
    }
    public Joystick(
            float _x,
            float _y,
            float _size,
            boolean return_back_X,
            boolean return_back_Y,
            boolean blockX,
            boolean blockY,
            Paint c)
    {
        x=_x;
        y=_y;
        size=_size;
        return_backX =return_back_X;
        return_backY=return_back_Y;
        block_X=blockX;
        block_Y=blockY;
        white=new Paint(c);
        white.setStyle(Paint.Style.STROKE);
        white.setTextSize(20);
        cc=new Paint(c);
        cc.setStrokeWidth(1);
        index=-1;
        end();
        jx = jy = 0;
        shiftY=shiftX=0;
        old_posX=x+size*0.5f;
        old_posY=y+size*0.5f;


    }

    public void paint(Canvas c) {


        //sensor emulator
        c.drawCircle( x+(size+jx*size)*0.5f, y+(size+jy*size)*0.5f , size/10, white);
        if (label!=null && label.length()>0) {
            Rect r=new Rect();
            white.getTextBounds(label,0,label.length(),r);
            c.drawText(label, x + (size+jx*size-r.right+r.left) * 0.5f, y + (size+jy*size+r.bottom-r.top) * 0.5f, white);
        }

        c.drawCircle(x+size*0.5f, y+size*0.5f , size*0.5f, white);


    }

}
