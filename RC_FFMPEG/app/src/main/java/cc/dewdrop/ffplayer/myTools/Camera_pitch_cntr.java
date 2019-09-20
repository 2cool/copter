package cc.dewdrop.ffplayer.myTools;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.Log;
import android.view.MotionEvent;

import cc.dewdrop.ffplayer.MainActivity;

public class Camera_pitch_cntr {
    private int camera_pitch_index=-1;



    private float old_x,old_y;

    public  Camera_pitch_cntr(){

    }


   // Commander.fpv_zoom

    private int buf2send=0;
    public float gimbal_pitch_add(float dx, float dy,float fpv_zoom){
        if (Math.abs(dy)>= Math.abs(dx)) {
            final int pixel2angle = 11;
            final double zoom = Math.max(1, Math.min(101, fpv_zoom)) - 1;
            final double _pixel2angle = pixel2angle * ((zoom / 10) + 1);
            //  Log.d("ZOOMZ",Double.toString(_pixel2angle));
            final int d_ang = (int) (dy / _pixel2angle);
            buf2send += d_ang;

            if (buf2send > 0) {
                MainActivity.camera_gimb_pitch_minus();
                buf2send--;
            } else if (buf2send < 0) {
                MainActivity.camera_gimb_pitch_plus();
                buf2send++;
            }
            return (float) _pixel2angle * d_ang;
        }else{
            if (dx>0)
                MainActivity.camera_gimb_roll_minus();
            else
                MainActivity.camera_gimb_roll_plus();
            return 0;
        }
    }


    public void reset(){
        camera_pitch_index=-1;
    }
    public void onTouchEvent(MotionEvent event, float fpv_zoom) {
        int actionMask = event.getActionMasked();
        int index = event.getActionIndex();
       // final float gx = event.getX(index);
        final float gy = event.getY(index);
        final float gx=event.getX(index);
        switch (actionMask) {
            case MotionEvent.ACTION_DOWN: // первое касание
            case MotionEvent.ACTION_POINTER_DOWN: // последующие касания
            {
                if (camera_pitch_index==-1 ){
                    camera_pitch_index=index;
                    old_x=gx;
                    old_y=gy;
                }
                break;
            }
            case MotionEvent.ACTION_UP: // прерывание последнего касания
            case MotionEvent.ACTION_POINTER_UP: // прерывания касаний
                if (camera_pitch_index>=0)
                    camera_pitch_index=-1;
                break;
            case MotionEvent.ACTION_MOVE: // движение
                if (camera_pitch_index==index){
                    if (Math.abs(gy-old_y)>=Math.abs(gx-old_x)) {
                        old_y += gimbal_pitch_add(0, gy - old_y, fpv_zoom);
                        old_x = gx;
                    }else{
                        gimbal_pitch_add(gx-old_x, 0, fpv_zoom);
                        old_x = gx;
                        old_y = gy;
                    }
                }
                break;
        }

    }
}
