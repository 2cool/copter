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



    private float old_y;

    public  Camera_pitch_cntr(){

    }


   // Commander.fpv_zoom

    private int buf2send=0;
    public float gimbal_pitch_add(float dy,float fpv_zoom){
        final int pixel2angle=11;
        final double zoom=Math.max(1,Math.min(101,fpv_zoom))-1;
        final double _pixel2angle=pixel2angle*((zoom/10)+1);
      //  Log.d("ZOOMZ",Double.toString(_pixel2angle));
        final int d_ang=(int)(dy/_pixel2angle);
        buf2send+=d_ang;

        if (buf2send>0){
            MainActivity.camera_gimb_minus();
            buf2send--;
        }else if (buf2send<0){
            MainActivity.camera_gimb_plus();
            buf2send++;
        }
        return (float)_pixel2angle*d_ang;
    }


    public void reset(){
        camera_pitch_index=-1;
    }
    public void onTouchEvent(MotionEvent event, float fpv_zoom) {
        int actionMask = event.getActionMasked();
        int index = event.getActionIndex();
       // final float gx = event.getX(index);
        final float gy = event.getY(index);

        switch (actionMask) {
            case MotionEvent.ACTION_DOWN: // первое касание
            case MotionEvent.ACTION_POINTER_DOWN: // последующие касания
            {
                if (camera_pitch_index==-1 ){
                    camera_pitch_index=index;
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

                    old_y+=gimbal_pitch_add(gy-old_y, fpv_zoom);

                }
                break;
        }

    }
}
