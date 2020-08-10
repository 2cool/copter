package cc.dewdrop.ffplayer.myTools;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.Log;

import cc.dewdrop.ffplayer.DrawView;
import cc.dewdrop.ffplayer.MainActivity;
import cc.dewdrop.ffplayer.Telemetry;

import static java.lang.Math.cos;

public class Monitor {
    Paint white,green,red,black;
    private float compsL;
    private float hight_t=0,hight,speed_t=0,speed;
    private double pitch, roll, yaw;
    private float xpos,ypos;
    private Bitmap bm,cmps;
    public void setPitch(double a){
        pitch=a;
    }
    public void setRoll(double a){
        roll=-a;
    }
    public void setHeight(double h){
     //   hight_t+=(h*10-hight_t)*0.05;
        hight = (float) h;//(float) (Math.round(hight_t)*0.1);

    }
    public void setSpeed(double s){
        speed_t+=(s*10-speed_t)*0.05;
        speed=(float)(Math.round(speed_t)*0.1);

    }
    public void setYaw(double a){
        while(a<0)
            a+=360;
        while (a>=360)
            a-=360;
        yaw=a;
    }
    private float scale;
    private float size;
    Paint gray;

    public Monitor(int x, int y, int size, Bitmap bm, Bitmap cmps){

        hight=0;
        yaw=0;
        speed=0;
        black=new Paint();
        black.setColor(0xff000000);
        gray=new Paint();
        gray.setColor(Color.BLACK);
        gray.setAlpha(35);
        this.size=size;
        // float sm[]=MainActivity.screenMetrics;
        scale=(float)size / bm.getWidth();
        xpos=x;
        ypos=y;
        pitch=roll=0;
        this.bm=bm;
        this.cmps=cmps;
        white = new Paint();
        white.setColor(Color.WHITE);
        white.setStrokeWidth(2);
        white.setTextSize(size/6);
        //   white.setAlpha(255);

        green=new Paint();
        green.setColor(Color.GREEN);
        green.setStrokeWidth(3);

        red=new Paint();
        red.setColor(Color.RED);
        red.setStrokeWidth(3);


        compsL= (float)cmps.getWidth()*9.0f/25.0f;
    }



    //  static float angle=0;//115;
    private Matrix matrix = new Matrix();
    public void paint(Canvas c) {
        //draw monitoring of pitch and yaw;
        c.drawRect(xpos - size * 0.7f, ypos - size, xpos + size * 0.7f, ypos + size, gray);
        matrix.reset();
        matrix.postRotate((float) roll);
        float k = (float) bm.getWidth() / 185.0f;
        Bitmap cropped = Bitmap.createBitmap(
                bm,
                0,
                (int) (307 * k + 2.4 * k * pitch),
                bm.getWidth(),
                bm.getWidth(),
                matrix,
                true);
        matrix.reset();
        matrix.postScale(scale, scale);
        Bitmap cropped2 = Bitmap.createBitmap(
                cropped,
                (cropped.getHeight() - bm.getWidth()) / 2,
                (cropped.getWidth() - bm.getWidth()) / 2, bm.getWidth(),
                bm.getWidth(),
                matrix,
                true);
        c.drawBitmap(cropped2, xpos - cropped2.getWidth() / 2, ypos - cropped2.getHeight() / 2, white);


        //draw central horizontal line of pitch
        c.drawLine(xpos - 50, ypos, xpos + 50, ypos, white);
        c.drawLine(xpos, ypos - 15, xpos, ypos + 15, white);


        //draw division of roll
        for (float a = -60; a <= 60; a += 20) {
            float ang = (float) (a / 180 * Math.PI);
            float size1 = size * 0.65f;
            float x0 = (float) Math.sin(ang) * size1;
            float y0 = -(float) Math.cos(ang) * size1;
            c.drawLine(xpos + x0, ypos + y0, xpos + x0 * 1.1f, ypos + y0 * 1.1f, white);
        }
        //draw pointer of roll
        float size1 = size * 0.6f;
        float x0 = (float) Math.sin(roll / 180 * Math.PI) * size1;
        float y0 = -(float) Math.cos(roll / 180 * Math.PI) * size1;
        c.drawLine(xpos + x0, ypos + y0, xpos + x0 * 1.2f, ypos + y0 * 1.2f, white);
        //draw height
        String text = Float.toString(hight) + " m";
        Rect r = new Rect();
        white.getTextBounds(text, 0, text.length(), r);
        //draw speed
        float  xx = xpos - bm.getWidth() / 2.7f - (r.right - r.left);
        float yy = ypos + (r.bottom - r.top) / 2;
        float w = white.measureText(text);
        c.drawRect(xx ,   yy,xx+w,yy-white.getTextSize(),black);
        c.drawText(text, xx, yy, white);

        text = Float.toString(speed) + " m/c";
        xx = xpos + bm.getWidth() / 3;
        yy = ypos + (r.bottom - r.top) / 2;
        w = white.measureText(text);
        c.drawRect(xx ,   yy,xx+w,yy-white.getTextSize(),black);
        c.drawText(text, xx, yy, white);
        //draw pointer of yaw
        c.drawLine(xpos, ypos - size1, xpos, ypos - size1 * 1.25f, white);


        //draw yaw
        matrix.reset();
        matrix.postScale(scale, scale);
        Bitmap compsN = Bitmap.createBitmap(
                cmps,
                (int) (16.0 / 360 / 25 * yaw * cmps.getWidth()),
                0,
                (int) (compsL),
                cmps.getHeight(),
                matrix,
                true);
        c.drawBitmap(compsN, xpos - compsL * scale * 0.5f, ypos - size1 * 1.5f, white);
        //phone direction
        green.setAlpha((DrawView.head_less_.is_pressed()) ? 255 : 100);
        //----------------------------------------------------------------------------

        float dy = (float) Math.max(-90, Math.min(90, DrawView.wrap_180(MainActivity.yaw - DrawView.wrap_180(yaw))));
        //Log.d("MONIT",Float.toString(dy)+" "+Double.toString(MainActivity.yaw)+ " "+Double.toString(yaw));
        w = -(scale * compsL);
        float d_yaw = w * dy / 180;
        d_yaw += w * 0.5;
        float x = xpos - compsL * scale * 0.5f - d_yaw;
        float y = ypos - size1 * 1.5f + cmps.getHeight() * 0.5f;
        c.drawLine(x, y, x, y + cmps.getHeight() * 0.5f, green);

        //----------------------------------------------------------------------------
        dy = (float) Math.max(-90, Math.min(90, DrawView.wrap_180(Telemetry.direction + MainActivity.yaw)));
     //   Log.d("MONIT","dir "+Double.toString(Telemetry.direction)+" telefon_yaw "+Double.toString(MainActivity.yaw));
        //final float w=-(scale*compsL);
        d_yaw = w * dy / 180;
        d_yaw += w * 0.5;
        x = xpos - compsL * scale * 0.5f - d_yaw;
        y = ypos - size1 * 1.5f + cmps.getHeight() * 0.5f;
        c.drawLine(x, y, x, y + cmps.getHeight() * 0.5f, red);


    }


}
