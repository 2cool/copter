package cc.dewdrop.ffplayer;


/**
 * Created by igor on 8/19/2016.
 */


import android.annotation.TargetApi;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;
import android.os.Build;
import android.support.constraint.solver.widgets.Rectangle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import cc.dewdrop.ffplayer.myTools.Img_button;
import cc.dewdrop.ffplayer.myTools.Square_Cells;

public class DrawMap extends View {

    public static ScaleGestureDetector mScaleGestureDetector;
    static public float sm[];
    static public Point screenP=new Point();
    static public int zoom=1;
    static public int type=7;
    static Img_button ib_zoom_out,ib_zoom_in,ib_menu,ib_add,ib_edit,ib_upload,ib_del;
    static MySlider sDirection;
    static public UPD_MON monitor=new UPD_MON();
    static public InetMaps im=new InetMaps(monitor);
    public MyTile myile;
    Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);

    Paint black = new Paint();
    int textX;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public DrawMap(Context context) {
        super(context);

       sm=Map.screenMetrics;
       Square_Cells sc=new Square_Cells(0,10,0.3f,sm); //scree rotated
        int nX=sc.getMaxX();
        int nY=sc.getMaxY();
        textX=sc.getRectSize();
        ib_zoom_out=new Img_button(sc.getRect(0,3),
                context.getResources().getDrawable(R.drawable.minus),
                context.getResources().getDrawable(R.drawable.minus),false);

        ib_zoom_in=new Img_button(sc.getRect(0,4),
                context.getResources().getDrawable(R.drawable.plus),
                context.getResources().getDrawable(R.drawable.plus),false);
        ib_menu=new Img_button(sc.getRect(nX-1,1),
                context.getResources().getDrawable(R.drawable.menu),
                context.getResources().getDrawable(R.drawable.menu),false);

        ib_add=new Img_button(sc.getRect(nX-1,3),
                context.getResources().getDrawable(R.drawable.add_mark),
                context.getResources().getDrawable(R.drawable.add_mark),false);
        ib_edit=new Img_button(sc.getRect(nX-1,4),
                context.getResources().getDrawable(R.drawable.edit),
                context.getResources().getDrawable(R.drawable.edit),false);
        ib_upload=new Img_button(sc.getRect(nX-1,2),
                context.getResources().getDrawable(R.drawable.upload),
                context.getResources().getDrawable(R.drawable.upload),false);
        ib_del=new Img_button(sc.getRect(nX-1,6),
                context.getResources().getDrawable(R.drawable.delite),
                context.getResources().getDrawable(R.drawable.delite),false);
    }
    private int oldtx=0, oldty=0;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private void dragImgM(final Point c){
        int tx=c.x>>8;
        int ty=c.y>>8;
        int iix=-(oldtx-tx);
        int iiy=-(oldty-ty);
        //ix=-ix;
        //iy=-iy;
        if (iix!=0 || iiy!=0){
            try{
                int sx=imgM.length;
                int sy=imgM[0].length;
                MyTile [][]timg=new MyTile[sx][sy];
                for (int y=0,iy=iiy; y<sy; y++,iy++)
                    for (int x=0,ix=iix; x<sx; x++,ix++){
                        if (ix>=0 && ix<sx && iy>=0 && iy<sy){
                            timg[x][y]=imgM[ix][iy];
                            //if (timg[x][y].img!=null) timg[x][y].img.getGraphics().fillRect(0, 0, 100, 100);

                        }else{
                            timg[x][y]=new MyTile();
                            int xx=sx+((ix<0)?ix:-ix);
                            int yy=sy+((iy<0)?iy:-iy);
                            if (xx>=0 && xx<sx && yy>=0 && yy<sy && imgM[xx][yy]!=null)
                                imgM[xx][yy].flush();
                        }

                    }
                oldtx=tx;
                oldty=ty;
                imgM=timg;
            }catch (Exception ex){System.out.println("dragImgM "+ex.toString());}
        }
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    static Rectangle r = new Rectangle();
    MyTile [][]imgM;
    private void updater(){
        if (threadRun == false){
            threadRun=true;
            new Thread(new Runnable() {
                @Override
                public void run() {
                    while(threadRun) {
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        }
                        //  Log.i("MAP", "upd");
                        postInvalidate();
                    }

                }
            }).start();
        }
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private void init(Canvas c){
        Rect r=c.getClipBounds();
        updater();

        int sx=(c.getWidth()+512)>>8;
        int sy=(c.getHeight()+512)>>8;
        imgM=new MyTile[sx][sy];

        sDirection=new MySlider(40,r.height()-100,r.width()-80,0.5);
        for (int y = 0; y < sy; y++)
            for (int x = 0; x < sx; x++)
                imgM[x][y] = new MyTile();
    }
    static boolean threadRun=false;
    static int oldType=-1;
    private void drawMap(Canvas c) {
        monitor.zoom = zoom;
        InetMaps.type=type;
        r.width = c.getWidth();
        r.height = c.getHeight();

        if (imgM == null)
            init(c);
        if (c==null)
            return;
        //try{
        if (r==null || imgM==null)return;
        double size=0,sizeR=0,n=imgM.length*imgM[0].length,blank=0;
        Point center=new Point(screenP.x-(r.width>>1),screenP.y-(r.height>>1));
        //  Log.i("MAP", "x= "+center.x);
        //  Log.i("MAP", "y= "+center.y);
        //imgM[x][y];
        Point d=new Point(center.x&255,center.y&255);
        dragImgM(center);
        try{//неиследованная ошибка вискакивает.
        for (int my=0,y=center.y; y<=center.y+r.height+256; y+=256,my++){
            int sy=(my<<8)-d.y;
            for (int mx=0,x=center.x; x<=center.x+r.width+256; x+=256,mx++){
                int sx=(mx<<8)-d.x;
                MyTile tile=(imgM[mx][my].img!=null && type==oldType)?imgM[mx][my]:(imgM[mx][my]=im.getTile(x>>8, y>>8, zoom));
                size+=tile.size;
                Bitmap img=tile.img;
                if (img!=null)
                    c.drawBitmap(img, sx, sy, paint);
            }
        }
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        oldType=type;
    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public static void addZoom(int z){
        zoom+=z;
        if (zoom>19) {
            zoom = 19;
            return;
        }
        if (zoom<1) {
            zoom = 1;
            return;
        }
        xDown=r.width>>1;
        yDown=r.height>>1;

        if (z==1) {
            screenP.x-=(r.width>>1)-(int)xDown;
            screenP.y-=(r.height>>1)-(int)yDown;

            screenP.x <<= 1;
            screenP.y <<= 1;
        }else
        {
            screenP.x >>= 1;
            screenP.y >>= 1;
        }

        Log.i("MAP", "zoom= "+zoom);
    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    double dist=0;
    int fullDist=0;
    static public Programmer prog=new Programmer();

    private void addDot() {
        // TODO add your handling code here:
        double i=19-(double)zoom;

        double x=(double)screenP.x*Math.pow(2,i);
        double y=(double)screenP.y*Math.pow(2,i);

        dist=prog.addDot_(x,y,Programmer.altitude,Programmer.speed_,Programmer.speedZ_,1000,0,GeoDot.LED6,Programmer.cam_zoom);
        if (dist>=0)
            fullDist+=dist;
        // jTextFieldFullLen.setText(Integer.toString((int)fullDist));
        // jTextFieldStepLen.setText(Integer.toString((int)dist));

        invalidate();
    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    static float xDown=0,yDown=0;
    final long tap_max_time = 1000;
    final int DRAG_LEN=1;//      32;
    static public boolean progLoaded=false;
    @TargetApi(Build.VERSION_CODES.FROYO)
    @Override
    public boolean onTouchEvent(MotionEvent event) {
        // get pointer index from the event object
        boolean update=false;

        super.onTouchEvent(event);
        int pointerIndex = event.getActionIndex();


        if (sDirection.onTouchEvent(event)) {
            double angle=360*sDirection.getPos()-180;

            Log.i("MAP","ang="+angle);

            if (DrawMap.selectedDot>=0)
                prog.dot[DrawMap.selectedDot].direction=Programmer.direction=angle;

            invalidate();
            return true;
        }
        ib_zoom_out.onTouchEvent(event);
        if (ib_zoom_out.getStat()==3)
            addZoom(-1);
        ib_zoom_in.onTouchEvent(event);
        if (ib_zoom_in.getStat()==3) {
            addZoom(1);

        }
        ib_menu.onTouchEvent(event);
        if (ib_menu.getStat()==3){
            this.showContextMenu();
        }

        ib_add.onTouchEvent(event);
        if (ib_add.getStat()==3){
            addDot();
            progLoaded=false;
        }

        ib_upload.onTouchEvent(event);
        if (ib_upload.getStat()==3){
            progLoaded=true;
            Commander.startUPLoadingProgram();
        }
        ib_edit.onTouchEvent(event);
        if (ib_edit.getStat()==3 &&selectedDot>=0  && MapEdit.active==false && prog.getSize()>0){
            MapEdit.active=true;
            Intent myIntent = new Intent(Map.cont, MapEdit.class);
            Map.cont.startActivity(myIntent);
        }


        ib_del.onTouchEvent(event);
        if (ib_del.getStat()==3 && MapEdit.active==false) {
                prog.deliteLast();
                if (prog.noData())
                    progLoaded=false;
                //invalidate();
        }
        int pIndex = event.getActionIndex();
        int maskedAction = event.getActionMasked();
        switch (maskedAction) {
            case MotionEvent.ACTION_DOWN:
            case MotionEvent.ACTION_POINTER_DOWN: {
                    xDown = event.getX(pIndex);
                    yDown = event.getY(pIndex);
                }
                break;

            case MotionEvent.ACTION_MOVE:  // a pointer was moved
                if (event.getPointerCount()==1) {
                    float x = event.getX(pIndex);
                    float y = event.getY(pIndex);
                    int deltaX = (int) (xDown - x);
                    int deltaY = (int) (yDown - y);
                    if (Math.abs(deltaX) >= DRAG_LEN || Math.abs(deltaY) >= DRAG_LEN) {
                        int addx = (int) (xDown - x);//>>2;
                        int addy = (int) (yDown - y);//>>2;
                        if ((addx*addx+addy*addy)<10000) {
                            screenP.x += addx;
                            screenP.y += addy;
                            xDown = x;
                            yDown = y;
                        }

                    }
                }

                break;

            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_POINTER_UP:

                break;
            case MotionEvent.ACTION_CANCEL:
                //  Log.i("MAP", "tap= "+tap);
                break;
        }
        mScaleGestureDetector.onTouchEvent(event);
        invalidate();
        return true;
    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public double lat,lon;
    private void fromPointToLatLng(){

        double y=(double)screenP.y/Math.pow(2, zoom);
        double x=(double)screenP.x/Math.pow(2, zoom);
        lat=(2 * Math.atan(Math.exp((y - 128) / -(256 / (2 * Math.PI)))) -
                Math.PI / 2)/ (Math.PI / 180.0);
        lon=(x - 128.0) / (256.0 / 360.0);

        //  longitude.setText(grad_min_sec(lng,'E','W'));
        //  latitude.setText(grad_min_sec(lat,'N','S'));
        ////   Log.i("MAP","lat="+lat);
        //  Log.i("MAP","lon="+lon);

        //  jTextPaneLat.setText(grad_min_sec(lat,'N','S'));
        //  jTextPaneLon.setText(grad_min_sec(lon,'E','W'));

    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    static private double startPos_lat,startPos_lon;

    Point lon_lat_2_XY(double lat,double lon){
        Point xy=new Point();
        double siny =  Math.min(Math.max(Math.sin(lat* (Math.PI / 180.0)), -.9999),.9999);
        double copterX=( 128.0 + lon * (256.0/360.0));
        double copterY=( 128.0 + 0.5 * Math.log((1 + siny) / (1 - siny)) * -(256.0 / (2 * Math.PI)));
        double i=(double)zoom;
        int y=(int)((double)screenP.y-(r.height>>1));
        int x=(int)((double)screenP.x-(r.width>>1));
        xy.x=(int)(copterX*Math.pow(2,i))-x;
        xy.y=(int)(copterY*Math.pow(2,i))-y;
        return xy;
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private void drawPhonePos(Canvas g){
        if (GPSservice.mLastLocation != null){
            Point xy=lon_lat_2_XY(GPSservice.mLastLocation.getLatitude(),GPSservice.mLastLocation.getLongitude());
            red.setStyle(Paint.Style.FILL);
            g.drawCircle(xy.x,xy.y,8,red);
            int x=(int)(100*Math.sin(Math.toRadians(-MainActivity.yaw+90)));
            int y=(int)(100*Math.cos(Math.toRadians(-MainActivity.yaw+90)));
            g.drawLine(xy.x,xy.y,xy.x-x,xy.y-y,red);
        }

    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private void drawCopterPos(Canvas g){
        if (progLoaded==false ){
            startPos_lat=Telemetry.lat;
            startPos_lon=Telemetry.lon;
        }

        Point xy=lon_lat_2_XY(Telemetry.lat,Telemetry.lon);

        int x=xy.x;
        int y=xy.y;

        red.setStyle(Paint.Style.FILL);
        g.drawCircle(xy.x,xy.y,5,red);
        //   g.drawArc(x-15, y-15, 30, 30, 0, 360);
        double direction=-Math.toRadians(Telemetry.heading);

        double sy=-30*Math.cos(direction+45.0*0.01745329);
        double sx=-30*Math.sin(direction+45.0*0.01745329);
        g.drawLine(x-(int)sx, y-(int)sy, x+(int)sx, y+(int)sy,red);
        sy=-30*Math.cos(direction-45.0*0.01745329);
        sx=-30*Math.sin(direction-45.0*0.01745329);
        g.drawLine(x-(int)sx, y-(int)sy, x+(int)sx, y+(int)sy,red);

        sy=-150000*Math.cos(direction);
        sx=-150000*Math.sin(direction);
        red.setStrokeWidth(1);
        g.drawLine(x, y, x+(int)sx, y+(int)sy,red);

    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private boolean drawDot(Canvas sg,int x, int y,int dir,int ray_length){

        double i = 19 - (double) zoom;
        if (i>4)
            i=4;

        int radius = (int) (16.0 / Math.pow(2, i));
        ray_length = (int) ((double)ray_length /Math.pow(2,i));

        double dx=x-sg.getWidth()/2;
        double dy=y-sg.getHeight()/2;
        boolean selected=(dx*dx+dy*dy)<radius*radius;
        red.setStyle((selected)?Paint.Style.FILL:Paint.Style.STROKE);

        sg.drawCircle(x, y, radius,red);

        double direction=-Math.toRadians(dir);

        double sy,sx;


        sy=-(ray_length)*Math.cos(direction);
        sx=-(ray_length)*Math.sin(direction);



        sg.drawLine(x, y, x+(int)sx, y+(int)sy, blue);
        return selected;
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private double lastDist1=0,fullDist1=0;
    private double lastAlt1=0;
    private double fullTime=0;
    public static int selectedDot=-1;
    private void drawProgDots(Canvas sg) {
        fullDist1=lastDist1=fullTime=0;
        double i = 19 - (double) zoom;
        int y = (int) ((double) screenP.y - (r.height >> 1));
        int x = (int) ((double) screenP.x - (r.width >> 1));
        int index = 0;
        GeoDot dot = prog.get(index);
        if (dot == null) {
            lastAlt1=0;
            return;
        }
        selectedDot=-1;
        int x0 = (int) (dot.tx / Math.pow(2, i) - x);
        int y0 = (int) (dot.ty / Math.pow(2, i) - y);
        if (drawDot(sg, x0, y0, (int)dot.direction,(int)(64)))
            selectedDot=0;
        double upTime=0, spTime=0;;
        if (dot.speedZ!=0)
            upTime=1.25*dot.dAlt/dot.speedZ;
        if (dot.speed>0)
            spTime=1.25*lastDist1/dot.speed;

        fullTime=Math.max(dot.timer_,upTime+spTime);
        lastAlt1= dot.alt;
        lastDist1=fullDist1=0;
        index=1;
        while (true) {
            dot = prog.get(index);
            if (dot == null)
                break;
            else {
                lastAlt1=dot.alt;
                lastDist1=dot.dDist;
                spTime=0;
                upTime=0;
                if (dot.speed>0)
                    spTime=1.25*lastDist1/dot.speed;
                if (dot.speedZ!=0)
                    if (dot.dAlt>0)
                        upTime=1.25*dot.dAlt/dot.speedZ;
                    else
                        upTime=-1.25*dot.dAlt/dot.speedZ/3*5;

                fullTime+=Math.max(Math.max(dot.timer_,spTime),upTime);
                fullDist1+=lastDist1;
                int x1 = (int) (dot.tx / Math.pow(2, i) - x);
                int y1 = (int) (dot.ty / Math.pow(2, i) - y);
                double tx=x1-x0;
                double ty=y1-y0;
                double pixelLen=Math.sqrt(tx*tx+ty*ty)/lastDist1;
                sg.drawLine(x0, y0, x1, y1, red);
                x0 = x1;
                y0 = y1;

                if (drawDot(sg, x0, y0, (int)dot.direction,(int)(pixelLen*dot.speed/0.2)))
                    selectedDot=index;
                index++;
            }
        }
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    Paint red=new Paint();
    Paint blue =new Paint();
    private void drawCross(final Canvas c){
        red.setColor(Color.RED);
        blue.setColor(Color.BLUE);
        red.setStyle(Paint.Style.STROKE);
        blue.setStyle(Paint.Style.STROKE);
        red.setStrokeWidth(5);
        blue.setStrokeWidth(5);
        float x=r.width>>1;
        float y=r.height>>1;
        c.drawLine(x-10,y,x+10,y,red);
        c.drawLine(x,y-10,x,y+10,red);
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    @Override
    public void onDraw(Canvas c) {

        updater();

        drawMap(c);
        sDirection.draw(c);
        drawCross(c);
        fromPointToLatLng();
        drawCopterPos(c);
        drawPhonePos(c);
        drawProgDots(c);

        Paint black=new Paint();
        if (type==7)
            black.setColor(Color.GREEN);
        else
            black.setColor(Color.BLACK);

        black.setTextSize(20);
        int yind=50;
        c.drawText("copter alt. "+MapEdit.getNum((MainActivity.motorsOnF())?Telemetry.relAlt:0,2)+" m.",textX,yind,black);
        c.drawText("lastDot alt. "+MapEdit.getNum(lastAlt1,2)+" m.",textX,yind+=20,black);

        c.drawText("stepLen. "+MapEdit.getNum(lastDist1,2)+" m.",textX,yind+=20,black);
        c.drawText("fullLen. "+MapEdit.getNum(fullDist1,2)+" m.",textX,yind+=20,black);
        c.drawText("Time. "+MapEdit.getNum(fullTime,2)+" s.",textX,yind+=20,black);
        c.drawText("dots. "+Integer.toString(Programmer.size()),textX,yind+=20,black);
        //drawMap(c);

        ib_zoom_out.paint(c);
        ib_zoom_in.paint(c);
        ib_menu.paint(c);
        ib_add.paint(c);
        ib_edit.paint(c);
        ib_del.paint(c);
        ib_upload.paint(c);
    }







}
