package cc.dewdrop.ffplayer.myTools;

import android.graphics.Rect;

public class Square_Cells {
     private  float sm[];
    private float RectBorder=0.3f;
    private float RectSize;
    private int nX,nY;
    private  double size;
    private  double border;
    public int getMaxX(){return nX;}
     public int getMaxY(){return nY;}
     public int getRectSize(){return (int)RectSize;}
    public Rect getRect(double x, double y){
        int x0=(int)(x*(size+border));
        int y0=(int)(y*(size+border));
        return new Rect(
                (int)(x0+border),
                (int)(y0+border),
                (int)(x0+border+size),
                (int)(y0+border+size)
        );
    }
    public Square_Cells(int x,int y, float _border, float _sm[]){
        RectBorder=_border;
        sm=_sm;
        if (x>0) {
            RectSize = sm[0] / x;
            nX = x;
            nY = (int) (sm[1] / RectSize);
            border = RectSize * RectBorder;
            size = (sm[0] - (border + 1) * nX) / nX;
        }else if (y>0){
            RectSize = sm[1] / y;
            nY = y;
            nX = (int) (sm[0] / RectSize);
            border = RectSize * RectBorder;
            size = (sm[1] - (border + 1) * nY) / nY;
        }
    }


}
