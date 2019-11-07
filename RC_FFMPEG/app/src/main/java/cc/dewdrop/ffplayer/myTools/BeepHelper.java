package cc.dewdrop.ffplayer.myTools;

import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Handler;
import android.os.Looper;

public class BeepHelper
{
static long old_time=0;
static public void beep(){
    long time=System.currentTimeMillis();
    if (time>old_time) {
        old_time=time+500;
        ToneGenerator toneGen1 = new ToneGenerator(AudioManager.STREAM_MUSIC, 100);
        toneGen1.startTone(ToneGenerator.TONE_CDMA_PIP, 300);
    }
}
}