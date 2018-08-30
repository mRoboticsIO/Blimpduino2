package com.jjrobots.blimpduino_v1;

/**
 * Created by Jose on 05/03/2017.
 */

public class MyProperties {
    private static MyProperties mInstance= null;

    public int angle;
    public int battery;
    public String debug;

    protected MyProperties(){}

    public static synchronized MyProperties getInstance(){
        if(null == mInstance){
            mInstance = new MyProperties();
        }
        return mInstance;
    }
}
