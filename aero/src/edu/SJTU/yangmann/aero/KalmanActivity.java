package edu.SJTU.yangmann.aero;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.FloatMath;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import jama.Matrix;
import jkalman.JKalman;

public class KalmanActivity extends Activity implements SensorEventListener {

    private SensorManager mSensorManager;
    private Sensor mAccelerometer = null;
    private Sensor mGravitySensor = null;
    private Sensor mMagneticSensor = null;
    private float mAccel;
    private float mAccelCurrent;
    private float mAccelLast;
    private float vaccx, vaccy, vaccz, vaccxnew, vaccynew, vaccznew,
            vworldx, vworldy, vworldz,
            vgrax, vgray, vgraz, vgraxnew, vgraynew, vgraznew,
            vmagx, vmagy, vmagz, vmagxnew, vmagynew, vmagznew;

    private float[] RotationMatrix = {(float) 1, (float) 0, (float) 0,
            (float) 0, (float) 1, (float) 0, (float) 0, (float) 0, (float) 1};
    private float[] GravityVector = {(float) 0, (float) 0, (float) 9.8};
    private float[] MagneticVector = {(float) 0, (float) 0, (float) 0};
    private float[] WorldVector = {(float) 0, (float) 0, (float) 0};
    private float[] AccelerVector = {(float) 0, (float) 0, (float) 0};


    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];  //改
    private float timestamp;
    private final float[] speed = new float[3];
    private final float[] loc = new float[3];
    private final float[] Fspeed = new float[3];
    private final float[] Floc = new float[3];

    private TextView TextViewX = null;    //文本框
    private TextView TextViewV = null;    //文本框
    private TextView TextViewA = null;    //文本框

    private TextView TextViewFX = null;    //文本框
    private TextView TextViewFV = null;    //文本框
    private TextView TextViewFA = null;    //文本框

    private TextView TextViewKS = null;

    private float dT, dT2;
    private int counter = 0, accounter = 0, decounter = 0, accnumber = 0;
    private static final float hz = (float) 2.0, pi = (float) 3.14159265, average = (float) 0.9, threshacc = (float) 1.3;
    private float ws = 0, threshwindow = (float) 16.0, accwindow = 0, mSin, mCos, accvalue;    //threshwindow=average*window
    private static final int fs = 16, wlen = 50, window = 16;
    private int slen = 0, judgetimecounter = 0, notaccounter = 0;
    private float[] sinc;
    private boolean startjudge = false;
    private float[] worldxfiltered, worldyfiltered, worldxraw, worldyraw, worldzraw, worldzfiltered;

    private JKalman kalman;
    private Matrix state;
    private Matrix state_corrected;
    private Matrix measurement;

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        dT2 = 0;
        dT = 0;

        ws = (float) (2 * pi * hz / ((float) fs));
        slen = 2 * (int) (fs / hz);
        sinc = new float[slen];
        worldxfiltered = new float[wlen];
        worldxraw = new float[wlen];
        worldyfiltered = new float[wlen];
        worldyraw = new float[wlen];
        worldzfiltered = new float[wlen];
        worldzraw = new float[wlen];
        for (int i = 0; i < slen; i++) {
            if (i - slen / 2 != 0) {
                sinc[i] = (float) (1 / pi * Math.sin(ws * (i - slen / 2)) / (i - slen / 2));
            } else {
                sinc[i] = ws / pi;
            }
        }
        threshwindow = window * average;


        // TODO 以上来自以前

        try {
            kalman = new JKalman(6, 3);

            state = new Matrix(6, 1); // state [x, y, z, dx, dy, dz]
            state_corrected = new Matrix(6, 1);
            measurement = new Matrix(3, 1); // measurement [dx, dy, dz]
            double[][] tr = {
                    {1, 0, 0, 1, 0, 0},
                    {0, 1, 0, 0, 1, 0},
                    {0, 0, 1, 0, 0, 1},
                    {0, 0, 0, 1, 0, 0},
                    {0, 0, 0, 0, 1, 0},
                    {0, 0, 0, 0, 0, 1}
            };
            kalman.setTransition_matrix(new Matrix(tr));
            kalman.setError_cov_post(kalman.getError_cov_post().identity());
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        Button bt1 = null;

        View.OnClickListener listener = null;                  //声明监听器

        TextViewX = (TextView) findViewById(R.id.textViewx);    //文本框
        TextViewV = (TextView) findViewById(R.id.textViewv);    //文本框
        TextViewA = (TextView) findViewById(R.id.textViewa);    //文本框

        TextViewFX = (TextView) findViewById(R.id.textViewFx);    //文本框
        TextViewFV = (TextView) findViewById(R.id.textViewFv);    //文本框
        TextViewFA = (TextView) findViewById(R.id.textViewFa);    //文本框

        TextViewKS = (TextView) findViewById(R.id.textViewKalmanState);

        bt1 = (Button) findViewById(R.id.button1);                            //按钮
        bt1.setOnClickListener(listener = new View.OnClickListener() {                //设置监听器
            @Override
            public void onClick(View v) {
                //TextViewX.setText("System time in nanoseconds: " + Long.toString(System.nanoTime()));				//设置文本   System.nanoTime() 挺好用 但是貌似用不到
                speed[0] = 0;
                Fspeed[0] = 0;
                speed[1] = 0;
                Fspeed[1] = 0;
                speed[2] = 0;
                Fspeed[2] = 0;
                loc[0] = 0;
                Floc[0] = 0;
                loc[1] = 0;
                Floc[1] = 0;
                loc[2] = 0;
                Floc[2] = 0;

                // TODO: Add a Timer for 30 seconds to get better calibration.
            }
        });

        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mAccel = 0.00f;
        mAccelCurrent = SensorManager.GRAVITY_EARTH;
        mAccelLast = SensorManager.GRAVITY_EARTH;

        mGravitySensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        mMagneticSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

    }

    @Override
    public void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);  //为何？ 不可用FASTEST
        mSensorManager.registerListener(this, mGravitySensor, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mMagneticSensor, SensorManager.SENSOR_DELAY_UI);

    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
    }

    private float[] getnewvector(float[] x, float[] y) {
        float[] mMatrix = new float[3];
        mMatrix[0] = x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        mMatrix[1] = x[3] * y[0] + x[4] * y[1] + x[5] * y[2];
        mMatrix[2] = x[6] * y[0] + x[7] * y[1] + x[8] * y[2];
        return mMatrix;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            float[] mGravity = event.values.clone();
            mAccelLast = mAccelCurrent;
            mAccelCurrent = FloatMath.sqrt(mGravity[0] * mGravity[0] + mGravity[1] * mGravity[1] + mGravity[2] * mGravity[2]);
            mAccel = mAccel * 0.9f + mAccelCurrent - mAccelLast;

            // Tried the elapsedRealtimeNanos() but didn't work. Moved it to the button callback.

            measurement.set(0, 0, event.values[0]);
            measurement.set(1, 0, event.values[1]);
            measurement.set(2, 0, event.values[2]);

            if (mAccel > 3) {
                Log.wtf("mAccel", "SHAKE!!!!");
            }

            state = kalman.Predict();
            state_corrected = kalman.Correct(measurement);

            Log.e("COUNT!!!!", " " + counter + "STAMP " + timestamp);
            // 积分的部分 NS2S
            if (timestamp != 0) {
                dT = (event.timestamp - timestamp) * NS2S;  //////////////////////////////
                dT2 += (event.timestamp - timestamp) * NS2S;                            // Timer for Filtered data
                speed[0] += event.values[0] * dT;
                speed[1] += event.values[1] * dT;
                speed[2] += event.values[2] * dT;

                loc[0] += speed[0] * dT;
                loc[1] += speed[1] * dT;
                loc[2] += speed[2] * dT;

                TextViewA.setText("Acc X: " + event.values[0] + "\nY: " + event.values[1] + "\nZ: " + event.values[2]);
                TextViewV.setText("Vel X: " + speed[0] + "\nY: " + speed[1] + "\nZ: " + speed[2]);
                TextViewX.setText("Accuracy: " + event.accuracy + "\nLoc X: " + loc[0] + "\nY: " + loc[1] + "\nZ: " + loc[2]);


                // 来自以前
                // 滤波方法大致如下： 先和以前的加速度取平均 然后再滤波

                AccelerVector[0] = vaccx = (vaccx + event.values[0]) / 2;
                AccelerVector[1] = vaccy = (vaccy + event.values[1]) / 2;
                AccelerVector[2] = vaccz = (vaccz + event.values[2]) / 2;

                //Log.e("COUNT", "" + counter);
                if (counter == 1) {    //  50/3 hz   Not Happening
                    // get world vector
                    SensorManager.getRotationMatrix(RotationMatrix, null,
                            GravityVector, MagneticVector);
                    WorldVector = getnewvector(RotationMatrix, AccelerVector);  //// TODO 非常重要的 但为什么会变成   w
                    // store world vect
                    for (int i = 1; i < wlen; i++) {
                        worldxraw[i] = worldxraw[i - 1];
                        worldxfiltered[i] = worldxfiltered[i - 1];
                        worldyraw[i] = worldyraw[i - 1];
                        worldyfiltered[i] = worldyfiltered[i - 1];
                        worldzraw[i] = worldzraw[i - 1];
                        worldzfiltered[i] = worldzfiltered[i - 1];
                    }
                    worldxraw[0] = vworldx = WorldVector[0];
                    worldyraw[0] = vworldy = WorldVector[1];
                    worldzraw[0] = vworldz = WorldVector[2];
                    //vworldz=WorldVector[2];
                    // start sinc filter
                    worldxfiltered[0] = worldyfiltered[0] = worldzfiltered[0] = 0;
                    for (int i = 0; i < slen; i++) {
                        worldxfiltered[0] += sinc[i] * worldxraw[i];
                        worldyfiltered[0] += sinc[i] * worldyraw[i];
                        worldzfiltered[0] += sinc[i] * worldzraw[i];
                    }

                    Fspeed[0] += worldxfiltered[0] * dT2;
                    Fspeed[1] += worldyfiltered[0] * dT2;
                    Fspeed[2] += worldzfiltered[0] * dT2;

                    Floc[0] += Fspeed[0] * dT2;
                    Floc[1] += Fspeed[1] * dT2;
                    Floc[2] += Fspeed[2] * dT2;

                    /// 输出结果的部分
                   /* /Log.v("mLocation", "[x]" + loc[0] + "\t[y]" + loc[1] + "\t[z]" +
                            loc[2] + "\tDelta Time[s] " + dT);*/
                    TextViewFA.setText("Acc X: " + worldxfiltered[0] +
                            "\nY: " + worldyfiltered[0] + "\nZ: " + worldzfiltered[0]);
                    TextViewFV.setText("Vel X: " + Fspeed[0] + "\nY: " + Fspeed[1] + "\nZ: " + Fspeed[2]);
                    TextViewFX.setText("------Filtered Data------\nLoc X: " + Floc[0] + "\nY: " + Floc[1] + "\nZ: " + Floc[2]);
                    dT2 = 0;

                    Log.v("TTEXT", "WHAT");
                    TextViewKS.setText("WHAT");

                } else if (counter >= 2) {
                    counter = 0;
                } else {
                    counter++;
                    Log.e("COUNT000",""+counter);
                }
            }
            timestamp = event.timestamp;

            // End of the ACC Sensor

        } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
            vgraxnew = event.values[0];
            vgraynew = event.values[1];
            vgraznew = event.values[2];
            GravityVector[0] = vgrax = (vgrax + vgraxnew) / 2;
            GravityVector[1] = vgray = (vgray + vgraynew) / 2;
            GravityVector[2] = vgraz = (vgraz + vgraznew) / 2;
            //Log.v("mGravity", "X: " + GravityVector[0]);
           /* tvgrax.setText("X: " + vgrax);
            tvgray.setText("Y: " + vgray);
            tvgraz.setText("Z: " + vgraz);*/
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            vmagxnew = event.values[0];
            vmagynew = event.values[1];
            vmagznew = event.values[2];
            MagneticVector[0] = vmagx = (vmagx + vmagxnew) / 2;
            MagneticVector[1] = vmagy = (vmagy + vmagynew) / 2;
            MagneticVector[2] = vmagz = (vmagz + vmagznew) / 2;
            //Log.v("mMag", "X: " + MagneticVector[0]);
            /*tvmagx.setText("X: " + vmagx);
            tvmagy.setText("Y: " + vmagy);
            tvmagz.setText("Z: " + vmagz);*/
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}
