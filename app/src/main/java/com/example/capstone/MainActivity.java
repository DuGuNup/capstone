package com.example.capstone;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.DialogInterface;
import android.content.Intent;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;


import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;

import android.telephony.SmsManager;


public class MainActivity extends AppCompatActivity {
    TextView mTvBluetoothStatus;
    TextView mTvReceiveData;
    Button mBtnConnect;
    Button mBtnData;
    //Button mBtnSendData;
    //TextView mTvSendData;
    //Button mBtnBluetoothOn;
    //Button mBtnBluetoothOff;
    Cursor cursor;
    SQLiteDatabase db = null;
    DBHelper dbHelper;

    BluetoothAdapter mBluetoothAdapter;
    Set<BluetoothDevice> mPairedDevices;
    List<String> mListPairedDevices;

    Handler mBluetoothHandler;
    ConnectedBluetoothThread mThreadConnectedBluetooth;
    BluetoothDevice mBluetoothDevice;
    BluetoothSocket mBluetoothSocket;

    final static int BT_REQUEST_ENABLE = 1;
    final static int BT_MESSAGE_READ = 2;
    final static int BT_CONNECTING_STATUS = 3;
    final static UUID BT_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    /*
    int Fall_Lv_1 = 1;
    int Fall_Lv_2 = 1;
    int Fall_Lv_3 = 1;
    */

    int Alert  = 1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //mTvBluetoothStatus = (TextView) findViewById(R.id.tvBluetoothStatus);
        mTvReceiveData = (TextView) findViewById(R.id.tvReceiveData);
        mBtnConnect = (Button) findViewById(R.id.btnConnect);
        mBtnData = (Button) findViewById(R.id.dataTable);
        //mBtnSendData = (Button)findViewById(R.id.btnSendData);
        //mTvSendData =  (EditText) findViewById(R.id.tvSendData);
        //mBtnBluetoothOn = (Button)findViewById(R.id.btnBluetoothOn);
        //mBtnBluetoothOff = (Button)findViewById(R.id.btnBluetoothOff);

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        dbHelper = new DBHelper(this, 4);
        db = dbHelper.getWritableDatabase();


        /*mBtnBluetoothOn.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View view) {
                bluetoothOn();
            }
        });
        mBtnBluetoothOff.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View view) {
                bluetoothOff();
            }
        });*/
        mBtnConnect.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View view) {
                listPairedDevices();
            }
        });
        mBtnData.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View view) {
                callData();
            }
        });
        /*mBtnSendData.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View view) {
                if(mThreadConnectedBluetooth != null) {
                    mThreadConnectedBluetooth.write(mTvSendData.getText().toString());
                    mTvSendData.setText("");
                }
            }
        });*/

        mBluetoothHandler = new Handler() {
            public void handleMessage(android.os.Message msg) {
                // 이 밑 어디선가 블루투스로 사고정보 수신
                if (msg.what == BT_MESSAGE_READ) {
                    String readMessage = null;
                    try {
                        readMessage = new String((byte[]) msg.obj, "UTF-8");
                    } catch (UnsupportedEncodingException e) {
                        e.printStackTrace();
                    }
                    mTvReceiveData.setText(readMessage);
                //저기랑 여기 사이
                    if (readMessage.contains("A") && Alert == 1) { //쌍따옴표 안에 들어가는게 사고 유형 코드
                        Alert = 0;//중복감지 방지를 위해 알레트 꺼버리기
                        Toast.makeText(getApplicationContext(), "전복사고 감지", Toast.LENGTH_LONG).show();//토스트 메시지로 알림
                        cursor = db.rawQuery("SELECT * FROM tableName", null);//주소록 로드
                        startManagingCursor(cursor);


                        while (cursor.moveToNext()) {//주소록에 모든 번호로 문자 보낼떄까지 반복
                            try {
                                //전송
                                SmsManager smsManager = SmsManager.getDefault();
                                smsManager.sendTextMessage(cursor.getString(1), null, "전복사고 감지", null, null);//문자 전송
                                Toast.makeText(getApplicationContext(), "전송 완료!", Toast.LENGTH_LONG).show();
                            } catch (Exception e) {
                                Toast.makeText(getApplicationContext(), "SMS faild, please try again later!", Toast.LENGTH_LONG).show();
                                e.printStackTrace();
                            }
                        }


                    }

                    if (readMessage.contains("C") && Alert == 1) {
                        Alert = 0;
                        Toast.makeText(getApplicationContext(), "추락사고 감지", Toast.LENGTH_LONG).show();
                        cursor = db.rawQuery("SELECT * FROM tableName", null);
                        startManagingCursor(cursor);

                        while (cursor.moveToNext()) {
                            try {
                                //전송
                                SmsManager smsManager = SmsManager.getDefault();
                                smsManager.sendTextMessage(cursor.getString(1), null, "추락사고 감지", null, null);
                                Toast.makeText(getApplicationContext(), "전송 완료!", Toast.LENGTH_LONG).show();
                            } catch (Exception e) {
                                Toast.makeText(getApplicationContext(), "SMS faild, please try again later!", Toast.LENGTH_LONG).show();
                                e.printStackTrace();
                            }
                        }
                    }

                    if (readMessage.contains("D") && Alert == 1) {
                        Alert = 0;
                        Toast.makeText(getApplicationContext(), "충돌사고 감지", Toast.LENGTH_LONG).show();
                        cursor = db.rawQuery("SELECT * FROM tableName", null);
                        startManagingCursor(cursor);

                        while (cursor.moveToNext()) {
                            try {
                                //전송
                                SmsManager smsManager = SmsManager.getDefault();
                                smsManager.sendTextMessage(cursor.getString(1), null, "충돌사고 감지", null, null);
                                Toast.makeText(getApplicationContext(), "전송 완료!", Toast.LENGTH_LONG).show();
                            } catch (Exception e) {
                                Toast.makeText(getApplicationContext(), "SMS faild, please try again later!", Toast.LENGTH_LONG).show();
                                e.printStackTrace();
                            }
                        }
                    }

                    if (readMessage.contains("E") && Alert == 1) {
                        Alert = 0;
                        Toast.makeText(getApplicationContext(), "충돌사고 후 전복 감지", Toast.LENGTH_LONG).show();
                        cursor = db.rawQuery("SELECT * FROM tableName", null);
                        startManagingCursor(cursor);

                        while (cursor.moveToNext()) {
                            try {
                                //전송
                                SmsManager smsManager = SmsManager.getDefault();
                                smsManager.sendTextMessage(cursor.getString(1), null, "충돌사고 후 전복 감지", null, null);
                                Toast.makeText(getApplicationContext(), "전송 완료!", Toast.LENGTH_LONG).show();
                            } catch (Exception e) {
                                Toast.makeText(getApplicationContext(), "SMS faild, please try again later!", Toast.LENGTH_LONG).show();
                                e.printStackTrace();
                            }
                        }
                    }

                    if (readMessage.contains("G") && Alert == 1) {
                        Alert = 0;
                        Toast.makeText(getApplicationContext(), "충돌사고 후 추락 감지", Toast.LENGTH_LONG).show();
                        cursor = db.rawQuery("SELECT * FROM tableName", null);
                        startManagingCursor(cursor);

                        while (cursor.moveToNext()) {
                            try {
                                //전송
                                SmsManager smsManager = SmsManager.getDefault();
                                smsManager.sendTextMessage(cursor.getString(1), null, "충돌사고 후 추락 감지", null, null);
                                Toast.makeText(getApplicationContext(), "전송 완료!", Toast.LENGTH_LONG).show();
                            } catch (Exception e) {
                                Toast.makeText(getApplicationContext(), "SMS faild, please try again later!", Toast.LENGTH_LONG).show();
                                e.printStackTrace();
                            }
                        }
                    }


                    /*
                    while (true) {
                        mTvReceiveData.setText("다시 감지를 시작하시려면 앱을 재시작해 주세요");
                    }
                    */


                }
            }
        };
    }
    /*
    void bluetoothOn() {
        if(mBluetoothAdapter == null) {
            Toast.makeText(getApplicationContext(), "블루투스를 지원하지 않는 기기입니다.", Toast.LENGTH_LONG).show();
        }
        else {
            if (mBluetoothAdapter.isEnabled()) {
                Toast.makeText(getApplicationContext(), "블루투스가 이미 활성화 되어 있습니다.", Toast.LENGTH_LONG).show();
                mTvBluetoothStatus.setText("활성화");
            }
            else {
                Toast.makeText(getApplicationContext(), "블루투스가 활성화 되어 있지 않습니다.", Toast.LENGTH_LONG).show();
                Intent intentBluetoothEnable = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(intentBluetoothEnable, BT_REQUEST_ENABLE);
            }
        }
    }
    void bluetoothOff() {
        if (mBluetoothAdapter.isEnabled()) {
            mBluetoothAdapter.disable();
            Toast.makeText(getApplicationContext(), "블루투스가 비활성화 되었습니다.", Toast.LENGTH_SHORT).show();
            mTvBluetoothStatus.setText("비활성화");
        }
        else {
            Toast.makeText(getApplicationContext(), "블루투스가 이미 비활성화 되어 있습니다.", Toast.LENGTH_SHORT).show();
        }
    }
    @Override

    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
            case BT_REQUEST_ENABLE:
                if (resultCode == RESULT_OK) { // 블루투스 활성화를 확인을 클릭하였다면
                    Toast.makeText(getApplicationContext(), "블루투스 활성화", Toast.LENGTH_LONG).show();
                    mTvBluetoothStatus.setText("활성화");
                } else if (resultCode == RESULT_CANCELED) { // 블루투스 활성화를 취소를 클릭하였다면
                    Toast.makeText(getApplicationContext(), "취소", Toast.LENGTH_LONG).show();
                    mTvBluetoothStatus.setText("비활성화");
                }
                break;
        }
        super.onActivityResult(requestCode, resultCode, data);
    }
    */

    public void callData() {
        Intent intent = new Intent(getApplicationContext(), MenuActivity.class);
        startActivity(intent);
    }


    void listPairedDevices() {
        if (mBluetoothAdapter.isEnabled()) {
            mPairedDevices = mBluetoothAdapter.getBondedDevices();

            if (mPairedDevices.size() > 0) {
                AlertDialog.Builder builder = new AlertDialog.Builder(this);
                builder.setTitle("장치 선택");

                mListPairedDevices = new ArrayList<String>();
                for (BluetoothDevice device : mPairedDevices) {
                    mListPairedDevices.add(device.getName());
                    //mListPairedDevices.add(device.getName() + "\n" + device.getAddress());
                }
                final CharSequence[] items = mListPairedDevices.toArray(new CharSequence[mListPairedDevices.size()]);
                mListPairedDevices.toArray(new CharSequence[mListPairedDevices.size()]);

                builder.setItems(items, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int item) {
                        connectSelectedDevice(items[item].toString());
                    }
                });
                AlertDialog alert = builder.create();
                alert.show();
            } else {
                Toast.makeText(getApplicationContext(), "페어링된 장치가 없습니다.", Toast.LENGTH_LONG).show();
            }
        } else {
            Toast.makeText(getApplicationContext(), "블루투스가 비활성화 되어 있습니다.", Toast.LENGTH_SHORT).show();
        }
    }

    void connectSelectedDevice(String selectedDeviceName) {
        for (BluetoothDevice tempDevice : mPairedDevices) {
            if (selectedDeviceName.equals(tempDevice.getName())) {
                mBluetoothDevice = tempDevice;
                break;
            }
        }
        try {
            mBluetoothSocket = mBluetoothDevice.createRfcommSocketToServiceRecord(BT_UUID);
            mBluetoothSocket.connect();
            mThreadConnectedBluetooth = new ConnectedBluetoothThread(mBluetoothSocket);
            mThreadConnectedBluetooth.start();
            mBluetoothHandler.obtainMessage(BT_CONNECTING_STATUS, 1, -1).sendToTarget();
            mBtnConnect.setSelected(true);
            mTvReceiveData.setText("연결 완료 사고 감지중");
        } catch (IOException e) {
            Toast.makeText(getApplicationContext(), "블루투스 연결 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
        }
    }

    private class ConnectedBluetoothThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedBluetoothThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "소켓 연결 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
            }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int bytes;

            while (true) {
                try {
                    bytes = mmInStream.available();
                    if (bytes != 0) {
                        SystemClock.sleep(100);
                        bytes = mmInStream.available();
                        bytes = mmInStream.read(buffer, 0, bytes);
                        mBluetoothHandler.obtainMessage(BT_MESSAGE_READ, bytes, -1, buffer).sendToTarget();
                    }
                } catch (IOException e) {
                    break;
                }
            }
        }
        /*
        public void write(String str) {
            byte[] bytes = str.getBytes();
            try {
                mmOutStream.write(bytes);
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "데이터 전송 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
            }
        }
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) {
                Toast.makeText(getApplicationContext(), "소켓 해제 중 오류가 발생했습니다.", Toast.LENGTH_LONG).show();
            }
        }
        */


    }
}
