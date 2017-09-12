/*---------------------------------------------------------------------
PID制御サンプルコード２ デバッグ用
pid_sample2.c

機能：PID制御のテンプレート
	今回は例としてロボットの左右車輪のモータトルクの制御とします。
	処理の流れは
		トルクセンサからの値を取得
		PID制御演算
		モータの印加電圧（PWM）を設定
	これを制御周期毎に実行する形になります。

作成日：2017/9/10

作成者：Goto Shunichi

備考：float型専用

-----------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "pid_control_all_f.h"  //PIDライブラリをインクルード

//サンプリング数[s/dt]
#define SAMPLE_NUM (int)(10 / DT)

//モータトルク設定値
#define SET_VALUE	(50.0)		//目標値

//PID設定パラメータ
#define PGAIN 		(1.0)		//比例ゲイン
#define TI			(1.0)		//積分時間(0.0禁止)
#define TD			(1.0)		//微分時間

#define DT			(0.01)		//制御周期dt[s]

#define DFF			(0.1)		//不完全微分の微分係数(0.1~0.125)

#define OUTMAX		(100.0)		//PID操作量最大値
#define OUTMIN		(-100.0)	//PID操作量最小値
#define DELTA_OUTMAX (5.0)		//PID操作変化量最大値
#define DELTA_OUTMIN (-5.0)		//PID操作変化量最小値

#define TAU			(30.0)		//１次遅れフィルタ等価時定数
#define RK4_GAIN	(1.0)		//１次遅れフィルタゲイン

#define PURE_DELAY	(4.0)		//等価むだ時間[s]


//PID制御構造体を定義
PIDParameter_t pid_struct;    //PID制御構造体

//むだ時間バッファ
float delay_fifo[(1024 * 100)];


//むだ時間バッファ(リングバッファ方式のFIFO)
float DelayTime(float *buf,float input,float puredelay,float dt)
{
	float output = 0.0,bufsize = 0.0;
	static int inputpoint = 0, outputpoint = 1;

	if(dt >= 1.0)
	{
		bufsize = puredelay * dt;
	}else{
		if(dt > 0.0)
		{
			bufsize = puredelay / dt;
		}
		if(dt == 0.0)
		{
			return (input);
		}
	}

	output = buf[outputpoint++];
	buf[inputpoint++] = input;

	if(bufsize <= outputpoint)
	{
		outputpoint = 0;
	}
	if(bufsize <= inputpoint)
	{
		inputpoint = 0;
	}

	return(output);
}

//1次遅れフィルタ（４次ルンゲクッタ法）
float Rk4(float indata, float tau, float dt,float gain)
{
	float k1 = 0.0, k2 = 0.0, k3 = 0.0, k4 = 0.0;
	static float outdata = 0.0;

	k1 = dt * (indata - outdata) /tau;
	k2 = dt * (indata - (outdata + k1 / 2.0)) / tau;
	k3 = dt * (indata - (outdata + k2 / 2.0)) / tau;
	k4 = dt * (indata - (outdata + k3)) / tau;

	outdata = outdata + gain * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

	return (outdata);
}

//制御対象シミュレーション
float Plant(float control_value){
	float delay_value = 0.0;

	//むだ時間シミュレーション
	delay_value = DelayTime(delay_fifo, control_value, PURE_DELAY, DT);
	//1次遅れシミュレーション
	return(Rk4(delay_value, TAU, DT,RK4_GAIN));
}

//グラフ描画初期化
void InitGraph(void){

	
}

//グラフ描画
void DrawGraph(&pid_struct, set_value, feedback_value,control_value){

}

//グラフ描画終了
void EndGraph(void){
	fclose(fp);
}

int main(void) {

	FILE *fp;

	//目標値
	float set_value = SET_VALUE;

	//フィードバック値
	static float feedback_value = 0.0;

	//制御量
	float control_value = 0.0;


	//PID制御構造体初期化
	InitPid(&pid_struct);

	//PIDパラメータ設定(右モータ)
	SetPidGain(&pid_struct, PGAIN, TI, TD);
	SetPidDt(&pid_struct, DT);
	SetPidDff(&pid_struct, DFF);
	SetPidOutlim(&pid_struct, OUTMAX, OUTMIN);
	SetPidDeltaoutlim(&pid_struct, DELTA_OUTMAX, DELTA_OUTMIN);

	//ファイル作成
	if ((fp = fopen("graph.csv", "w")) == NULL) {
		printf("file open error!!\n");
		exit(1);
	}

	//グラフ出力初期化
	InitGraph();

	for(i=1;i >= SAMPLE_NUM;i++){

		//PID制御演算
		control_value = VResPID(&pid_struct, set_value, feedback_value);
		
		//制御対象シミュレーション
		feedback_value = Plant(control_value);

		//グラフ描画
		DrawGraph(&pid_struct, set_value, feedback_value,control_value);
	}
	fclose(fp);
	return (0);
}