/*---------------------------------------------------------------------
PID制御サンプルコード
pid_sample.c

機能：PID制御出力と１次遅れ+むだ時間でシミュレーションをする。
	printf()の出力をcsv形式としているので、コマンドラインのリダイレクトでファイル出力するとよい。
	Excelで開くことができる。
	１次遅れフィルタのアルゴリズムは４次ルンゲクッタ法を用いた。
	なお同時にステップ応答と不完全微分のシミュレーションも行っている。

作成日：2016/2/10
最終更新日：

作成者：Goto Shunichi

備考：float型専用

-----------------------------------------------------------------------*/

#include <stdio.h>
#include "pid_control_all_f.h"

#define STEP		(10*60*2)	//繰り返し数（総サンプリング数）
#define SETVAL		(50.0)		//目標値
#define STARTVAL	(0.0)		//１次遅れフィルタの初期値

#define PGAIN 		(8.0)		//Pゲイン
#define TI			(8.0)		//積分時間(0.0禁止)
#define TD			(2.0)		//微分時間

#define DT			(0.1)		//制御周期[s]
#define DFF			(0.1)		//微分係数(0.1~0.125)

#define OUTMAX		(150.0)		//PID操作量最大値
#define OUTMIN		(0.0)		//PID操作量最小値
#define DELTA_OUTMAX (50.0)		//PID操作変化量最大値
#define DELTA_OUTMIN (-50.0)	//PID操作変化量最小値

#define TAU			(30.0)		//１次遅れフィルタ等価時定数
#define RK4_GAIN	(1.0)		//１次遅れフィルタゲイン

#define PURE_DELAY	(4.0)		//等価むだ時間[s]



//PIDパラメータ型変数を定義
PID_STATE_t pid_samples = newPID_state_init();

//むだ時間バッファ
float delay_fifo[(1024 * 100)];
float delay_fifo_step[(1024 * 100)];


//1次遅れフィルタ関数プロトタイプ
float rk4(float indata, float tau, float dt,float gain);
float rk4_step(float indata, float tau, float dt,float gain);
float delaytime(float *buf,float input,float puredelay,float dt);
float delaytime_step(float *buf,float input,float puredelay,float dt);
float lagged_derivative(float indata, float td, float dff, float dt);



int main(void) {
	int hogea = STEP,i;
	float hogeb = DT,
			hogec = STARTVAL,
			hoged = 0.0,
			hogee = 0.0,
			hogef = 0.0,
			hogeg = TAU,
			hogeh = RK4_GAIN,
			hogei = PURE_DELAY,
			lagdiff = 0.0;
	float readPgain = 0.0,
			readTigain = 0.0,
			readTdgain = 0.0;


		//PIDパラメータ設定
		Set_pid_gain(&pid_samples, PGAIN, TI, TD);
		Set_pid_dt(&pid_samples, DT);
		Set_pid_dff(&pid_samples, DFF);
		Set_pid_outlim(&pid_samples, OUTMAX, OUTMIN);
		Set_pid_deltaoutlim(&pid_samples, DELTA_OUTMAX, DELTA_OUTMIN);

		//PIDパラメータ読み出し
		Read_vpidgain(&pid_samples,&readPgain,&readTigain,&readTdgain);
		hoged = readPgain;
		hogee = readTigain;
		hogef = readTdgain;

		//csv処理(グラフタイトル)
		printf("Step,");
		printf("setvalue,");
		printf("pid_output,");
		printf("feedback,");
		printf("step-resp,");
		printf("lagdiff_velo,");
		printf("lagdiff,");
		printf(",");
		printf("Step=%d,dt=%f[s],rk4start=%f," ,hogea,hogeb,hogec);
		printf("Pgain=%f,Ti=%f,Td*PureTdgain=%f,Tau=%f[s],rk4gain=%f,Puredelay=%f[s]",
				hoged,hogee,hogef,hogeg,hogeh,hogei);
		printf("\n");

		//制御ループ計算
		for ( i = 0; i < STEP; ++i)
		{
			static float feedback = STARTVAL;
			float setvalue = SETVAL ,pid_output = 0.0,delay_output = 0.0,time = 0.0;
			static float no_ctrl_rk4 = STARTVAL;

		/*シミュレーション１　1次遅れ＋むだ時間系のPID制御(ステップ入力)------------------------*/
			//PID演算
			pid_output = Velocitytype_i_pd(&pid_samples,setvalue, feedback);

			//むだ時間シミュレーション
			delay_output = delaytime(delay_fifo,pid_output,PURE_DELAY,DT);

			//1次遅れシミュレーション(４次ルンゲクッタ)
			feedback = rk4(delay_output, TAU, DT,RK4_GAIN);

		/*シミュレーション２　1次遅れ＋むだ時間系のステップ応答（PID制御無し）---------------------*/
			//ステップ応答用
			no_ctrl_rk4 = delaytime_step(delay_fifo_step,SETVAL,PURE_DELAY,DT);
			no_ctrl_rk4 = rk4_step(no_ctrl_rk4,TAU,DT,RK4_GAIN);

		/*シミュレーション１・２のCSV出力処理---------------------------------------------*/
			time = (float)i * DT;

			printf("%f", time);			printf(",");
			printf("%f", setvalue);		printf(",");
			printf("%f", pid_output);	printf(",");
			printf("%f", feedback);		printf(",");
			printf("%f", no_ctrl_rk4);	printf(",");

		/*シミュレーション３　PID制御の不完全微分のみシミュレーション（ステップ入力）------------------*/
			//不完全微分ステップ応答用
			lagdiff = lagged_derivative(SETVAL,TD,DFF,DT);
			printf("%f", lagdiff);		printf("\n");
		}
	return (0);
}

//不完全微分
float lagged_derivative(float indata, float td, float dff, float dt)
{
	static float velocity_d = 0.0, data_d = 0.0;
	static float error[3] = {0.0, 0.0, 0.0};

	error[0] = indata;

	velocity_d = (td * (error[0] - 2*error[1] +error[2])) + (td * dff * velocity_d);
	velocity_d = velocity_d / (dt + dff * td);

	//速度不完全微分値出力
	printf("%f,",velocity_d);

	error[2] = error[1];
	error[1] = error[0];

	//積分して位置微分信号へ
	data_d += velocity_d;

	return (data_d);

}


//むだ時間バッファ(リングバッファ方式のFIFO)
float delaytime(float *buf,float input,float puredelay,float dt)
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


//ステップ応答用むだ時間バッファ（リングバッファによるFIFO）
float delaytime_step(float *buf,float input,float puredelay,float dt)
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
float rk4(float indata, float tau, float dt,float gain)
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

//
//1次遅れフィルタ ステップ応答用
float rk4_step(float indata, float tau, float dt,float gain)
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
