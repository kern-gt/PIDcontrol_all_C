/*---------------------------------------------------------------------
PID制御サンプルコード２
pid_sample2.c

機能：PID制御のテンプレート

作成日：2017/9/10

作成者：Goto Shunichi

備考：float型専用

-----------------------------------------------------------------------*/
#include "pid_control_all_f.h"  //PIDライブラリをインクルード


#define SETVAL		(50.0)		//目標値

//PIDゲイン
#define PGAIN 		(8.0)		//比例ゲイン
#define TI			(8.0)		//積分時間(0.0禁止)
#define TD			(2.0)		//微分時間

#define DT			(0.1)		//制御周期[s]
#define DFF			(0.1)		//微分係数(0.1~0.125)

#define OUTMAX		(100.0)		//PID操作量最大値
#define OUTMIN		(-100.0)	//PID操作量最小値
#define DELTA_OUTMAX (10.0)		//PID操作変化量最大値
#define DELTA_OUTMIN (-10.0)	//PID操作変化量最小値


//PID制御構造体を定義
PID_STATE_t pid_samples;

//周期タイマ実行フラグ(割込みハンドラから操作)
volatile int timer_flag = false;


//プロトタイプ宣言
float GetSensorValue(void);
void  SetActuatorValue(float output_value);


int main(void) {

	float set_value = SETVAL;   //目標値
	float feedback_value = 0.0; //フィードバック値
	float output_value = 0.0;   //制御量

	//PID制御構造体初期化
	InitPid(&pid_samples);

	//PIDパラメータ設定
	Set_pid_gain(&pid_samples, PGAIN, TI, TD);
	Set_pid_dt(&pid_samples, DT);
	Set_pid_dff(&pid_samples, DFF);
	Set_pid_outlim(&pid_samples, OUTMAX, OUTMIN);
	Set_pid_deltaoutlim(&pid_samples, DELTA_OUTMAX, DELTA_OUTMIN);

	while(1){
		if(timer_flag == true){
			timer_flag = false;

			//フィードバック値を取得
			feedback_value = GetSensorValue();
			//PID制御演算
			output_value = VResPID(&pid_samples, set_value, feedback_value);
			//アクチュエータ出力を設定
			SetActuatorValue(output_value);
		}
	}	
	return (0);
}

float GetSensorValue(void){
	//センサなどのフィードバック値を取得します。
	return(0.0);
}


void SetActuatorValue(float output_value){
	//アクチュエータの出力値を設定します。
}

void TimerHandler(void){
	//制御周期DTの周期ハンドラを想定します。
	timer_flag = true;
}