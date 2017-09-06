/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
PID制御ヘッダ　ver2.xx
pid_control_all.h

機能：パラメータ設定/読出し、偏差PID,微分先行形P-ID、比例微分先行形I-PD
	全てfloat型変数を使用

作成日：2015/08

作成者：Goto Shunichi

備考：float型専用

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**/

#ifndef _GTS02_PID_CONTROL_F_H_
#define _GTS02_PID_CONTROL_F_H_

/**------------------------------------------------------------------------------
<<外部公開マクロ定義>>
------------------------------------------------------------------------------**/

//PIDパラメータ初期化
#define newPID_state_init() {1.0, 0.1, 0.1, 0.1, 0.1, 100.0, -100.0, 1.0, -1.0,{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},0.0,{0.0, 0.0, 0.0},{0.0, 0.0},0.0};
/*初期化パラメータ------------------------------------------------------------
	pgain	= 1.0;			//比例ゲイン
	ti		= 0.1;			//積分時間
	td　		= 0.1;			//微分時間
	dff　	= 0.1;			//実用微分係数(0.1~0.125)
	dt　		= 0.1;			//制御周期　内部は(秒)

	outmax 	=  100.0;		//操作量最大値
	outmin 	= -100.0;		//操作量最小値
	delta_outmax =  1.0;	//操作変化量最大値
	delta_outmin = -1.0;	//操作変化量最小値

	error[3] 			= {0.0,0.0,0.0};	//偏差バッファ
	inputbuf[3] 		= {0.0,0.0,0.0};	//入力バッファ
	velocity_d 			= 0.0;				//速度D項出力
	pidout[3] 			= {0.0,0.0,0.0};	//位置PID項出力
 	limited_pidout[2]	= {0.0,0.0};		//制限済み位置PID出力
	switch_i_flag 		= 0.0;				//リミッタ動作時に速度I項を無視する
-----------------------------------------------------------------------*/

/**----------------------------------------------------------------------------
<<外部公開型定義>>
-----------------------------------------------------------------------------**/

//PIDパラメータ型　システム変数
typedef struct
{
	//制御パラメータ
	float pgain;				//比例ゲイン
	float ti;					//積分時間
	float td;					//微分時間
	float dff;					//微分係数(0.1~0.125)
	float dt;					//制御周期(秒)

	//操作量リミッタ
	float outmax;				//操作量最大値
	float outmin;				//操作量最小値
	float delta_outmax;			//操作変化量最大値
	float delta_outmin;			//操作変化量最小値

	//PID内部変数
	float error[3];				//偏差バッファ
	float inputbuf[3];			//入力バッファ
	float velocity_d;			//速度D項出力
	float pidout[3];			//位置PID項出力
	float limited_pidout[2];	//制限済み位置PID出力
	float switch_i_flag;		//リミッタ動作時に速度I項を無視する

} PID_STATE_t ;


/**-----------------------------------------------------------------------------
<<外部公開プロトタイプ定義>>
------------------------------------------------------------------------------**/
/*パラメータ設定-----------------------------------------------------------*/
//制御周期設定関数　引数は(秒)
void Set_pid_dt(PID_STATE_t *pid_state_p,			//PIDパラメータ
								float dt);			//制御周期

//微分係数設定関数(0.1~0.125))
void Set_pid_dff(PID_STATE_t *pid_state_p,			//PIDパラメータ
								float dff);			//微分係数

///pidゲイン設定関数
void Set_pid_gain(PID_STATE_t *pid_state_p,			//PIDパラメータ
				  				float pgain,		//比例ゲイン
				  				float ti,			//積分時間
				  				float td);			//微分時間

//操作量限界値設定関数
void Set_pid_outlim(PID_STATE_t *pid_state_p,		//PIDパラメータ型
								float outmax,		//操作量上限値
								float outmin);		//操作量下限値

//操作変化量限界値設定関数
void Set_pid_deltaoutlim(PID_STATE_t *pid_state_p,	//PIDパラメータ
					 		float delta_outmax,		//操作増加量限界値
					 		float delta_outmin);	//操作減少量限界値

/*パラメータ読出し----------------------------------------------------------*/
//制御周期読出し関数　戻り値は(秒)
void Read_pid_dt(PID_STATE_t *pid_state_p,			//PIDパラメータ
								float *dt);			//制御周期

//微分係数読出し関数
void Read_pid_dff(PID_STATE_t *pid_state_p,			//PIDパラメータ
								float *dff);		//微分係数

//pidゲイン読み出し関数
void Read_pid_gain(PID_STATE_t *pid_state_p,		//PIDパラメータ
				  				float *pgain,		//比例ゲイン
				  				float *ti,			//積分時間
				  				float *td);			//微分時間

//操作量限界値読出し関数
void Read_pid_outlim(PID_STATE_t *pid_state_p,		//PIDパラメータ
								float *outmax,		//操作量最大値
								float *outmin);		//操作量最小値

//操作変化量限界値読出し関数
void Read_pid_deltaoutlim(PID_STATE_t *pid_state_p,	//PIDパラメータ
					  		float *delta_outmax,	//操作変化量最大値
					  		float *delta_outmin);	//操作変化量最小値

/*PID制御関数-----------------------------------------------------------*/
//実用偏差速度形ディジタルPID
float Velocitytype_pid(PID_STATE_t *pid_state_p,	//PIDパラメータ
					   			float setval,		//目標値
					   			float inputval);	//制御入力値

//実用微分先行速度形ディジタルPID
float Velocitytype_p_id(PID_STATE_t *pid_state_p,	//PIDパラメータ
					   	   		float setval,		//目標値
					   	   		float inputval);	//制御入力

//実用比例微分先行速度形ディジタルPID
float Velocitytype_i_pd(PID_STATE_t *pid_state_p,	//PIDパラメータ
					   			float setval,		//目標値
								float inputval);	//制御入力


#endif //_GTS02_PID_CONTROL_F_H_
