/*インクルード***********************************************************/
#include "lpc13xx.h"
#include "gpio.h"
#include "vs-wrc103.h"
#include "ixbus.h"
#include "xprintf.h"
#include "Drive.h"
#include "isEncoder.h"
#include "Power.h"
#include "isCamera.h"

/*触りたくない、変更がいらない、ほぼ確立した、プログラムを関数化*/
void printview( void );//terataemを使う
void every(void);//カウントなど常に行う処理
void bz(void);//bzcntを用いるための処理
void pidsel(int h);//pidを選ぶ　引数：１～３
void Xsave(int h);//距離データを記録する　引数：何個目のマーカーか	１０の－１乗ｍｍ（１センチ＝100）で換算
void datasave(int h);//コース情報を記録する　引数：何個目のマーカーか
void LRnote(void);//コースを少し進んでから左右のデーターを控える
void errormod(void);
int section(void);
int Lmakercheck(int h,int l);//左にマーカーがあるかチェックする　引数：ｈ＝何回連続で反応したか　ｌ＝前のマーカーから何１０の－１乗ｍｍたったか
int Rmakercheck(int h,int l);//右に・・・					戻り値１：センサーが反応　０：反応していない
int widetrace(void);//十字路に対応したトレース　戻り値；十字路に入ったら１、入ってから数センチも１それ以外は０
void select(int d,int l,int m);//選ぶモード、引数ｄ＝ドの音での値、ｌ＝レのおと、ｍ＝ミの音
void logg_ini(void);
void logg_write(int n);
void logg_read(void);

#define	race
//#define	trace0
//#define	trace1
//#define SCI		//シリアルモニターが必要時コメントをはずす
//#define teraterm
//#define teraterm2
//#define test

/*マクロ***********************************************************/

/*グローバル変数***********************************************************/
int	pattern = 0;
int	cntbz = 0;
int	cnt01 = 0;//周期のカウント
int	cnt02 = 0;
int	cnt03;
//int				cnt11 = 0;//使いたいタイミングでカウント
int				flag1 = 0;//フラグ 1=R,2=L
int				flag2 = 0;
int				flag3 = 0;
int 			d_flag= 0;
long			right = 0;
long			left = 0;
//int			data[400] = {};
int				linedata[40] ={};
long			linedata_x[40]={};
long			linedata_x2[40]={};
long			linedata_xl[40]={};
long			linedata_xr[40]={};
long			linedata_xl2[40]={};
long			linedata_xr2[40]={};
long			linedata_end;
long			linedata_sub;
long			linedata_sub2;
long			linedata_subsub;
long			wide_data;
long		pre_Tripmeter,gole_Tripmeter;
int				line = 0;
int				line_end;
int i;
int	j;

int t = 0;
int t2 = 0;

#define COUNT 400
int	ramFlag = 0;                	/* 1:データ記録 0:記録しない    */
int	ramAdress;			/* RAMアドレス					*/
int	ramTimer;
long logg_tripmeter[COUNT];
int logg_pattern[COUNT];
int logg_speed[COUNT];

/*メイン関数***********************************************************/
int main (void)
{
	int n = 0;
	int mode = 0;		/* スプリント:1  トレース:2*/
	int Max_speed = 0;
	int sp;
	//制御周期の設定[単位：Hz　範囲：30.0~]
	const unsigned short MainCycle = 400;
	Init(MainCycle);		//CPUの初期設定
	Encoder_ini();			//エンコーダを初期化します。使用する前にからなず読みだす必要があります。
	logg_ini();						//	ログの初期化
	initCamera(15,113);		// カメラの初期化
	//Calibration();
	BuzzerSet(90,10);
	LED(3);
	BuzzerStart();
	BuzzerStop();
	Motor(0,0);
	LED(1);

#ifdef SCI
	//シリアル通信初期化
	LED(3);
	while(!getSW());
	LED(1);
	printview();
	InitSci3(CBR_115200,non,1);
#endif
//	InitSci3(CBR_115200,non,1);
	xfunc_out = SciByteTx;

	/* LiPoの過放電チェック */
/*
	while(PowerCheck()){
		BuzzerStart();
		BuzzerSet(100,10);
		if(getSW() ) {
			Wait(500);
			break;
		}
	}
	BuzzerStop();
*/
/*****************************************************************************************/
	while(1){
		Sync();
		every();//カウント等
		bz();//ブザー処理
		Camera();//カメラ処理
		i = pid_angle;//センター値置き換え
		logg_write(1);
		//カウント処理
		cnt01++;
#ifdef race
		switch(pattern){
		case 0: /* スプリントかトレースを選択 */
			BuzzerStart();
			if(Encoder_Rvalue() > 10){
				BuzzerSet(130,10);
				LED(1);
				if(getSW()){
					pattern = 10;	/* スプリントを選択 */
					mode = 1;
					Wait(250);
					LED(0);
					Tripmeter_reset();
					BuzzerStop();
				}
			}else if(Encoder_Lvalue() > 10){
				BuzzerSet(179,10);
				LED(2);
				if(getSW()){
					pattern = 30;	/*トレースを選択 */
					mode = 2;
					Wait(250);
					LED(1);
					Tripmeter_reset();
					BuzzerStop();
				}
			}else{
				LED(0);
				BuzzerStop();
			}
		break;
/* スプリント */
		case 10: /* 速度の設定 */
			pidsel(3);
			set_Speed(mode);		/* 最高速度の設定	*/
			Wait(250);
			pattern = 11;
		break;
		case 11: /* 右スタート　左スタートの選択 */
			Tripmeter_reset();
			Wait(500);
			initCamera(15,113);
			while(1){
				BuzzerStart();
				if(Encoder_Rvalue() > Encoder_Lvalue()){
					BuzzerSet(160,10);
					flag1 = 1;
					LED(1);
				}else{
					BuzzerSet(179,10);
					flag1 = 2;
					LED(2);
				}
				if(getSW()){
					BuzzerSet(88,10);
					Wait(500);
					BuzzerStop();
					pattern = 12;
					break;
				}
			}
		break;
		case 12: /* 指でスタート時、最初がwaitにより過露光となり1となる対策　*/
			pattern = 13;
			Wait(500);
		break;
		case 13:/* スタート待ち */
			if(getSW() /*|| finger()*/) {
				pattern =15;
				if(flag1 == 1)initCamera(54,113);
				if(flag1 == 2)initCamera(15,73);
				cnt01 = 0;
//				ramFlag = 1;
				Tripmeter_reset();
			}
			if(cnt01 < 300) {
				if(flag1 == 1)	LED(1);
				else				LED(2);
			}else if(cnt01 < 400){
				if(flag1 == 1)	LED(2);
				else				LED(1);
			}else{
				cnt01 = 0;
			}
		break;
		case 15: /* しばらく直進で進む */
			if(flag1 == 1){
				Motor(60,50);
			}else{
					Motor(60,50);
			}
			LED(0);
			if(Tripmeter() > 110){					/* 当日変更あり */
				pattern = 16;
			}
		break;
		case 16: /* ラインを見つける */
			//左に置いたときよう、右に線がある
			LED(3);
			if(flag1 == 1){
				if(Center>0){
					n++;
				}else{
					n = 0;
				}
			}
			//右に置いたときよう、左に線がある
			if(flag1 == 2){
				if(0>Center){
					n++;
				}else{
					n = 0;
				}
			}
			if(n > 4){
				n = 0;
				pattern = 20;
			}
		break;
		case 20: /* スタート後 */
			//左に置いたときよう、右に線がある
			if(flag1 == 1){
				if((-5<Center)&&(Center<0)){
					pattern = 21;
					BuzzerStart();
				}
			}
			//右に置いたときよう、左に線がある
			if(flag1 == 2){
				if((0<Center)&&(Center<5)){
					pattern = 21;
					BuzzerStart();
				}
			}
			/*逆サイドにラインを発見できないとき */
			if(Center == 0){
				n++;
			}
			if(n > 20)pattern = 21;
		break;
		case 21: /* 線の発見後 */
			LED(3);
			//左に置いたときよう、右に線がある
			if(flag1 == 1){
				Motor(0,75);
				if((0<Center)&&(Center<5)){
					initCamera(15,113);
					pattern = 23;
					BuzzerStop();
				}
			}
			//右に置いたときよう、左に線がある
			if(flag1 == 2){
				Motor(75,0);
				if((-5<Center)&&(Center<0)){
					pattern = 23;
					initCamera(15,113);
					BuzzerStop();
				}
			}
		break;
		case 22: /*	Kpを強めで直進 */
			LED(3);
			Run(60,pid_angle);
			if(Tripmeter() > 600){
				pidsel(1);
				pattern = 23;
			}
		break;
		case 23: /* 直進 */
			LED(0);
			Run(100,pid_angle);
			if(Now_speed > Max_speed)Max_speed = Now_speed;
			if(Tripmeter() >5000){														/* 当日変更あり */
				ramFlag = 1;
				cnt01 = 0;
				pattern = 24;
			}
		break;
		case 24: /* 減速してエンドポイントを探す */
			sp = Max_speed - cnt01 * 40;
			if(sp < 1000) sp = 1000;
//			Run(	speed_Control(sp),pid_angle);
			Run(80,pid_angle);
			if( Wide== 0  || Wide > 50 ){
				gole_Tripmeter = Tripmeter();
				cnt01 = 0;
//				ramFlag = 1;
				Run(-50,0);
				pattern = 71;
			}
		break;
/* トレース */
		case 30: /* 速度の設定 */
			pidsel(2);
			set_Speed(mode);		/* 最高速度の設定	*/
			Wait(500);
			pattern = 31;
		break;
		case 31:/* スタート待ち */
			if(getSW()) {
				pattern = 40;
				Tripmeter_reset();
			}
			if(cnt01 < 100) {
				LED(1);
			}else if(cnt01 < 200){
				LED(2);
			}else{
				cnt01 = 0;
			}
		break;
		case 32: /* スタートマーカー まで*/
			LED(0);
			Run(70,pid_angle);
			if(isRmaker()) {
				Tripmeter_reset();
				ramFlag = 1;
				pattern = 40;
			}
		break;
		case 40: /* 通常トレース */
			LED(3);
			if(pid_angle > 0){
				Run(speed_Control(speed - 2*pid_angle * curve_speed), pid_angle);
			}else{
				Run(speed_Control(speed + 2*pid_angle * curve_speed),pid_angle);
			}

			if(Tripmeter() > 19000 /*&& isRmaker()*/){
				gole_Tripmeter = Tripmeter();
				cnt02 = 0;
				cnt03 = 0;
				pattern = 710;
			}

		break;
/* 停止は共通 */
		case 70: /* 停止 */
/*
			if(Now_speed  > 5){
				Run(-100,0);
				cnt02 = cnt03 = 0;
			}else if(Now_speed < 0){
				if(speed >70)	Run(-15,0);
				else				Run(0,0);
			}else if(Now_speed == 0){
				cnt03++;
				if(cnt03 > 10)pattern = 71;
			}else{
				if(speed >70)	Run(-100,0);
				else				Run(0,0);
				cnt02++;
				if(cnt02 > 50){
					pattern = 71;
				}
			}
*/
			if(Tripmeter() - pre_Tripmeter > 0){
//				Run(-100,0);
				Motor(-70,-100);
				cnt02 = cnt03 = 0;
			}else if(Tripmeter() - pre_Tripmeter < 0){
				Run(-15,0);
			}else{
				Run(0,-5);
				cnt03++;
				if(cnt03 > 50)pattern = 71;
			}
			pre_Tripmeter = Tripmeter();

		break;
		case 71:
//			if(mode == 2){
				if( Tripmeter() - gole_Tripmeter < 300){
					Run(20,0);
				}else{
					pattern = 72;
				}
//			}else{
//				pattern = 72;
//			}
		break;
		case 710:
			Run(0,pid_angle);
		break;
		case 72:
			Run(0,0);
			LED(0);
			ramFlag = 0;
			BuzzerStart();
			BuzzerSet(160,10);
			pattern = 100;

/*
			if(mode == 1){
				BuzzerStart();
				BuzzerSet(160,10);
				pattern = 100;
			}else{
				Tripmeter_reset();
				pattern = 30;
			}
*/
		break;
		case 100: /* loggの読み出し */
			Run(0,0);
			while(!getSW());
			BuzzerStop();
			LED(1);
			logg_read();
		break;
		}
#endif
	}
}
void printview( void )
{
	//制御周期の設定[単位：Hz　範囲：30.0~]
	//シリアル通信初期化
	InitSci3(CBR_115200,non,1);

	xfunc_out = SciByteTx;
	xprintf("hello");
	//ループ
	while(1){
		LED(2);
		Sync();

		Camera();
		i = pid_angle;
		LED(1);
//		strightRun(40);
		cnt01++;
		if(cnt01 > 1000){
				#ifdef SCI
					if(getSW())graph_view();
					else	bi_view();
					xprintf("PID = %d\n",i);
				//}
				#endif
					xprintf("finger = %d getSW = %d isRmaker=%d (%d)\n",finger(),getSW(),isRmaker(),ADRead(2));
					isEncoder_monitor();
					xprintf("Lvalue  %d Rvalue  %d \n",Lvalue,Rvalue);
					xprintf("PowerValue %d\n",PowerValue());
			cnt01 = 0;
		}

	//		SciByteTx(t);
	}
}

void bz(void)
{
	if(cntbz >1){
		cntbz--;
		BuzzerSet(90,250);
		BuzzerStart();

	}else if(cntbz >0){
		BuzzerStop();
	}
}

void Xsave(int h)
{
	linedata_x[h] = Tripmeter();
	linedata_xl[h] = TripmeterLeft();
	linedata_xr[h] = TripmeterRight();
}
int widetrace(void)
{
	if(Wide > 40){
		Motor(speed,speed);
		wide_data = Tripmeter();
		return 1;
	}else if(Tripmeter() - wide_data < 200){
		Motor(speed,speed);
		return 1;
	}else if(Wide == 0){
		if(flag1 == 1){
			Motor(speed,-speed);
		}else{
			Motor(-speed,speed);
		}
		return 0;
	}else{
		Motor(speed-i,speed+i);
		if(maker == 0){
			if(Center > 0)	flag1 = 1;
			else			flag1 = 2;
		}
		return 0;
	}
}
int Lmakercheck(int h,int l)
{
	if((Tripmeter()> l) && (maker == 2)){
		t2++;
		if(t2 > h)	return 1;
		else		return 0;
	}else{
		t2 = 0;
		return 0;
	}
}
int Rmakercheck(int h,int l)
{
	if((Tripmeter()> l) && (maker == 1)){
		t++;
		if(t > h)	return 1;
		else		return 0;
	}else{
		t = 0;
		return 0;
	}
}
void LRnote(void)
{
	left = TripmeterLeft();
	right = TripmeterRight();
}
void datasave(int h)
{

	if(linedata_x[h] - linedata_x[h-1] > 15000){
		left = linedata_xl[h] - left;
		right = linedata_xr[h] - right;
		if(left - right > 1500){
			linedata[h] = 0x2;
			linedata[h-1] |= 0x1;
		}else if(right - left > 1500){
			linedata[h] = 0x2;
			linedata[h-1] |= 0x1;
		}else{
			linedata[h] = 0x2;
		}
	}else if(linedata_x[h] - linedata_x[h-1] > 2000){
		left = linedata_xl[h] - left;
		right = linedata_xr[h] - right;
		if(left - right > 1500){
			linedata[h] = 0x1;
			linedata[h-1] |= 0x1;
		}else if(right - left > 1500){
			linedata[h] = 0x1;
			linedata[h-1] |= 0x1;
		}else{
			linedata[h] = 0x2;
		}
	}else{
		linedata[h] = 0x1;
		linedata[h-1] |= 0x1;
	}
	if(linedata[h] == 0x1){
		linedata[h-1] |= 0x1;
	}
	linedata_xl2[h] =left;
	linedata_xr2[h] =right;
}
void select(int d,int l,int m)
{
	if(cnt01 > 1600){
		cnt01 = 0;
	}else if(cnt01 > 800){
		LED(2);
		speed_sub = m;

		speed_slow = m*7/10;
		BuzzerSet(142,10);
	}else if(cnt01 > 400){
		LED(1);
		BuzzerSet(160,10);
		speed_sub = l;

		speed_slow = l*2/3;
	}else {

		LED(3);
		BuzzerSet(179,10);
		speed_sub = d;

		speed_slow = d/2;
	}
}
void errormod(void)
{
	long sub=0;
	long error=0;
	int su = 0;
	error = Tripmeter() - linedata_x2[0];
	if(error < 0)error = -error;
	for(j=1;j<=line_end;j++){
		sub =  Tripmeter() - linedata_x2[j];
		if(sub < 0)sub = -sub;
		if(error > sub){
			error = sub;
			su = j;
		}
	}
	error = Tripmeter() - linedata_x2[su];
	for(j=0;j<=line_end;j++){
		if(linedata_x2[j] > Tripmeter()){
			linedata_x2[j] += error;
		}
	}
}
int section(void)
{
	for(j=0;j<=line_end;j++){
		if(j == 0){
			if((0 < Tripmeter())&&(Tripmeter() < linedata_x2[0])){
				break;
			}
		}else{
			if((linedata_x2[j-1] < Tripmeter())&&(Tripmeter() < linedata_x2[j])){

				break;
			}
		}
	}
	linedata_sub =  linedata_x2[j];
	return j;
}
/****************************/
/* loggの初期化　	*/
/*　引数　なし				 	*/
/* 戻り値　なし 		*/
/****************************/
void logg_ini(void)
{
	int i;

	ramFlag = 0;
	ramAdress = 0;
	ramTimer = 0;
	for(i = 0; i < COUNT; i++){
		logg_tripmeter[i] = 0;
		logg_pattern[i] = 0;
	}

}
/****************************/
/* loggの書き込み　	*/
/*　引数　間隔				 	*/
/* 戻り値　なし 		*/
/****************************/
void logg_write(int n)
{
	if(ramFlag == 1){
		ramTimer++;
		if( ramTimer >=n){
			logg_tripmeter[ramAdress] = Tripmeter();
			logg_pattern[ramAdress] =pattern;
//			logg_speed[ramAdress] = Wide;
			logg_speed[ramAdress] = Now_speed;
			ramAdress++;
			ramTimer = 0;
			if(ramAdress > COUNT){
				ramFlag = 0;
			}
		}
	}
}
/****************************/
/* loggの読み出し　	*/
/*　引数　なし				 	*/
/* 戻り値　なし 		*/
/****************************/
void logg_read(void)
{
	int i;
	//制御周期の設定[単位：Hz　範囲：30.0~]
	//シリアル通信初期化
	InitSci3(CBR_115200,non,1);

	xfunc_out = SciByteTx;
	xprintf("hello");

	xprintf("tripmeter,pattern\n");
	xprintf("gole_Tripmeter = %d\n",gole_Tripmeter);
	xprintf("speed = %d\n",speed);
	for(i = 0; i < COUNT; i++){
		xprintf("%ld,%d,%d\n",logg_tripmeter[i],logg_pattern[i],logg_speed[i]);
	}

}
