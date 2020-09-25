//
//  Observer.cpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "Observer.hpp"
#include "StateMachine.hpp"

// global variables to pass FIR-filtered color from Observer to Navigator and its sub-classes
rgb_raw_t g_rgb;
hsv_raw_t g_hsv;
int16_t g_grayScale, g_grayScaleBlueless;
// global variables to gyro sensor output from Observer to  Navigator and its sub-classes
int16_t g_angle, g_anglerVelocity;
int16_t g_challenge_stepNo, g_color_brightness;


Observer::Observer(Motor* lm, Motor* rm, Motor* am, Motor* tm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor = am;
    tailMotor = tm;
    touchSensor = ts;
    sonarSensor = ss;
    gyroSensor  = gs;
    colorSensor = cs;

    distance = azimuth = locX = locY = 0.0;
    prevAngL = prevAngR = 0;
    integD = integDL = integDR = 0.0; // temp

    notifyDistance = 0;
    traceCnt = 0;
    prevGS = INT16_MAX;
    touch_flag = false;
    sonar_flag = false;
    backButton_flag = false;
    lost_flag = false;
    blue_flag = false;
    
    // For Challenge Run
    line_over_flg = false;
    move_back_flg = false;
    slalom_flg = false;
    curRgbSum = 0;
    prevRgbSum = 0;
    curAngle = 0;
    prevAngle = 0;
    curDegree = 0;
    prevDegree = 0;
    cntDegree = 0;
    g_challenge_stepNo = 0;
    printf("初期化された\n");
    prevDis = 0;
    prevDisX = 0;
    prevDisY = 0;
    roots_no = 0;
    turnDegree=0;
    gyroSensor->setOffset(0);

    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    ma = new MovingAverage<int32_t, MA_CAP>();
}

void Observer::activate() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler set", clock->now()));
}

void Observer::reset() {
    distance = azimuth = locX = locY = 0.0;
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
    integD = integDL = integDR = 0.0; // temp
}

void Observer::notifyOfDistance(int32_t delta) {
    notifyDistance = delta + distance;
}

int32_t Observer::getDistance() {
    return (int32_t)distance;
}

int16_t Observer::getAzimuth() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    return degree;
}

int16_t Observer::getDegree() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    if (degree > 180){
        degree -= 360;
    }
    return degree;
}

int32_t Observer::getLocX() {
    return (int32_t)locX;
}

int32_t Observer::getLocY() {
    return (int32_t)locY;
}

void Observer::operate() {
    colorSensor->getRawColor(cur_rgb);
    // process RGB by the Low Pass Filter
    cur_rgb.r = fir_r->Execute(cur_rgb.r);
    cur_rgb.g = fir_g->Execute(cur_rgb.g);
    cur_rgb.b = fir_b->Execute(cur_rgb.b);
    curRgbSum = cur_rgb.r + cur_rgb.g + cur_rgb.b;
    rgb_to_hsv(cur_rgb, cur_hsv);
    // save filtered color variables to the global area
    g_rgb = cur_rgb;
    g_hsv = cur_hsv;
    // calculate gray scale and save them to the global area
    if(!garage_flg){
        g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 150 + cur_rgb.b * 29) / 256;
        g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 150 + (cur_rgb.b - cur_rgb.g) * 29) / 256; // B - G cuts off blue
    }else{
        g_grayScale = (cur_rgb.r * 200 + cur_rgb.g * 10 + cur_rgb.b * 29) / 239;
        g_grayScaleBlueless = (cur_rgb.r * 200 + cur_rgb.g * 10 + (cur_rgb.b - cur_rgb.g) * 29) / 239; // B - G cuts off blue
        g_color_brightness = colorSensor->getBrightness();
    }

    // save gyro sensor output to the global area
    g_angle = gyroSensor->getAngle();
    g_anglerVelocity = gyroSensor->getAnglerVelocity();

    // accumulate distance
    int32_t curAngL = leftMotor->getCount();
    int32_t curAngR = rightMotor->getCount();
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    distance += deltaDist;
    prevAngL = curAngL;
    prevAngR = curAngR;
    // calculate azimuth
    double deltaAzi = atan2((deltaDistL - deltaDistR), WHEEL_TREAD);
    azimuth += deltaAzi;
    if (azimuth > M_2PI) {
        azimuth -= M_2PI;
    } else if (azimuth < 0.0) {
        azimuth += M_2PI;
    }
    // estimate location
    locX += (deltaDist * sin(azimuth));
    locY += (deltaDist * cos(azimuth));

    // monitor distance
    if ((notifyDistance != 0.0) && (distance > notifyDistance)) {
        syslog(LOG_NOTICE, "%08u, distance reached", clock->now());
        notifyDistance = 0.0; // event to be sent only once
        stateMachine->sendTrigger(EVT_dist_reached);
    }
    
    // monitor touch sensor
    bool result = check_touch();
    if (result && !touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now());
        touch_flag = true;
        stateMachine->sendTrigger(EVT_touch_On);
    } else if (!result && touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now());
        touch_flag = false;
        stateMachine->sendTrigger(EVT_touch_Off);
    }
    
    // monitor sonar sensor
    result = check_sonar();
    if (result && !sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now());
        sonar_flag = true;
        stateMachine->sendTrigger(EVT_sonar_On);
    } else if (!result && sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now());
        sonar_flag = false;
        stateMachine->sendTrigger(EVT_sonar_Off);
    }
    
    // monitor Back Button
    result = check_backButton();
    if (result && !backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now());
        backButton_flag = true;
        stateMachine->sendTrigger(EVT_backButton_On);
    } else if (!result && backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now());
        backButton_flag = false;
        stateMachine->sendTrigger(EVT_backButton_Off);
    }

    if (!frozen) { // these checks are meaningless thus bypassed when frozen
        // determine if still tracing the line
        // result = check_lost();
        // if (result && !lost_flag) {
        //     syslog(LOG_NOTICE, "%08u, line lost", clock->now());
        //     lost_flag = true;
        //     stateMachine->sendTrigger(EVT_line_lost);
        // } else if (!result && lost_flag) {
        //     syslog(LOG_NOTICE, "%08u, line found", clock->now());
        //     lost_flag = false;
        //     stateMachine->sendTrigger(EVT_line_found);
        // }

        // temporary dirty logic to detect the second black to blue change
        int32_t ma_gs;
        if (prevGS == INT16_MAX) {
            prevTime = clock->now();
            prevGS = g_grayScale;
            ma_gs = ma->add(0);
        } else {
            curTime = clock->now();
            gsDiff = g_grayScale - prevGS;
            timeDiff = curTime - prevTime;
            ma_gs = ma->add(gsDiff * 1000000 / timeDiff);
            prevTime = curTime;
            prevGS = g_grayScale;
        }

        if ( (ma_gs > 150) || (ma_gs < -150) ){
            //syslog(LOG_NOTICE, "gs = %d, MA = %d, gsDiff = %d, timeDiff = %d", g_grayScale, ma_gs, gsDiff, timeDiff);
            if ( !blue_flag && ma_gs > 150 && g_rgb.b - g_rgb.r > 60 && g_rgb.b <= 255 && g_rgb.r <= 255 ) {
                blue_flag = true;
                syslog(LOG_NOTICE, "%08u, line color changed black to blue", clock->now());
                stateMachine->sendTrigger(EVT_bk2bl);
            } else if ( blue_flag && ma_gs < -150 && g_rgb.b - g_rgb.r < 40 ) {
                blue_flag = false;
                syslog(LOG_NOTICE, "%08u, line color changed blue to black", clock->now());
                stateMachine->sendTrigger(EVT_bl2bk);
            }
        }

        // determine if tilt
        // if ( check_tilt() ) {
        //     stateMachine->sendTrigger(EVT_tilt);
        // }
    }
    
    curDegree = getDegree();
    sonarDistance = sonarSensor->getDistance();

    // Preparation for slalom climbing
    if(!slalom_flg && !garage_flg){
        if (g_challenge_stepNo == 0 && sonarDistance >= 1 && sonarDistance <= 10 && !move_back_flg){
            state = ST_slalom;
            armMotor->setPWM(-50);
            stateMachine->sendTrigger(EVT_slalom_reached);
            g_challenge_stepNo = 1;

            locX = 0;
            locY = 0; 
            distance = 0;
            prevAngL =0;
            prevAngR =0;
            leftMotor->reset();
            rightMotor->reset();
            azimuth = 0;

            stateMachine->sendTrigger(EVT_slalom_reached);
            armMotor->setPWM(60);
            g_challenge_stepNo = 10;
            move_back_flg = true;
        }

        // Robot tilts whenn
        curAngle = g_angle;
        if(curAngle < -9){
            prevAngle = curAngle;
        }
        if (prevAngle < -9 && curAngle >= 0){
            printf("スラロームオン\n");
            slalom_flg = true;
            curAngle = 0;
            prevAngle = 0;
            prevDis = distance;
            prevDisY = locY;
            prevDegree=getDegree();
            armMotor->setPWM(-100);
        }
    }

    //スラローム専用処理
    if(slalom_flg && !garage_flg){
        //ログ出力
        // if (g_challenge_stepNo >= 0 && g_challenge_stepNo <= 200){
        //      if (++traceCnt && traceCnt > 50) {
        //          printf(",locX=%lf,locY=%lf,distance=%lf,prevDis=%lf,g_angle=%d,curDegree=%d, sonarDistance=%d, g_challenge_stepNo=%d,r+g+b=%d\n",locX,locY,distance,prevDis,g_angle,curDegree, sonarDistance,g_challenge_stepNo,curRgbSum);
        //          traceCnt = 0;
        //      }
        // }

        //初期位置の特定
        if(g_challenge_stepNo == 10 && distance - prevDis > 30){
            prevDisX = locX;
            prevRgbSum = curRgbSum;
            prevRgbSum = curRgbSum;
            if(curRgbSum < 100){
                //もともと黒の上にいる場合、ライン下方面に回転
                prevDisX = locX;
                g_challenge_stepNo = 20;
            }else{
                //左カーブ
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 11;
            }
        }else if(g_challenge_stepNo == 11){
            //センサーで黒を検知した場合
            if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                g_challenge_stepNo = 20;
            }
            //左カーブで黒を見つけられなかった場合、リバース、元の位置まで戻る
            else if(own_abs(prevDisX-locX) > 140 ){
                if(prevRgbSum - curRgbSum > 20){
                    //黒に近づいていたら、もう少し頑張る
                }else{
                    //黒の片鱗も見えなければ逆走する
                    stateMachine->sendTrigger(EVT_slalom_challenge);
                    g_challenge_stepNo = 12;
                }
            }
        //左カーブから戻る
        }else if(g_challenge_stepNo == 12){
            if(prevDisX - locX <= 0){
                //右カーブへ移行
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 13;
            
            //途中で黒を検知した場合、左下へ移動
            }else if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                g_challenge_stepNo = 20;
            }

        }else if(g_challenge_stepNo == 13){
            //センサーで黒を検知した場合
            if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 20;
            }        

        }else if(g_challenge_stepNo == 20 && curDegree >= -57){
            //その場で左回転
            g_challenge_stepNo = 21;
            stateMachine->sendTrigger(EVT_slalom_challenge);//21


        }else if(g_challenge_stepNo == 20 && curDegree < -57){
            //その場で右回転
            g_challenge_stepNo = 22;
            stateMachine->sendTrigger(EVT_slalom_challenge);//22

        }else if((g_challenge_stepNo == 21 && curDegree <= -57) || (g_challenge_stepNo == 22 && curDegree >= -63)){
            //角度が一定角度になったら、停止、直進
                g_challenge_stepNo = 23;
                stateMachine->sendTrigger(EVT_slalom_challenge); //23
                g_challenge_stepNo = 24;

        } else if(g_challenge_stepNo == 24){
            if(own_abs(prevDisX - locX) > 120){
                //進行方向に対して横軸に距離進んだら、角度を0度に戻す
                stateMachine->sendTrigger(EVT_slalom_challenge); //24
                g_challenge_stepNo = 30;
            }
        }else if(g_challenge_stepNo == 30){
            if(curDegree > prevDegree - 2){
                //進行方向に対して横軸に進んだら、角度を戻す
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 40;
            }
        // ２つ目の障害物に接近したら向きを変える
        }else if (g_challenge_stepNo == 40){
            if(locY - prevDisY > 554 || check_sonar(0,5)){
                printf(",２つ目の障害物に接近したら向きを変える\n");
                g_challenge_stepNo = 41;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 50;
                prevRgbSum = curRgbSum;
                line_over_flg = false;
                prevRgbSum = curRgbSum;
            }
        // 視界が晴れたら左上に前進する
        }else if(g_challenge_stepNo == 50  && (check_sonar(50,255)|| own_abs(curDegree-prevDegree) > 55)){
            printf(",視界が晴れたところで前進する");
            stateMachine->sendTrigger(EVT_slalom_challenge);
            g_challenge_stepNo = 60;
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整し３つ目の障害物に接近する
        }else if (g_challenge_stepNo == 60 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
            }
            if(prevRgbSum < 100 && curRgbSum > 125){
                printf(",黒ラインを超えたら向きを調整し障害物に接近する\n");
                stateMachine->sendTrigger(EVT_slalom_challenge);
                //prevDis=distance;
                prevDisX = locX;
                g_challenge_stepNo = 61;
                prevDegree = curDegree;
                line_over_flg = true;
                g_challenge_stepNo = 71;
            }
        }else if(g_challenge_stepNo == 71 && check_sonar(0,5)){
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 80;
                line_over_flg = true;
                prevDegree = curDegree;
        // 視界が晴れたら左下に前進する
        }else if (g_challenge_stepNo == 80  && own_abs(prevDegree - curDegree) > 65){
                printf(",視界が晴れたら左下に前進する\n");
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 90;
                prevRgbSum = curRgbSum;
                line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整する
        }else if (g_challenge_stepNo == 90 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
            }
            if(prevRgbSum < 100 && curRgbSum > 150){
                printf(",黒ラインを超えたら向きを調整する\n");
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 100;
                line_over_flg = true;
            }
            prevDegree = curDegree;
        // ４つ目の障害物に接近する
        }else if(g_challenge_stepNo == 100 && check_sonar(0,30) && own_abs(curDegree - prevDegree) > 55){ //
            printf(",４つ目の障害物に接近する。locY=%lf,prevDisY=%lf,curDegree=%d\n",locY,prevDisY,curDegree);
            stateMachine->sendTrigger(EVT_slalom_challenge);
            g_challenge_stepNo = 110;
        // ４つ目の障害物に接近したら向きを変える
        }else if (g_challenge_stepNo == 110 && check_sonar(0,5)){
            printf(",４つ目の障害物に接近したら向きを変えるlocY=%lf,prevDisY=%lf,curDegree=%d\n",locY,prevDisY,curDegree);
            stateMachine->sendTrigger(EVT_slalom_challenge);
            g_challenge_stepNo = 111;
            prevDegree = curDegree;
        
        //step110直後に視界が晴れる可能性を回避
        }else if(g_challenge_stepNo == 111 && own_abs(curDegree - prevDegree) > 40){
            g_challenge_stepNo = 120;
        
        // 視界が晴れたら左上に前進する
        }else if (g_challenge_stepNo == 120 &&  (check_sonar(21,255) || own_abs(curDegree - prevDegree) > 70)){
            printf(",視界が晴れたら左上に前進する\n");
            stateMachine->sendTrigger(EVT_slalom_challenge);
            g_challenge_stepNo = 130;
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整する
        }else if (g_challenge_stepNo == 130){
            if (!line_over_flg){
                if (curRgbSum < 100) {
                    prevRgbSum = curRgbSum;
                }
                if(prevRgbSum < 100 && curRgbSum > 160){
                    line_over_flg = true;
                }
            }else if (curRgbSum < 60 && line_over_flg) {
                printf(",黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整する\n");
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 140;
            }
            prevDegree = curDegree;
        // 直進しスラロームを降りる
        }else if(g_challenge_stepNo == 140 && own_abs(curDegree - prevDegree) > 48 ){
            printf(",直進しスラロームを降りる\n");
            stateMachine->sendTrigger(EVT_slalom_challenge);
            armMotor->setPWM(30);
            g_challenge_stepNo = 150;
        }

        // スラローム降りてフラグOff
        if(g_angle > 6 && g_challenge_stepNo == 150){
            printf("スラロームオフ\n");
            state = ST_block;
            slalom_flg = false;
            garage_flg = true;
            locX = 0; //初期化
            locY = 0; //初期化 
            distance = 0;//初期化
            prevAngL =0; //初期化 
            prevAngR =0; //初期化 
            azimuth = 0; //初期化 
            leftMotor->reset(); //初期化 
            rightMotor->reset(); //初期化 
            armMotor->setPWM(-100); //初期化 
            curAngle = 0;//初期化
            prevAngle = 0;//初期化
        }
    }

    //ボーナスブロック＆ガレージ専用処理
    if (garage_flg && !slalom_flg){
    //    if (g_challenge_stepNo >= 151 && g_challenge_stepNo <= 300){
    //         if (++traceCnt && traceCnt > 30) {
    //             printf(",garage_flg=%d,slalom_flg=%d,distance=%lf,g_angle=%d,curDegree=%d, sonarDistance=%d, g_challenge_stepNo=%d,r=%d,g=%d,b=%d\n",garage_flg,slalom_flg,distance,g_angle,curDegree, sonarDistance,g_challenge_stepNo,cur_rgb.r,cur_rgb.g,cur_rgb.b);
    //             traceCnt = 0;
    //         }
    //    }

        //ガレージON後、距離が一部距離が残るようであるため、距離がリセットされたことを確認
        if(g_challenge_stepNo == 150 && distance < 100 && distance > 1){
            g_challenge_stepNo = 151;
        }

        else if( g_challenge_stepNo == 151 && distance > 145){
            // ソナー稼働回転、方向を調整
            printf("ソナー稼働回転、方向を調整\n");
            stateMachine->sendTrigger(EVT_block_challenge); //151
            g_challenge_stepNo = 160;

        //ソナーの値から進行方向確認
        }else if(g_challenge_stepNo == 160 && check_sonar(255,255)){
            prevDegree = getAzimuth();
            g_challenge_stepNo = 170;
        }

        //升目ライン方向へ進行
//        else if(g_challenge_stepNo == 170 && own_abs(getAzimuth() - prevDegree) > 3){
        else if(g_challenge_stepNo == 170 ){
            printf("升目ライン方向へ進行\n");
            prevDis = distance;
            // 升目ラインに接近
            stateMachine->sendTrigger(EVT_block_challenge); //170
            clock->sleep(1000);//スラローム後、大きくラインを左に外しても、手前の黒ラインに引っかからないためにスリープ
            g_challenge_stepNo = 180;
        }

        //升目ラインの大外枠とクロス。黒、赤、黄色の３パターンのクロスがある
        else if(g_challenge_stepNo == 180){
        
            //黒を見つけたら、下向きのライントレース
            if(cur_rgb.g + cur_rgb.b <= 100 && cur_rgb.r <=50 && cur_rgb.g <=40 && cur_rgb.b <=60){
                printf("黒を見つけたら、下向きのライントレース\n");
                g_challenge_stepNo = 190;
                stateMachine->sendTrigger(EVT_block_challenge);//190
                g_challenge_stepNo = 191;
            }
            //赤を見つけたら、赤からブロックへ  直進
            else if(cur_rgb.r - cur_rgb.b >=40 && cur_rgb.g <65 && cur_rgb.r - cur_rgb.g >30){
                printf("赤を見つけたら、赤からブロックへ  直進\n");
                g_challenge_stepNo = 200;
                stateMachine->sendTrigger(EVT_block_challenge); //200
                g_challenge_stepNo = 201;
                //captain->decide(EVT_turnRight2); // ここで物体へターン
                prevDegree = getAzimuth();
                printf("赤色超えました\n");
            }
            
            //黄色を見つけたら、右に直進のライントレース
            else if(cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 &&  cur_rgb.r - cur_rgb.g <= 30){
                printf("黄色を見つけたら、右に直進のライントレース\n");
                g_challenge_stepNo = 210;
                stateMachine->sendTrigger(EVT_block_challenge); //210
                //captain->decide(EVT_turn_right_180); // ちょっとだけ外へずらす 
                g_challenge_stepNo = 211;

                clock->sleep(800);
                stateMachine->sendTrigger(EVT_block_challenge); //211
                //captain->decide(EVT_go_b3); // 物体に接近
                g_challenge_stepNo = 212;
                
                printf("黄色超えました1\n");
                prevDis = distance;
            }
         //黒ライン進入の続き
        }else if(g_challenge_stepNo == 191){

                stateMachine->sendTrigger(EVT_line_on_p_cntl); //191
                //captain->decide(EVT_go_line_p); // 比例制御ライントレース
                g_challenge_stepNo = 220;
                prevDegree = getAzimuth();
                roots_no = 1;
        //赤ライン進入の続き
        }else if(g_challenge_stepNo == 201 && getAzimuth() - prevDegree >93){
                printf("赤色超えました2\n");
                stateMachine->sendTrigger(EVT_block_challenge); //201
                //captain->decide(EVT_go_30); // 物体に接近
                g_challenge_stepNo = 250;
                roots_no = 2;
        //黄色ライン進入の続き
        }else if(g_challenge_stepNo==212 && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130 && distance-prevDis>30){
                printf("黄色超えました23\n");
                stateMachine->sendTrigger(EVT_line_on_pid_cntl); //212
                //captain->decide(EVT_go_line_t); // 物体に接近
                g_challenge_stepNo = 250;
        
        //黒ラインからの黄色を見つけたらブロック方向へターン
        }else if(g_challenge_stepNo ==220 && cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 &&  cur_rgb.r - cur_rgb.g <= 30){
            int x = getAzimuth();
            printf("ここのprevDegree=%d,azi=%d,sa=%d\n",prevDegree,x,prevDegree-x);
            //prevDegreeは黒ライン侵入時、回転後のもの
            cntDegree = prevDegree - x ;
            prevDegree = getAzimuth();
            g_challenge_stepNo = 230;
            stateMachine->sendTrigger(EVT_block_area_in); //230
            g_challenge_stepNo = 231;
            //captain->decide(EVT_step); // 一旦停止
            //captain->decide(EVT_turnLeft_m15p50); // 回転開始

        }else if(g_challenge_stepNo==231 && prevDegree - getAzimuth() + cntDegree > 67 + 26){
            prevDis = distance;
            stateMachine->sendTrigger(EVT_block_challenge); //231
            //captain->decide(EVT_go_30); // 物体に接近
            g_challenge_stepNo = 232;
            printf("ここまで黄色エリア１ cntDegree=%d,azi=%d,sa=%d,gosa=%d\n",prevDegree,getAzimuth(),prevDegree -getAzimuth(),cntDegree);
        
        
        }else if(g_challenge_stepNo == 232 && distance - prevDis > 180 && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130 ){
            printf("ここまで黄色エリア２\n");
            stateMachine->sendTrigger(EVT_line_on_pid_cntl); //232
            g_challenge_stepNo = 250;
            //captain->decide(EVT_go_line_t); // 物体に接近
        
        //赤を見つけたら黒を見つけるまで直進、その後ライントレース
        }else if(g_challenge_stepNo == 220 && cur_rgb.r - cur_rgb.b >= 40 && cur_rgb.g < 60 && cur_rgb.r - cur_rgb.g > 30){
            g_challenge_stepNo = 240;
            //直前までライントレース
            stateMachine->sendTrigger(EVT_block_area_in); //240
            g_challenge_stepNo = 241;
            //captain->decide(EVT_go_30); // 物体に接近
        
        //赤を離脱するために以下の分岐にあるカラーを順番にたどる
        }else if( cur_rgb.r - cur_rgb.b < 20 && g_challenge_stepNo == 241){
            g_challenge_stepNo = 242;
        }else if( cur_rgb.r - cur_rgb.b >=40 && g_challenge_stepNo == 242){
            g_challenge_stepNo = 243;
        
        //赤を通過時、大きくラインを外れたら、カーブして戻る
        }else if(g_challenge_stepNo == 243 && cur_rgb.r + cur_rgb.g + cur_rgb.b > 300){
            g_challenge_stepNo = 244;
            stateMachine->sendTrigger(EVT_block_challenge); //244
            g_challenge_stepNo = 245;
            //captain->decide(EVT_turn_180); // ラインに戻る
            clock->sleep(10);
            stateMachine->sendTrigger(EVT_line_on_pid_cntl); //245
            //captain->decide(EVT_go_line_t); // 黄色に向かう
            g_challenge_stepNo = 220;

        //ラインを外れていなければ、黒のライントレースへ
        }else if(g_challenge_stepNo == 243 && cur_rgb.r + cur_rgb.g + cur_rgb.b <= 100){
            g_challenge_stepNo = 246;
            stateMachine->sendTrigger(EVT_line_on_pid_cntl); //246 
            //captain->decide(EVT_go_line_t); // 黄色に接近
            g_challenge_stepNo = 220;
        
        }else if(g_challenge_stepNo==250){                
            //ブロックに直進、ブロックの黄色を見つけたら
            if(cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160){
                    printf("ブロックGETしてほしい\n");                 
                    g_challenge_stepNo = 260;
            }
        //ガレージに戻るべく、ターン
        }else if(g_challenge_stepNo == 260){
            //走行体回頭
            stateMachine->sendTrigger(EVT_block_area_in); //260
            //captain->decide(EVT_turn_180);
            prevDegree = getAzimuth();
            cntDegree = 0;
            g_challenge_stepNo = 261;

            //赤〇ポイントからのブロック到達の場合
            if(roots_no==2){ 
                //回転角度に140度をセット
                turnDegree = 140;
            //黒ライン、黄色直進からのブロック到達の場合
            }else{
                //回転角度に80度をセット
                turnDegree = 100;
            }
        }else if(g_challenge_stepNo == 261 && cntDegree > turnDegree){
            stateMachine->sendTrigger(EVT_block_challenge); //261
            //captain->decide(EVT_turn_180_slowly); // 回頭中
            g_challenge_stepNo = 262;

        }else if(g_challenge_stepNo == 261){ //角度が条件に該当しない場合、差分を累積していく。
            int x= getAzimuth();
            cntDegree += getTurnDgree(prevDegree,x);
            prevDegree = x;
            //printf("cntDegree=%d,prevDegree=%d,",cntDegree,prevDegree);
        //前方に何もない状況になったら進行
        }else if(g_challenge_stepNo == 262){
            if(check_sonar(255,255)){
                g_challenge_stepNo++;
                cntDegree = 0;
                prevDegree = getAzimuth();
            }
        }else if(g_challenge_stepNo == 263){
            int x= getAzimuth();
            cntDegree += getTurnDgree(prevDegree,x);
            prevDegree = x;

            if(cntDegree >= 5){ //255を発見してから何度曲がるかを調整
                stateMachine->sendTrigger(EVT_block_challenge); //263
                //captain->decide(EVT_go_b3); // ガレージラインへ
                g_challenge_stepNo = 270;
                prevDis = distance;
            }
            //printf("prevDegree=%d,",prevDegree);
        }


        else if(g_challenge_stepNo == 270 && distance-prevDis > 500){
            //黒線ブロックを超えるまで、一定距離を走行
            g_challenge_stepNo = 280;
        }
        //緑を見つけたら、減速
        else if(g_challenge_stepNo == 280 && cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60){ 
            //stateMachine->sendTrigger(EVT_block_challenge); //280
            g_challenge_stepNo = 281;
 
        //一度白を通過
        }else if(g_challenge_stepNo == 281 && curRgbSum > 300){
            g_challenge_stepNo = 282;

        //緑をみつけたらカーブ開始
        }else if(g_challenge_stepNo == 282 && cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60 ){
            stateMachine->sendTrigger(EVT_block_challenge); //282
            g_challenge_stepNo = 283;
        
        //一度白を通過
        }else if(g_challenge_stepNo == 283 && curRgbSum > 300){
            g_challenge_stepNo = 284;

        }else if(g_challenge_stepNo == 284 && cur_rgb.r + cur_rgb.g <= 150){  //青または黒をみつけたら、完全にターン)
            g_challenge_stepNo = 284;
            stateMachine->sendTrigger(EVT_block_challenge); //284
            g_challenge_stepNo = 285;
            clock->sleep(500);
        // ガレージの奥の距離を捉えたら直進
        }else if(g_challenge_stepNo == 285 && check_sonar(35,250)){
            prevDegree = getAzimuth();
            cntDegree = 0;
            g_challenge_stepNo = 286;
        }else if(g_challenge_stepNo == 286){
            int x = getAzimuth();
            cntDegree += getTurnDgree(prevDegree,x);
            prevDegree=x;
            //printf("cntDegree=%d,",cntDegree);

            if(cntDegree > 27){
                stateMachine->sendTrigger(EVT_block_challenge); //286
                //captain->decide(EVT_go_slowly);
                g_challenge_stepNo = 290;
            }

        }else if(g_challenge_stepNo == 290 && check_sonar(0,15)){
                stateMachine->sendTrigger(EVT_block_challenge); //290
                //captain->decide(EVT_stop);
                garage_flg = false;
        }


        // if (g_challenge_stepNo >=10 && g_challenge_stepNo <=290){
        //     printf("dis=%d,distance=%d,prev_dis=%d,deg=%d,prev_deg=%d,r=%03u, g=%03u, b=%03u,state=%d,\n",sonarSensor->getDistance(),distance,prevDis,getAzimuth(),prevDegree,g_rgb.r, g_rgb.g, g_rgb.b,g_challenge_stepNo);//sano t
        // }

    }//ガレージ終了

// sano ended







    /*
    int32_t iDeltaD  = (int32_t)(1000.0 * deltaDist );
    int32_t iDeltaDL = (int32_t)(1000.0 * deltaDistL);
    int32_t iDeltaDR = (int32_t)(1000.0 * deltaDistR);
    if (iDeltaDL != 0 && iDeltaDR != 0) {
        _debug(syslog(LOG_NOTICE, "%08u, %06d, %06d, %06d, %06d", clock->now(), getDistance(), iDeltaD, iDeltaDL, iDeltaDR));
    }
    */

    integD  += deltaDist;  // temp
    integDL += deltaDistL; // temp
    integDR += deltaDistR; // temp
    // display trace message in every PERIOD_TRACE_MSG ms
    if (++traceCnt * PERIOD_OBS_TSK >= PERIOD_TRACE_MSG) {
        traceCnt = 0;
        /*
        // temp from here
        int32_t iD  = (int32_t)integD;
        int32_t iDL = (int32_t)integDL;
        int32_t iDR = (int32_t)integDR;
        _debug(syslog(LOG_NOTICE, "%08u, %06d, %06d, %06d, %06d", clock->now(), getDistance(), iD, iDL, iDR));
        integD = integDL = integDR = 0.0;
        // temp to here
        */
        /*
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), getDistance(), getAzimuth(), getLocX(), getLocY()));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): angle = %d, anglerVelocity = %d", clock->now(), g_angle, g_anglerVelocity));
        */
        //_debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): sensor = %d, target = %d, distance = %d", clock->now(), g_grayScale, GS_TARGET, getDistance()));
    }
}

void Observer::deactivate() {
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler unset", clock->now()));
}

bool Observer::check_touch(void) {
    if (touchSensor->isPressed()) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_sonar(void) {
    int32_t distance = sonarSensor->getDistance();
    if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_sonar(int16_t sonar_alert_dist_from, int16_t sonar_alert_dist_to) {
    int32_t distance = sonarSensor->getDistance();
    //printf(",distance2=%d, sonar_alert_dist_from=%d, sonar_alert_dist_to=%d\n",distance, sonar_alert_dist_from, sonar_alert_dist_to );
    if (distance >= sonar_alert_dist_from && distance <= sonar_alert_dist_to) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_backButton(void) {
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_lost(void) {
    if (g_grayScale > GS_LOST) {
        return true;
    } else {
        return false;
    }
    /*
    int8_t otRes_r, otRes_g, otRes_b;
    otRes_r = ot_r->test(cur_rgb.r);
    otRes_g = ot_g->test(cur_rgb.g);
    otRes_b = ot_b->test(cur_rgb.b);
    if ((otRes_r == POS_OUTLIER && otRes_g == POS_OUTLIER) ||
        (otRes_g == POS_OUTLIER && otRes_b == POS_OUTLIER) ||
        (otRes_b == POS_OUTLIER && otRes_r == POS_OUTLIER)) {
        return true;
    } else {
        return false;
    }
    */
}

// bool Observer::check_tilt(void) {
//     int16_t anglerVelocity = gyroSensor->getAnglerVelocity();
//     if (anglerVelocity < ANG_V_TILT && anglerVelocity > (-1) * ANG_V_TILT) {
//         return false;
//     } else {
//         _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): TILT anglerVelocity = %d", clock->now(), anglerVelocity));
//         return true;
//     }
// }

void Observer::freeze() {
    frozen = true;
}

void Observer::unfreeze() {
    frozen = false;
}

//角度累積分を計算
int16_t Observer::getTurnDgree(int16_t prev_x,int16_t x){

    int16_t hoge = prev_x-x;
    if(hoge < 0){
        hoge = hoge * -1;
    }
    if(hoge > 180){
        hoge = 360-hoge;
    }

    return hoge;    
}

Observer::~Observer() {
    _debug(syslog(LOG_NOTICE, "%08u, Observer destructor", clock->now()));
}