# blbl-飘扬布鲁斯
用的这个程序，可以飞，但是转向不是很灵敏，改成转向的时候让一边翅膀少拍，另一边多扑腾就可以了。有高手可以改改这个程序，实在看不懂。https://surin.reru.ac.th/wp-content/uploads/2020/09/flap_control.zip
https://www.youtube.com/watch?v=L3r4dEI-Qwk&t=1s
Test Arduino NANO Servo Flap Control Ornithopter wings 
这是我第二次尝试在伺服动力的鸟。我用的是在Ebay网站https://www.ebay.com/itm/2PCS-Pro-Mi...72.m2749.l2649上买的Arduino中文克隆版，
KST MS320伺服器，微型接收器和300MA LIPO。
重量为130G，机翼40“(1029mm)长X 10”(254mm)宽，固定尾翼8.75“(222mm)长X 11 1/4 (286mm)宽，中间有一个垂直鳍，以帮助转弯。
最大上翼角为80度，下翼角为65度。在节流阀关闭的情况下，机翼的滑翔角大约是向上25度。为了让飞机转起来，代码改变了襟翼的中心角度，这样在全舵状态下，一只翅膀向上约85度，另一只翅膀向下约70度。
在它的5次飞行中，它的最大高度是10英尺(3.5米)。我已经改变了CG移动电池到前面。我已经改变了尾角从目前的20度上升到5度。这些东西都没有帮助我们达到10英尺以上的高度。”
code 
https://surin.reru.ac.th/topics/rc-ro...
https://www.rcgroups.com/forums/showp...

https://www.rcgroups.com/forums/showt...
Here is the currently used Arduino code modified fro the KST servos.
The short video is a low throttle.

![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762864-244-IMG_20180111_193313579.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762865-133-IMG_20180111_193342606.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762866-64-IMG_20180111_193424128.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762867-63-IMG_20180111_193432578.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762868-239-IMG_20180111_193440355.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762869-232-IMG_20180226_122754300.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762870-53-IMG_20180226_122838704.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762871-42-IMG_20180226_122907170.jpg)
![图片描述](https://static.rcgroups.net/forums/attachments/1/1/5/3/3/0/a10762872-5-IMG_20180226_122927941.jpg)


//Arduino servo flap system
//copyright Steve Morris 10-25-16
#include <Servo.h>
volatile int pwm1_value = 0;
volatile int pwm2_value = 0;
volatile int prev_time0 = 0;
volatile int prev_time4 = 0;
static int servo_comm1 = 0;
static int servo_comm2 = 0;
volatile int flapangle1 = 0;
volatile int flapangle2 = 0;
volatile int rudder = 0;
volatile int millisold = 0;
int millinow= 0;
float dt=0;
float flapmag = 0;
static float flapdeg = 0;
float tcommand = 0;
float floattime=0;
float temp=0;
float glide_deg = +5.0;
static int pwm1_value_temp = 0;
static int pwm2_value_temp = 0;
float omegadot=0.0;
float thetadot=0.0;
static float omega=0.0;
static float theta=0.0;
static float k0=1.0;
static float k2=10.0;
static float servo_zero1=-4;
static float servo_zero2=0;

Servo myservo1, myservo2; // create servo object to control a servo

void setup() {
//Serial.begin(115200);
// when pin D2 goes high, call the rising function
attachInterrupt(0, rising0, RISING);
attachInterrupt(4, rising4, RISING);
myservo1.attach(5); // attaches the servo on pin 5 to the servo object
myservo2.attach(6); // attaches the servo on pin 6 to the servo object
}
void loop() {

millinow=millis();
floattime=millinow/1000.0;
dt=(millinow-millisold)/1000.0;
millisold=millinow;

tcommand=(pwm1_value-480.0)/8.22;
omegadot=k0*tcommand-k2*omega;
thetadot=omega;
flapdeg=sin(theta);
theta=theta+omega*dt;
omega=omega+omegadot*dt;

flapmag=(pwm1_value-880)/41.0+6;
flapdeg=flapmag*sin(theta);//variable amplitude+freq

rudder =(int)(30.0+(pwm2_value-1500)/30);

flapangle1=(int)((rudder-flapdeg+servo_zero1)*2.0+15.0);
flapangle2=(int)((rudder+flapdeg+servo_zero2)*2.0+ 15.0);

if (pwm1_value > 1040){
servo_comm1 = flapangle1;
servo_comm2 = flapangle2;
}

//Glide Lock
if (pwm1_value < 1010){

servo_comm1 = (int)((rudder-glide_deg+servo_zero1)*2.0+25.0);
servo_comm2 = (int)((rudder+glide_deg+servo_zero2)*2.0+25.0);

}

myservo1.write(servo_comm1); // tell servo to go to position in variable 'pos'
myservo2.write(servo_comm2); // tell servo to go to position in variable 'pos'

//Serial.println(servo_comm1);
//Serial.println(floattime);
//Serial.println(dt);
//Serial.println(flapangle);
//Serial.println(flapdeg);
//Serial.println(temp);
//Serial.println(tcommand);
//Serial.println(thr pwm1_value);
//Serial.println(thr pwm2_value);

}

void rising0() {
attachInterrupt(0, falling0, FALLING);
prev_time0 = micros();
}

void falling0() {
attachInterrupt(0, rising0, RISING);
pwm1_value = micros()-prev_time0;
}
void rising4() {
attachInterrupt(4, falling4, FALLING);
prev_time4 = micros();
}
void falling4() {
attachInterrupt(4, rising4, RISING);
pwm2_value = micros()-prev_time4;
}
