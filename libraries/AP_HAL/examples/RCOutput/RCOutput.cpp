/*
  simple test of RC output interface
  Attention: If your board has safety switch,
  don't forget to push it to enable the PWM output.
 */

#include <AP_HAL/AP_HAL.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup (void)
{
    hal.console->printf("Starting AP_HAL::RCOutput test\n");
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif
    for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i);
    }
    hal.rcout->set_freq(1,200); //设置PWM输出通道频率，默认值为50HZ
    hal.rcout->set_freq(0,200);  //注意:根据PWM通道分组的不同部分PWM通道不允许频率超过50HZ
    hal.rcout->set_freq(3,40 );
    hal.rcout->force_safety_off(); //安全开关按钮强制使能，处于解锁状态。
}

static uint16_t pwm = 1500;
static int8_t delta = 1;

void loop (void)
{
    //for (uint8_t i=0; i < 14; i++) {
    //    hal.rcout->write(i, pwm);
    //    pwm += delta;
    //    if (delta > 0 && pwm >= 2000) {
    //        delta = -1;
    //        hal.console->printf("decreasing\n");
    //    } else if (delta < 0 && pwm <= 1000) {
    //        delta = 1;
    //        hal.console->printf("increasing\n");
    //    }
    //}
    //hal.scheduler->delay(5);

    //修改后
    int16_t user_input ;
    hal.rcout->write(1, pwm); // 设置PWM脉宽
    hal.rcout->write(0, pwm);
    hal.rcout->write(3, pwm);
    // read in user input
    while (hal.console->AP_HAL::UARTDriver *AP_HAL::HAL::console){
		//AP HAL : :UARTDriver *AP HAL: :HAL: : console
		user_input = hal.console->read();   //获取串口输入，根据串口输入改变占空比
		if(user_input == 'U'|| user_input == 'u') {
		   pwm += 100; //键盘输入U，增加占空比(高电平的宽度)。
        }
		if (user_input=='D'||user_input== 'd') {
		   pwm -= 100; //键盘输入D减小高电平的脉宽。
		}
    }
}

AP_HAL_MAIN();
