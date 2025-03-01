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
    hal.rcout->set_freq(1,200); //����PWM���ͨ��Ƶ�ʣ�Ĭ��ֵΪ50HZ
    hal.rcout->set_freq(0,200);  //ע��:����PWMͨ������Ĳ�ͬ����PWMͨ��������Ƶ�ʳ���50HZ
    hal.rcout->set_freq(3,40 );
    hal.rcout->force_safety_off(); //��ȫ���ذ�ťǿ��ʹ�ܣ����ڽ���״̬��
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

    //�޸ĺ�
    int16_t user_input ;
    hal.rcout->write(1, pwm); // ����PWM����
    hal.rcout->write(0, pwm);
    hal.rcout->write(3, pwm);
    // read in user input
    while (hal.console->AP_HAL::UARTDriver *AP_HAL::HAL::console){
		//AP HAL : :UARTDriver *AP HAL: :HAL: : console
		user_input = hal.console->read();   //��ȡ�������룬���ݴ�������ı�ռ�ձ�
		if(user_input == 'U'|| user_input == 'u') {
		   pwm += 100; //��������U������ռ�ձ�(�ߵ�ƽ�Ŀ��)��
        }
		if (user_input=='D'||user_input== 'd') {
		   pwm -= 100; //��������D��С�ߵ�ƽ������
		}
    }
}

AP_HAL_MAIN();
