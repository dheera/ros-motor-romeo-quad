// Arduino IDE code for Romeo BLE Quad to control 4 motors
// and output encoder data for all 4 motors
// via TTL serial

// PWM code based on code in Romeo BLE Quad Wiki with modifications

#include "stdlib.h"
#include "Arduino.h"

typedef struct {
        uint32_t RCC_APBPeriph_GPIO;    // GPIO clock
        uint32_t RCC_APBPeriph_TIM;     // timer clock
        uint32_t pin;                   // GPIO pin
        GPIO_TypeDef *GPIO;             // GPIO
        TIM_TypeDef *TIM;               // TIMER
} EncoderIO;

EncoderIO encoderIO[4]= {
  {RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM3,GPIO_Pin_6 | GPIO_Pin_7,GPIOB,TIM3},
  {RCC_APB2Periph_GPIOA,RCC_APB1Periph_TIM2,GPIO_Pin_0 | GPIO_Pin_1,GPIOA,TIM2},
  {RCC_APB2Periph_GPIOB,RCC_APB1Periph_TIM4,GPIO_Pin_6 | GPIO_Pin_7,GPIOB,TIM4},
  {RCC_APB2Periph_GPIOC,RCC_APB2Periph_TIM8,GPIO_Pin_6 | GPIO_Pin_7,GPIOC,TIM8}
};

unsigned char dut[4]; // PWM value
unsigned char counter; // 0-255 counter that increments by 1 every 1ms
unsigned char PWMFlag;

void TIM5_IRQHandler(void)
{              
  TIM5->SR = ~0x0001;
  counter++;
    
  if(PWMFlag & 0x01){
    if(!(PWMFlag & 0x10)){
      GPIO_ResetBits(GPIOC, GPIO_Pin_11);
    }else{
      if(counter<dut[0]){
        GPIO_SetBits(GPIOC, GPIO_Pin_11);
      }else if(counter!=255){
        GPIO_ResetBits(GPIOC, GPIO_Pin_11);
      }
    }
   }else{
      if(!(PWMFlag & 0x10)){
        GPIO_ResetBits(GPIOC, GPIO_Pin_12);
      }else{
        if(counter<dut[0]){
          GPIO_SetBits(GPIOC, GPIO_Pin_12);
        }else if(counter!=255){
          GPIO_ResetBits(GPIOC, GPIO_Pin_12);
        }
      }
    }        
        
  if(PWMFlag & 0x02){
    if(!(PWMFlag & 0x20)){
      GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    }else{
      if(counter<dut[1]){
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
      }else if(counter!=255){
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
      }
    }
  }else{
      if(!(PWMFlag & 0x20)){
        GPIO_ResetBits(GPIOC, GPIO_Pin_10);
      }else{
        if(counter<dut[1]){
          GPIO_SetBits(GPIOC, GPIO_Pin_10);
        }else if(counter!=255){
      GPIO_ResetBits(GPIOC, GPIO_Pin_10);
        }
      }
    }
        
    if(PWMFlag & 0x04){
      if(!(PWMFlag & 0x40)){
        GPIO_ResetBits(GPIOB,GPIO_Pin_9);
      }else{
        if(counter<dut[2]){
          GPIO_SetBits(GPIOB,GPIO_Pin_9);
        }else if(counter!=255){
          GPIO_ResetBits(GPIOB,GPIO_Pin_9);
        }
      }  
    }else{
      if(!(PWMFlag & 0x40)){
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
      }else{
        if(counter<dut[2]){
            GPIO_SetBits(GPIOB, GPIO_Pin_8);
        }else if(counter!=255){
            GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        }
      }
    }
        
    if(PWMFlag & 0x08)
    {
      if(!(PWMFlag & 0x80)){
        GPIO_ResetBits(GPIOB,GPIO_Pin_5);
      }else{
        if(counter<dut[3]){
          GPIO_SetBits(GPIOB,GPIO_Pin_5);
        }else if(counter!=255){
          GPIO_ResetBits(GPIOB,GPIO_Pin_5);
        }
      }
    }else{
      if(!(PWMFlag & 0x80)){
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);
      }else{
        if(counter<dut[3]){
          GPIO_SetBits(GPIOD,GPIO_Pin_2);
        }else if(counter!=255){
          GPIO_ResetBits(GPIOD,GPIO_Pin_2);
        }
      }
    }        
  
}

void TIM_Configuration(void)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;        
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
  NVIC_Init(&NVIC_InitStructure);               

  TIM_TimeBaseStructure.TIM_Period = 9;             
  TIM_TimeBaseStructure.TIM_Prescaler = 27;           
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;         
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0X0;    
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);   
   
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);         
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);  
  TIM_Cmd(TIM5, ENABLE);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}

void PWMStart(int channel)
{
  //PWMFlag |= motorIO[channel].pwmflag2;
  PWMFlag |= (0x10<<channel);
}

void PWMStop(int channel)
{
  //PWMFlag &= ~(motorIO[channel].pwmflag2);
  PWMFlag &= ~(0x10<<channel);
}

void PWMInit(unsigned char digitalPin) 
{
  pinMode(digitalPin,OUTPUT);
  
  if(digitalPin==8){
    PWMFlag |= 0x01;
  }else if(digitalPin==23){
    PWMFlag &= 0xfe;
  }else if(digitalPin==7){
    PWMFlag |= 0x02;
  }else if(digitalPin==9){
    PWMFlag &= 0xfd;
  }else if(digitalPin==24){
    PWMFlag |= 0x04;
  }else if(digitalPin==14){
    PWMFlag &= 0xfb;
  }else if(digitalPin==4){
    PWMFlag |= 0x08;
  }else if(digitalPin==25){
    PWMFlag &= 0xf7;
  }
}

const int motorDirPin[4][2] = {
  {8,23},
  {7,9},
  {24,14},
  {4,25}
};

void setup() {
   delay(1000);
   Serial3.begin(115200);


     TIM_Configuration();
     for(int j=0;j<4;j++){
       pinMode(motorDirPin[j][0], OUTPUT);
       digitalWrite(motorDirPin[j][0], LOW);
       PWMInit(motorDirPin[j][1]);
       PWMStart(j);
       encoderIO[j].TIM->CNT = 30000;

        GPIO_InitTypeDef GPIO_InitStructure;
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_ICInitTypeDef TIM_ICInitStructure;

                if(encoderIO[j].TIM == TIM4){
                GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);
        }

        if(encoderIO[j].TIM == TIM8){
                RCC_APB2PeriphClockCmd(encoderIO[j].RCC_APBPeriph_TIM,ENABLE);
        }else{
                RCC_APB1PeriphClockCmd(encoderIO[j].RCC_APBPeriph_TIM,ENABLE);
        }
        RCC_APB2PeriphClockCmd(encoderIO[j].RCC_APBPeriph_GPIO,ENABLE);
        GPIO_InitStructure.GPIO_Pin = encoderIO[j].pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(encoderIO[j].GPIO,&GPIO_InitStructure);

        TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
        TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;
        TIM_TimeBaseStructure.TIM_CounterMode = 0x00;
        TIM_TimeBaseInit(encoderIO[j].TIM, &TIM_TimeBaseStructure);

        TIM_EncoderInterfaceConfig(encoderIO[j].TIM,3,0,0);
        TIM_ICStructInit(&TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_ICFilter = 6;
        TIM_ICInit(encoderIO[j].TIM, &TIM_ICInitStructure);

       TIM_ClearFlag(encoderIO[j].TIM,1);
       TIM_Cmd(encoderIO[j].TIM,ENABLE);
    }

   Serial3.println("# start");
}

unsigned long currentTime;
unsigned long lastSetTime;

char buf[9];
unsigned int bufpos=255;

bool signs[4] = {false, false, false, false};

void set(uint8_t channel, int16_t value) {
  if(value>=0) {
       dut[channel] = 0xFF & (((uint16_t)value) >> 7);
       if(signs[channel]) dut[channel] = 0;
       //if(dut[channel]==1) dut[channel]==2;
       if(dut[channel] == 0) {
         PWMStop(channel);
         pinMode(motorDirPin[channel][0], OUTPUT);
         digitalWrite(motorDirPin[channel][0], LOW);
         pinMode(motorDirPin[channel][1], OUTPUT);
         digitalWrite(motorDirPin[channel][1], LOW);
       } else {
         pinMode(motorDirPin[channel][0], OUTPUT);
         digitalWrite(motorDirPin[channel][0], LOW);
         pinMode(motorDirPin[channel][1], OUTPUT);
         digitalWrite(motorDirPin[channel][1], LOW);
         PWMInit(motorDirPin[channel][1]);
         PWMStart(channel);
       }
       signs[channel] = false;
      } else {
       dut[channel] = 0xFF & ((~(uint16_t)value) >> 7);
       if(!signs[channel]) dut[channel] = 0;
       //if(dut[channel]==1) dut[channel]==2;
       if(dut[channel] == 0) {
         PWMStop(channel);
         pinMode(motorDirPin[channel][0], OUTPUT);
         digitalWrite(motorDirPin[channel][0], LOW);
         pinMode(motorDirPin[channel][1], OUTPUT);
         digitalWrite(motorDirPin[channel][1], LOW);
       } else {
         pinMode(motorDirPin[channel][0], OUTPUT);
         digitalWrite(motorDirPin[channel][0], LOW);
         pinMode(motorDirPin[channel][1], OUTPUT);
         digitalWrite(motorDirPin[channel][1], LOW);
         PWMInit(motorDirPin[channel][0]);
         PWMStart(channel);
       }
       signs[channel] = true;
      }
}

void processCommand() {
  uint8_t checksum = buf[8];
  if((buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]) % 255 != checksum) return;

  if(buf[0] == 'G') {
    uint8_t channel = buf[1];
    int16_t value = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    /*
     Serial3.print("# ");
    Serial3.print(channel);
    Serial3.print(" => ");
    Serial3.println(value);*/

    if(channel >=0 && channel <= 3) {
       set(channel, value);
       lastSetTime = millis();
    }

  }
}

char outbuf[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void loop() {

    
  char c;
  
  while(Serial3.available()) {
    
    c = Serial3.read();

    if(bufpos == 255) {
      // wait for start byte
      if(c == 0xF0) {
        bufpos = 0;
      }
    } else {
      // packet already started, read bytes
      buf[bufpos++] = c;
      if(bufpos>=9) {
        // packet fineshed, process packet
        processCommand();
        bufpos = 255;
      }
    }
  }

  outbuf[0] = 0xF0; // start byte
  outbuf[1] = 0x45; // encoder info

  for(int channel=0;channel<4;channel++) {
    short count = TIM_GetCounter(encoderIO[channel].TIM) - 30000;
    encoderIO[channel].TIM->CNT = 30000;
    outbuf[2+channel] = count;
  }
  outbuf[9] = (outbuf[1] + outbuf[2] + outbuf[3] + outbuf[4] + outbuf[5]) % 255;
  for(int i=0;i<10;i++) {
    Serial3.print(outbuf[i]);
  }
  delay(5);
  
  if(millis() - lastSetTime > 1000) {
    for(int j=0;j<4;j++) {
      set(j, 0);
    } 
  }
}
