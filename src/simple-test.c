  while(1)
  {
    unsigned long tmp = CNT;
    low(11);
    waitcnt(tmp + CLKFREQ/50)
    high(11);
    waitcnt(tmp + CLKFREQ/1000)
  }

  pwm_start(20000);
  pwm_set(11, 0, 1000);
  waitcnt(CNT + CLKFREQ);
  pwm_set(11, 0, 2000);