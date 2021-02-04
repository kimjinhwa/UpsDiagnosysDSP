## DiagnosysUps Software Document
### For Debug 
	- make DAC for test
	  This is for input test ADC, So It use inputA_0 pin. 
	 - Later, It must be changed.
	 - #define DAC_TEST_USED   = 1
### Note all 
    - It is tested using EVK KIT, 10Mhz , So it must be changed to 20Mhz 
    - Total read adc is 20.
    - Each fft_routine() function is executed sequentially. 
    - Can start special point fft. 
    - For run sample program, use RFFTin1Buff_test, Later It will be cleard. and Can use for another purpose.
### note_1 fft_routine();
	- This Function take 105519 sysclk needed.( measured)
    - At 200Mhz Cpu clock 1 sysclk is 5ns 
    - So 105519*10nS = 527,595 ns

### note_2 __interrupt void adcA1ISR(void)  
	- Adc sampling is 8Khz (125us)
	- buffer size 800 (125us*800=100,000us)
	- total adc 20
	- From ADC Conversion to 20 Adc reading sysclk is 11,697 ( 11,697 * 50ns = 584,850ns   0.58us) 

### How to request fft data
	- 1. if you want to execute n-th data to fft, then set request_fft set value 0~20.
	- 2. then set index=0 to start from first 
	- 3. wait until bufferFull ==1
	- 4. execute fft function.


_italic_  : _this_ word would become italic. 
**bold**  : This will **really** get your point across.


![Benjamin Bannekat](https://upload.wikimedia.org/wikipedia/commons/5/56/Tiger.50.jpg)

### How to calculate fft
	- stage = 9  ; 2^9 = 512 So needed 512*2=1024 buffer
    - Calculate twiddle factor
      RFFT_f32_sincostable(hnd_rfft);        
	- Calculate real FFT with 12-bit
      RFFT_adc_f32(hnd_rfft_adc);
    - Calculate magnitude 
      RFFT_f32_mag(hnd_rfft);             
    - In the RFFTmagBuff, get Max data position And Caluate frequency
      frequency =(float)F_PER_SAMPLE * (float)position;
      where position is maxvalue position, F_PER_SAMPLE=(ADC_SAMPLING_FREQ/(float)RFFT_SIZE) 
      F_PER_SAMPLE = 8Khz / 512 = 15
      if we get the position 82, 82*5 = 1,230Hz ( In this example sample data)
### To get THD 
	- 1. get Max Magnitude
		(2*Maxvalue)/N   , where N is 512
	- 2. Y=power( (2*abs(X)/N),2)  
	- 3. THD = sqrt( sum(Y) /(2*Maxvalue)/N  )

