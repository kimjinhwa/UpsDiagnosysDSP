#define ADC_SAMPLING_FREQ       8000.0L

#define RESULTS_BUFFER_SIZE     1024
#define EX_ADC_RESOLUTION       12

#define RFFT_STAGES     9
#define RFFT_SIZE       (1 << RFFT_STAGES)
#define F_PER_SAMPLE    (ADC_SAMPLING_FREQ/(float)RFFT_SIZE)
