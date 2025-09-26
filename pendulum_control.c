
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
// USB communication
#include <termios.h>
// TCP/IP communication
#include <arpa/inet.h>

#include <malloc.h>

#include <pthread.h>
#include <sched.h> // To set POSIX thread affinity

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/sem.h>

/* RTDM header */
#include <rtdm/rtdm.h>

#include <pigpio.h>

#include "WavePWM.h"
#include "RT_SPI2ADDA.h"
#include "SPI_AS5048.h"
#include "RT_Spi2RelEnc.h"
#include "time_check.h"
#include "print_bit.h"
#include "conversion.h"
#include "constants.h"


#define NSEC_PER_SEC 1000000000
#define USEC_PER_SEC 1000000
#define MSEC_PER_SEC 1000

#define N_BACKUP	500000

RT_TASK adda_task, print_task;
pthread_t NR_task_sender, NR_task_receiver;

/*** TCP/IP_COMMUNICATION_tasks' global variables ***/

#define PORT_SENDER   12006            // First Server port
#define PORT_RECEIVER (PORT_SENDER+1)  // Second Server port
#define BUFFER_SIZE 1024               // Message buffer width
#define NUM_FLOAT_VAL_TO_SEND    9
#define NUM_FLOAT_VAL_TO_RECEIVE 4

double shared_values_to_send[NUM_FLOAT_VAL_TO_SEND-1];
int new_values_received = 0;
double shared_value_received[NUM_FLOAT_VAL_TO_RECEIVE];

/********************************************/

#define PRINT_ON 0

// Setting od Std CSs as extended GPIO pins
int spi_gpio_handler;

/*** PWM global variables ***/

long k = 0;

double maximum_voltage = MAXIMUM_VOLTAGE;
double volt_to_set = 0.0, null_volt = 0.0;

/****************************/

unsigned int cycle_ns;
double period;

int run = 1;

//Timer
long multi_rw_time = 0, worst_multi_rw_time = 0;

/*** ADDA_task's global variables ***/

uint16_t max_adc1 = 0, min_adc1 = 3000;
uint16_t max_adc2 = 0, min_adc2 = 3000;
uint16_t max_adc3 = 0, min_adc3 = 3000;
uint16_t max_adc4 = 0, min_adc4 = 3000;
uint16_t max_adc5 = 0, min_adc5 = 3000;
uint16_t max_adc6 = 0, min_adc6 = 3000;
uint16_t max_adc7 = 0, min_adc7 = 3000;
uint16_t max_adc8 = 0, min_adc8 = 3000;

double media_adc1 = 0, media_adc2 = 0,  media_adc3 = 0,  media_adc4 = 0;
double media_adc5 = 0, media_adc6 = 0,  media_adc7 = 0,  media_adc8 = 0;
int count_media1 = 0, count_media2 = 0, count_media3 = 0, count_media4 = 0;
int count_media5 = 0, count_media6 = 0, count_media7 = 0, count_media8 = 0;

uint16_t valori_adcs[8], valori_adcs_prev[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/* structure for saving data from the actual (col 0) and the previous (col 1) sample time
amp_PHASE_2_act		amp_PHASE_2_old
amp_PHASE_1_act		amp_PHASE_1_old*/
int first_amp_motori = 1;
double amp_motori[8][2];
double amp_filtrata[8][2];

int count_zero_1 = 0, count_zero_2 = 0, count_zero_3 = 0, count_zero_4 = 0;
int count_zero_5 = 0, count_zero_6 = 0, count_zero_7 = 0, count_zero_8 = 0;

double salvataggio_amp[N_BACKUP][15];
int step_no_amp = 0;

ad5592r_dev dev_ad5592r;

int error_occurred = 0;
// Pendulum Absolute Encoder
double pendulum_absenc_pos_init = (3.60865324+0.0105); //DO NOT CHANGE IT
double pendulum_absenc_pos_2, pendulum_absenc_pos_3, pendulum_absenc_pos_prev;
double pendulum_th = 25.5/180.0*M_PI; //DO NOT CHANGE IT
AS5048_dev dev_as5047P;
double pendulum_absenc_pos; //PENDULUM ENCODER POSITION
uint16_t pendulum_raw_absenc_pos;

// Motor Absolute Encoder
double motor_absenc_pos_init = 3.515864; //DO NOT CHANGE IT
double motor_absenc_pos_2, motor_absenc_pos_3, motor_absenc_pos_prev;
double motor_th = 80.0/180.0*M_PI; //DO NOT CHANGE IT
AS5048_dev dev_as5048;
double motor_absenc_pos; //ENCODER POSITION
uint16_t motor_raw_absenc_pos;

double motor_relenc_pos; //MOTOR POSITION
long rel_enc_counter;
double rel_angle_prev_rotation = 0.0;
int rel_full_rotation = 0;
double motor_relenc_pos_2;
double r_angle;

int main_thread_start = 0;

long time_elapsed, worst_case_time_elapsed = -1;

/**************************************/

/******** Semaphore Management ********/

#define SEM_INIT 1       /* Initial semaphore count */
#define SEM_MODE S_FIFO  /* Wait by FIFO order (use S_PRIO for priority ordering)*/

RT_SEM sem_SPI, sem_TCPIP;

/**************************************/

/*
* Function to set where threads must execute them code
*/
void set_affinity_cpu(RT_TASK *task, int cpu) {
    cpu_set_t cpus;
    CPU_ZERO(&cpus);          
    CPU_SET(cpu,&cpus);          
    int ret=rt_task_set_affinity(task,&cpus);  
	rt_printf("\nRet after set affinity %d",ret);
}

void set_affinity_cpu_NR(pthread_t *task, int cpu) {
    cpu_set_t cpus;
    CPU_ZERO(&cpus);          
    CPU_SET(cpu, &cpus);  // It must be core 3       
    int ret=pthread_setaffinity_np(*task,sizeof(cpu_set_t),&cpus);  
	rt_printf("\nRet after set affinity %d",ret);
}

/* NOTE: error handling omitted. */

void catch_signal(int sig)
{
	rt_printf("\nEnding...\n");
	run = 0;
	usleep(10e5);

	// PWM close function
	stopWavePWM();

	rt_task_delete(&adda_task);

	if (PRINT_ON)
		rt_task_delete(&print_task);

	//SPI_PIGPIO_close_device(&spi_gpio_handler);

	RT_addaClose(&dev_ad5592r);
	rt_printf("Adda close!\n");

	rt_printf("AS5047P abs encoder close!\n");
	gpioWrite(ENC_ABS_SWITCH, 0);
	as5048_close(&dev_as5047P);
	rt_printf("AS5048 abs encoder close!\n");
	gpioWrite(ENC_ABS_SWITCH, 1);
	as5048_close(&dev_as5048);

	RT_relencClose();
	rt_printf("RELATIVE encoder close!\n");

	// Ask to cancel the Non Real-Time thread
	pthread_cancel(NR_task_sender);
	pthread_join(NR_task_sender, NULL);

	// Ask to cancel the Non Real-Time thread
	pthread_cancel(NR_task_receiver);
	pthread_join(NR_task_receiver, NULL);

	rt_sem_delete(&sem_SPI);
	rt_sem_delete(&sem_TCPIP);
	
	gpioTerminate();

	print_times();
	
	rt_printf("\nSaving file...\n");

	/************************************************/
	
	int max_step = (step_no_amp < N_BACKUP) ? step_no_amp : N_BACKUP;
	FILE *ampBackup;
    ampBackup = fopen("./output_savings/amp_backup.txt", "w");
    
    for(int i=0; i<max_step; i++)
    	fprintf(ampBackup, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
								salvataggio_amp[i][0],  salvataggio_amp[i][1],  salvataggio_amp[i][2],
								salvataggio_amp[i][3],  salvataggio_amp[i][4],  salvataggio_amp[i][5],
								salvataggio_amp[i][6],  salvataggio_amp[i][7],  salvataggio_amp[i][8],
								salvataggio_amp[i][9],  salvataggio_amp[i][10], salvataggio_amp[i][11],
								salvataggio_amp[i][12], salvataggio_amp[i][13], salvataggio_amp[i][14] );
    	
    fclose(ampBackup);
	rt_printf("\nBackup AMPERE completato!\n");

	/************************************************/
	
	rt_printf("\nEnd!\n");
}

void *NR_TCPIP_comm_sender(void *arg)
{
	rt_printf("\n(NR_task_sender): Non_Real-Time sender task Start!\n");
	pthread_setname_np(pthread_self(), "NR_task_sender");

	/************************/
	/**** INITIALIZATION ****/
	/************************/

	int server_fd, client_socket, opt = 1;
    struct sockaddr_in server_address, client_address;
    socklen_t client_addr_len = sizeof(client_address);
    char buffer[BUFFER_SIZE] = {0};

    // Initialization of a socket TCP/IP
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        perror("(NR_task_sender): Initialization Socket Error");
        exit(EXIT_FAILURE);
    }

	// Set SO_REUSEADDR option
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("(NR_task_sender): Setsockopt Error");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    // Server address configuration
    server_address.sin_family = AF_INET;             // IPv4
    server_address.sin_addr.s_addr = INADDR_ANY;     // Listen on all interfaces
    server_address.sin_port = htons(PORT_SENDER);    // Server port

    // Socket Binding to the specified address and port
    if (bind(server_fd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        perror("(NR_task_sender): Binding Error");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    // Server listening mode for at most 3 connections
    if (listen(server_fd, 3) < 0) {
        perror("(NR_task_sender): Listen Error");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    printf("(NR_task_sender): Server awaits connections on port %d\n", PORT_SENDER);

    // Accept an incoming connection
    client_socket = accept(server_fd, (struct sockaddr *)&client_address, &client_addr_len);
    if (client_socket < 0) {
        perror("Errore in accept");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    printf("(NR_task_sender): Client connection established.\n");

	usleep(2e6);
	
	/************************/

	double t_samp = 1.0 / MSEC_PER_SEC; //0.0004 milliseconds, almost 2 KHz
	int t_samp_nano = (int) (t_samp * NSEC_PER_SEC); 
	int t_samp_micro = (int) (t_samp * USEC_PER_SEC);

	double t_init = (double) rt_timer_read();

	double value_to_send[NUM_FLOAT_VAL_TO_SEND-1];

	while (run) /* start the control loop */
	{
		// In order to take trace of the real elapsed time
		double t_act = (double) rt_timer_read();
    	double time_elapsed =  (t_act - t_init) / (double) NSEC_PER_SEC;
		
		// Wait for the TCP/IP semaphore unit... 
		rt_sem_p(&sem_TCPIP, TM_INFINITE);

		// Shared variables must be handled here
		for (int i = 0; i < (NUM_FLOAT_VAL_TO_SEND-1); i++)
			value_to_send[i] = shared_values_to_send[i];
		
		// Release the TCP/IP semaphore unit 
		rt_sem_v(&sem_TCPIP);

		char uint8_value_to_send_vec[NUM_FLOAT_VAL_TO_SEND*4];
		char *tmp_ptr;
		float tmp_float_val;
		int global_counter = 0;

		tmp_float_val = (float) time_elapsed;
		tmp_ptr = (char *) &tmp_float_val;
		for (int j = 0; j < 4; j++) {
			uint8_value_to_send_vec[global_counter] = (char) tmp_ptr[j];
			global_counter += 1;
		}

		for (int i = 0; i < NUM_FLOAT_VAL_TO_SEND-1; i++) {

			tmp_float_val = (float) value_to_send[i];
			tmp_ptr = (char *) &tmp_float_val;
			for (int j = 0; j < 4; j++) {
				uint8_value_to_send_vec[global_counter] = (char) tmp_ptr[j];
				global_counter += 1;
			}

		}

		// Send back information to the Client
		send(client_socket, uint8_value_to_send_vec, sizeof(uint8_value_to_send_vec), 0);

		/*printf("Sent message to the Client: %0.5f %0.5f %0.5f %0.5f %0.5f %0.5f %0.5f %0.5f %0.5f\n\n", time_elapsed,
		                            value_to_send[0], value_to_send[1], value_to_send[2], value_to_send[3], 
									value_to_send[4], value_to_send[5], value_to_send[6], value_to_send[7] );*/
		

		// Sleep that emulates a periodic task behaviour
		usleep(t_samp_micro);

		pthread_testcancel();

	}

	// Closing SOCKET communication
    close(client_socket);
    close(server_fd);

	rt_printf("\n(NR_task_sender): Non Real-Time task End!\n");
}

void *NR_TCPIP_comm_receiver(void *arg)
{
	rt_printf("\n(NR_task_receiver): Non_Real-Time receiver task Start!\n");
	pthread_setname_np(pthread_self(), "NR_task_receiver");

	/************************/
	/**** INITIALIZATION ****/
	/************************/

	int server_fd, client_socket, opt = 1;
    struct sockaddr_in server_address, client_address;
    socklen_t client_addr_len = sizeof(client_address);

    // Initialization of a socket TCP/IP
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        perror("(NR_task_receiver): Initialization Socket Error");
        exit(EXIT_FAILURE);
    }

	// Set SO_REUSEADDR option
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("(NR_task_receiver): Setsockopt Error");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    // Server address configuration
    server_address.sin_family = AF_INET;             // IPv4
    server_address.sin_addr.s_addr = INADDR_ANY;     // Listen on all interfaces
    server_address.sin_port = htons(PORT_RECEIVER);  // Server port

    // Socket Binding to the specified address and port
    if (bind(server_fd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        perror("(NR_task_receiver): Binding Error");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    // Server listening mode for at most 3 connections
    if (listen(server_fd, 3) < 0) {
        perror("(NR_task_receiver): Listen Error");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    printf("(NR_task_receiver): Server awaits connections ...\n");

    // Accept an incoming connection
    client_socket = accept(server_fd, (struct sockaddr *)&client_address, &client_addr_len);
    if (client_socket < 0) {
        perror("Errore in accept");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    printf("(NR_task_receiver): Client connection established.\n");

	usleep(2e6);
	
	/************************/

	while (run) /* start the control loop */
	{
		uint8_t command_received;
		float *float_data_received_pointer;
		char *uint8_data_received;

		float float_data_received[NUM_FLOAT_VAL_TO_RECEIVE];
		uint8_data_received = (char *)float_data_received;

		// Receive Client data
		int bytes_read = recv(client_socket, uint8_data_received, NUM_FLOAT_VAL_TO_RECEIVE * 4, 0);

		if (bytes_read != NUM_FLOAT_VAL_TO_RECEIVE * 4)
		{
			fprintf(stderr, "Receiving error: %d received bytes\n", bytes_read);
			pthread_join(NR_task_receiver, NULL);
			break;
		}
		/*
		printf("\nReceived RAW DATA: ");
		for (int i = 0; i < NUM_FLOAT_VAL_TO_RECEIVE * 4; i++)
			printf("%d ", uint8_data_received[i]);
		*/
		printf("\nReceived float DATA: ");
		for (int i = 0; i < NUM_FLOAT_VAL_TO_RECEIVE; i++)
			printf("%.6f ", float_data_received[i]);
		printf("\n");

		// Wait for the TCP/IP semaphore unit...
		rt_sem_p(&sem_TCPIP, TM_INFINITE);

		new_values_received = 1;
		// Shared variables must be handled here
		for (int i = 0; i < NUM_FLOAT_VAL_TO_RECEIVE; i++)
			shared_value_received[i] = (double) float_data_received[i];

		// Release the TCP/IP semaphore unit
		rt_sem_v(&sem_TCPIP);

		pthread_testcancel();

	}

	// Closing SOCKET communication
    close(client_socket);
    close(server_fd);

	rt_printf("\n(NR_task_receiver): Non Real-Time task End!\n");
}


void RT_adda(void *arg)
{
	
	double theta_1;
	double theta_1_prev;
	double theta_2;
	double theta_2_prev;
	double theta_1d;
	double theta_1d_prev;
	double theta_2d;
	double theta_2d_prev;
	double K1 = -0.3043;
	double K2 = 9.2119;
	double K3 = -1.4322;
	double K4 = 1.7199;
	double v_in;
    double a = 300.0;
    double Ts = 0.001;
	double abs_enc_offset = 0;
    double u;
    double set_point = M_PI/4.0;
	double error;

	RTIME now, previous;

	/************************************* INIT Current sensors *************************************/

	uint8_t adda_bit_conv[16];
	
	uint8_t channel_modes[8], channel_offstate[8];
	
	channel_modes[0] = CH_MODE_ADC; /* Channel 0 is ADC */
	channel_modes[1] = CH_MODE_ADC; /* Channel 1 is ADC */
	channel_modes[2] = CH_MODE_UNUSED; /* Channel 2 is UNUSED */
	channel_modes[3] = CH_MODE_UNUSED; /* Channel 3 is UNUSED */
	channel_modes[4] = CH_MODE_UNUSED; /* Channel 4 is UNUSED */
	channel_modes[5] = CH_MODE_UNUSED; /* Channel 5 is UNUSED */
	channel_modes[6] = CH_MODE_UNUSED; /* Channel 6 is UNUSED */
	channel_modes[7] = CH_MODE_UNUSED; /* Channel 7 is UNUSED */

	int ret = RT_addaInit(&dev_ad5592r, channel_modes, channel_offstate, USE_SPI_AUX_ADDA, BCM283X_SPI_SPEED_15MHz, USE_INTERNAL_VREF);
	
	//lettura GPIO
	uint8_t gpio_read_value;
	
	//Multiread ADC
	uint8_t adc_chans_to_read[8] = {1, 1, 0, 0, 0, 0, 0, 0}; 
	uint8_t num_adc_to_read = 2;
	
	//Multiwrite DAC
	uint8_t dac_chans_to_write[1] = {0};
	uint8_t num_dac_to_write = 0;
	uint16_t valori_dacs[1] = {0};

	// Preliminary reading
	RT_addaMulti_repeat_read_adc_CMD(&dev_ad5592r, adc_chans_to_read);
	RT_addaMulti_repeat_read_adc_RTRN_adaptable(&dev_ad5592r, adc_chans_to_read, valori_adcs, valori_adcs_prev);
	
	rt_printf("ADC 1 (FASE 1): %d \t ", valori_adcs[0]);
	print_bits_uint16(valori_adcs[0]);
	
	rt_printf("ADC 2 (FASE 2): %d \t ", valori_adcs[1]);
	print_bits_uint16(valori_adcs[1]);

	/************************************************************************************************/

	/************************************* INIT Pendulum Encoder *************************************/

	// INIT ABSOLUTE PENDULUM ENCODER
	gpioWrite(ENC_ABS_SWITCH, 0);
	uint8_t bit_conversion[16];
	ret = as5048_init(&dev_as5047P, USE_SPI_STD, 1, 300.0, 0.0001, USE_AS5047P_ENC); //POLE OF THE SYSTEM IS HERE
	// Request of the angle for the next frame
	as5048_getRotation_CMD(&dev_as5047P);
	usleep(100);
 	as5048_getRotationInRadians_RTRN(&dev_as5047P, &pendulum_absenc_pos, &pendulum_raw_absenc_pos, bit_conversion); //Get the rotation of the absolute encoder of the motor
	as5048_getRotation_CMD(&dev_as5047P);

	as5048_getPreciseAngleInRadians(&dev_as5047P, &pendulum_absenc_pos_2);

	/*************************************************************************************************/

	/************************************* INIT Absolute Encoder *************************************/

	// INIT ABSOLUTE MOTOR ENCODER
	gpioWrite(ENC_ABS_SWITCH, 1);
	ret = as5048_init(&dev_as5048, USE_SPI_STD, 1, 300.0, 0.0001, USE_AS5048_ENC); //POLE OF THE SYSTEM IS HERE
	// Request of the angle for the next frame
	as5048_getRotation_CMD(&dev_as5048);
	usleep(100);
	//Get the rotation of the absolute encoder of the motor
 	as5048_getRotationInRadians_RTRN(&dev_as5048, &motor_absenc_pos, &motor_raw_absenc_pos, bit_conversion); 
	as5048_getRotation_CMD(&dev_as5048);

	as5048_getPreciseAngleInRadians(&dev_as5048, &motor_absenc_pos_2);
	motor_absenc_pos_init = motor_absenc_pos;

	/*************************************************************************************************/

	// Motor Absolute Encoder (Theta 1)
	theta_1_prev = motor_absenc_pos_2 - motor_absenc_pos_init; 
	// Pendulum Absolute Encoder (Theta 2)
	theta_2_prev = pendulum_absenc_pos_2 - pendulum_absenc_pos_init;

	/************************************* INIT Relative Encoder *************************************/

	usleep(1000);
	// INIT RELATIVE MOTOR ENCODER
	ret = RT_relencInit_TWO_AUX(1);
	long rotAbsEnc_to_relEnc;
	rad2relEnc(&rotAbsEnc_to_relEnc, motor_absenc_pos);
	
	rt_printf("Raw value to load to the relative encoder: %ld\n", rotAbsEnc_to_relEnc);
	RT_relencWrite_data_register(SPI_KNEE, 0);
	RT_relencLoad_counter(SPI_KNEE);

	/*************************************************************************************************/

	for (int i = 0; i < (NUM_FLOAT_VAL_TO_SEND-1); i++)
		shared_values_to_send[i] = 0;
	
	usleep(1*10e3);
	
	/*
	* Arguments: &task (NULL=self),
	*            start time,
	*            period (here: 0.1 ms)
	*/
	double t_samp = 1.0 / MSEC_PER_SEC; //0.0001 seconds
	//double t_samp = 0.1 / MSEC_PER_SEC; //0.0001 seconds
	int t_samp_nano = (int) (t_samp * NSEC_PER_SEC); //1 000 000 nanosec
	
	rt_printf("T ms: %f\t T ns: %d\n", t_samp, t_samp_nano);
	rt_task_set_periodic(NULL, TM_NOW, t_samp_nano);
	
	usleep(5000);

	// PWM initialization
	initWavePWM();
	VtoWavePWM(0.0, 0.0, maximum_voltage, maximum_voltage);

	k = 1;
	
	gpioWrite(PWM_ENABLE, 0);

	//for timing library
	int id = create_timing_stats();
	
	while (run) /* start the control loop */
	{
		//rt_task_wait_period(NULL);
		new_wait_period(id);
		
		// ADDA PART
		previous = rt_timer_read();
		
		main_thread_start = 1;

		/************************************* Current sensors *************************************/

		RT_addaMulti_repeat_read_adc_RTRN_adaptable(&dev_ad5592r, adc_chans_to_read, valori_adcs, valori_adcs_prev);

		//convert ADC values to amp values 
		
		adc2amp(&amp_motori[IDX_PHASE_A_1][0], valori_adcs[IDX_PHASE_A_1], 1, ADC_ZERO_AMP_1, ADC_MAX_AMP);
		adc2amp(&amp_motori[IDX_PHASE_B_1][0], valori_adcs[IDX_PHASE_B_1], 1, ADC_ZERO_AMP_2, ADC_MAX_AMP);
		
		/*
		adc2amp_compensate(&amp_motori[IDX_PHASE_A_1][0], valori_adcs[IDX_PHASE_A_1], 1, ADC_ZERO_AMP_1, IDX_PHASE_A_1);
		adc2amp_compensate(&amp_motori[IDX_PHASE_B_1][0], valori_adcs[IDX_PHASE_B_1], 1, ADC_ZERO_AMP_2, IDX_PHASE_B_1);
		*/
		/*******************************************************************************************/

		/************************************* Absolute Encoder *************************************/

		// PENDULUM ABSOLUTE ENCODER PART
		gpioWrite(ENC_ABS_SWITCH, 0);
		int ret_pendulum_enc; 
		ret_pendulum_enc = as5048_getRotationInRadians_RTRN(&dev_as5047P, &pendulum_absenc_pos, &pendulum_raw_absenc_pos, bit_conversion);
		// Richiedo l'angolo per il frame successivo
		as5048_getRotation_CMD(&dev_as5047P);

		as5048_getPreciseAngleInRadians(&dev_as5047P, &pendulum_absenc_pos_2); // GET THE CONTINUOUS ANGLE POSITION OF THE MOTOR

		/*********************************************************************************************/
		
		/************************************* Absolute Encoder *************************************/

		// MOTOR ABSOLUTE ENCODER PART
		gpioWrite(ENC_ABS_SWITCH, 1);
		int ret_motor_enc; 
		ret_motor_enc = as5048_getRotationInRadians_RTRN(&dev_as5048, &motor_absenc_pos, &motor_raw_absenc_pos, bit_conversion);
        // Richiedo l'angolo per il frame successivo
		as5048_getRotation_CMD(&dev_as5048);

		as5048_getPreciseAngleInRadians(&dev_as5048, &motor_absenc_pos_2); // GET THE ANGLE POSITION OF THE MOTOR / 2PI

		/*********************************************************************************************/

		/************************************* Relative Encoder *************************************/

		// MOTOR RELATIVE ENCODER PART (SPI_HIP or SPI_KNEE)
		rel_enc_counter = RT_relencRead_counter(SPI_KNEE) % (MAX_INT_REL_ENC+1);
		if (rel_enc_counter < 0)
			rel_enc_counter += (MAX_INT_REL_ENC+1);
		relEnc2rad(&motor_relenc_pos, rel_enc_counter);

		// Use Relative just for velocity, while the absolute one for position 
		r_angle = motor_relenc_pos - rel_angle_prev_rotation; // Create a relative angle to the starting point of the motor

		if (fabs(r_angle) > (0.8*2.0*M_PI)){ // Useful to take note of number of motor turns
			rel_full_rotation += ( r_angle > 0 ) ? -1 : 1; 	
		}

		rel_angle_prev_rotation = motor_relenc_pos; 		// Saving of the motor position
		
		// Get the angle in radian depending of the number of turns
		motor_relenc_pos_2 = motor_relenc_pos+2.0*M_PI*(double)rel_full_rotation; 
		
		/*********************************************************************************************/

		/***********************/
		/*** START USER CODE ***/
		/***********************/
		// Write here your own code

		// Motor Absolute Encoder (Theta 1)
		motor_absenc_pos_3 = motor_absenc_pos_2-motor_absenc_pos_init;	
		// Pendulum Absolute Encoder (Theta 2	)
		pendulum_absenc_pos_3 = pendulum_absenc_pos_2-pendulum_absenc_pos_init; 

		theta_1 = motor_absenc_pos_3;
		theta_1d = (1-Ts*a)*theta_1d_prev+a*theta_1-a*theta_1_prev;
		theta_1d_prev = theta_1d;
		theta_1_prev = theta_1;

		theta_2 = pendulum_absenc_pos_3;
		theta_2d = (1-Ts*a)*theta_2d_prev+a*theta_2-a*theta_2_prev;
		theta_2d_prev = theta_2d;
		theta_2_prev = theta_2;

		v_in = -K1*theta_1-K2*theta_2-K3*theta_1d-K4*theta_2d;
		volt_to_set = v_in;
		
		// Wait for the TCP/IP semaphore unit... 
		rt_sem_p(&sem_TCPIP, TM_INFINITE);

		// Shared variables must be managed here
		shared_values_to_send[0] = amp_motori[IDX_PHASE_A_1][0];
		shared_values_to_send[1] = amp_motori[IDX_PHASE_B_1][0];
		shared_values_to_send[2] = theta_1;
		shared_values_to_send[3] = theta_2;
		shared_values_to_send[4] = theta_1d;
		shared_values_to_send[5] = theta_2d;
		shared_values_to_send[6] = volt_to_set;
		
		
		if (new_values_received) {
			//volt_to_set = shared_value_received[0];

			new_values_received = 0;
		}

		// Release the TCP/IP semaphore unit 
		rt_sem_v(&sem_TCPIP);

		/*********************/
		/*** END USER CODE ***/
		/*********************/

		/************************************* Set Phase Voltage  *************************************/


		if ( fabs(pendulum_absenc_pos_3) > pendulum_th || fabs(motor_absenc_pos_3) > motor_th || error_occurred ) {
			VtoWavePWM(0, 0, maximum_voltage, maximum_voltage);
			gpioWrite(PWM_ENABLE, 1);
			error_occurred = 1;
		}
		else {
			// Set phase voltages (change only the first entry of the function, "volt_to_set")
			VtoWavePWM(volt_to_set, null_volt, maximum_voltage, maximum_voltage);
		}

		/**********************************************************************************************/

		k++;

		if (first_amp_motori) {
			first_amp_motori = 0;
			for (int k = 0; k < 8; k++)
				amp_motori[k][1] = amp_motori[k][0];
		}
		
		now = rt_timer_read();
		multi_rw_time = (long) now - previous;
		if (worst_multi_rw_time < multi_rw_time) worst_multi_rw_time = multi_rw_time;
		
		// Check ADC values for debug
		if(valori_adcs[0] > max_adc1) max_adc1 = valori_adcs[0];
		else if (valori_adcs[0] < min_adc1) min_adc1 = valori_adcs[0];
		
		if(valori_adcs[1] > max_adc2) max_adc2 = valori_adcs[1];
		else if (valori_adcs[1] < min_adc2) min_adc2 = valori_adcs[1];

		
		if(valori_adcs[0] == 0) count_zero_1 += 1;
		else
		{
			media_adc1 += valori_adcs[0];
			count_media1 += 1;
		}
		if(valori_adcs[1] == 0) count_zero_2 += 1;
		else
		{
			media_adc2 += valori_adcs[1];
			count_media2 += 1;
		}
	
		
		now = rt_timer_read();
		time_elapsed = (long) now - previous;
		if (time_elapsed > worst_case_time_elapsed)
		    worst_case_time_elapsed = time_elapsed;


		// Saving values to a bakup array
		
		if(step_no_amp < N_BACKUP)
		{
			salvataggio_amp[step_no_amp][0] = amp_motori[IDX_PHASE_A_1][0]; 
			salvataggio_amp[step_no_amp][1] = amp_motori[IDX_PHASE_B_1][0];
			salvataggio_amp[step_no_amp][2] = motor_absenc_pos; // Single-turn Final shaft
			salvataggio_amp[step_no_amp][3] = (double) rel_enc_counter;
			salvataggio_amp[step_no_amp][4] = motor_relenc_pos; // Single-turn Motor shaft
			salvataggio_amp[step_no_amp][5] = motor_absenc_pos_2; // Multi-turn Final shaft
			salvataggio_amp[step_no_amp][6] = motor_relenc_pos_2; // Multi-turn Motor shaft
			salvataggio_amp[step_no_amp][7] = volt_to_set;
			salvataggio_amp[step_no_amp][8] = 0;
			salvataggio_amp[step_no_amp][9] = 0;
			salvataggio_amp[step_no_amp][10] = 0;
			salvataggio_amp[step_no_amp][11] = 0;
			salvataggio_amp[step_no_amp][12] = 0;
			salvataggio_amp[step_no_amp][13] = 0;
			salvataggio_amp[step_no_amp][14] = 0;

			step_no_amp += 1;
		}
		
	}
	
	rt_printf("\nADDA End!\n");
	
}

void RT_print()
{
	rt_printf("\nPrint Start!\n");


	rt_task_set_periodic(NULL, TM_NOW, 1e8); // previous: 1e8

	int32_t rotor_turns = 0;

	while (!main_thread_start) 
		usleep(1000);

	while (run) /* start the control loop */
	{
		rt_task_wait_period(NULL);
		
		rt_printf("Valori ADCs:\t1) %d\t2) %d\t3) %d\t4) %d\t5) %d\t6) %d\t7) %d\t8) %d\n", 
				  valori_adcs[0], valori_adcs[1], valori_adcs[2], valori_adcs[3], valori_adcs[4], valori_adcs[5], valori_adcs[6], valori_adcs[7]);
		
		as5048_getFullRotations(&dev_as5048, &rotor_turns);
		rt_printf("(MOTOR ABS ENCODER) Time elapsed: %d, Raw rotation: %d, Single-Turn rot [rad]: %.6f, Multi-Turn rot [rad]: %.6f, Num Final shaft turns: %d\n", 
		          time_elapsed,motor_raw_absenc_pos, motor_absenc_pos, motor_absenc_pos_2, rotor_turns);
		print_bits_uint16(motor_raw_absenc_pos);

		rt_printf("(MOTOR REL ENCODER) Time elapsed: %d, Raw rotation: %d, Single-Turn rot [rad]: %.6f, Multi-Turn rot [rad]: %.6f, Num Final shaft turns: %d\n", 
		          time_elapsed, rel_enc_counter, motor_relenc_pos, motor_relenc_pos_2, rel_full_rotation);

		rt_printf("\n");
	}
	
	rt_printf("1) MIN = %d\tMAX = %d\n", min_adc1, max_adc1);
	rt_printf("2) MIN = %d\tMAX = %d\n", min_adc2, max_adc2);
	rt_printf("3) MIN = %d\tMAX = %d\n", min_adc3, max_adc3);
	rt_printf("4) MIN = %d\tMAX = %d\n", min_adc4, max_adc4);
	rt_printf("5) MIN = %d\tMAX = %d\n", min_adc5, max_adc5);
	rt_printf("6) MIN = %d\tMAX = %d\n", min_adc6, max_adc6);
	rt_printf("7) MIN = %d\tMAX = %d\n", min_adc7, max_adc7);
	rt_printf("8) MIN = %d\tMAX = %d\n", min_adc8, max_adc8);
	
	rt_printf("ZERO) UNO = %d\tDUE = %d\tTRE = %d\tQUATTRO = %d\tCINQUE = %d\tSEI = %d\tSETTE = %d\tOTTO = %d\n",
					count_zero_1, count_zero_2, count_zero_3, count_zero_4, count_zero_5, count_zero_6, count_zero_7, count_zero_8);
	
	rt_printf("MEDIA) UNO = %f\tDUE = %f\tTRE = %f\tQUATTRO = %f\tCINQUE = %f\tSEI = %f\tSETTE = %f\tOTTO = %f\n", 
	          media_adc1 / ((double) count_media1), media_adc2 / ((double) count_media2), 
			  media_adc3 / ((double) count_media3), media_adc4 / ((double) count_media4),
			  media_adc5 / ((double) count_media5), media_adc6 / ((double) count_media6),
			  media_adc7 / ((double) count_media7), media_adc8 / ((double) count_media8));

	rt_printf("\nWorst case time elapsed compensation: %d ns\n", worst_case_time_elapsed);
	
	rt_printf("\nPrint End!\n");
}


int main() 
{
	/* 
	*  gpioCfgClock: set the PWM sample rate in nanoseconds
	*
	*  The gpioCfg* functions MUST be called before gpioInitialise.
	*
	*  Arguments:	cfgMicros: 1, 2, 4, 5, 8, 10
					cfgPeripheral: 0 (PWM), 1 (PCM)
    				cfgSource: deprecated, value is ignored
	*/
	gpioCfgClock(2, 1, 0);

	/* Initialize and check gpio */
	int status = gpioInitialise();

	if (status < 0)
	{
		fprintf(stderr, "pigpio initialisation failed.\n");
		return -1;
	}

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	cycle_ns = 1000000; //1 millisecond
	//cycle_ns = 100000; //0.1 millisecond
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	/**************************************/
	/*** START INITIALIZATION USER CODE ***/
	/**************************************/
	// Wtite here your own initialization code (if there exists)
	

	/************************************/
	/*** END INITIALIZATION USER CODE ***/
	/************************************/

	/*************** Create SPI semaphore ********************/
	int ret = rt_sem_create(&sem_SPI,"SPISemaphore",SEM_INIT,SEM_MODE);
	if(ret < 0)
	{
		rt_printf("Error while initializing the SPI semaphore\n");
	}
	/*********************************************************/
	/************** Create TCP/IP semaphore ******************/
	ret = rt_sem_create(&sem_TCPIP,"TCPIPSemaphore",SEM_INIT,SEM_MODE);
	if(ret < 0)
	{
		rt_printf("Error while initializing the TCP/IP semaphore\n");
	}
	/*********************************************************/

	// Setting od Std CSs as extended GPIO pins
	/*bool cs_free[2] = {0, 1};
	ret = SPI_PIGPIO_use_CS_as_GPIO(&spi_gpio_handler, cs_free);*/

	// Initialization of Absolute encoder driven with SPI
	gpioSetMode(ENC_INC_SWITCH, PI_OUTPUT);
	gpioWrite(ENC_INC_SWITCH, 1);
	gpioSetMode(ENC_ABS_SWITCH, PI_OUTPUT);
	gpioWrite(ENC_ABS_SWITCH, 1);

	usleep(1000000); // 1 second
    
	/***************** Create ADDA_task **********************/
	rt_task_create(&adda_task, "Adda_Task", 0, 95, 0);
	// current_ring_task rans over first core
	set_affinity_cpu(&adda_task, 0);
	rt_task_start(&adda_task, &RT_adda, NULL);
	/********************************************************/
	
	if (PRINT_ON) {
		/***************** Create PRINT_task *********************/
		rt_task_create(&print_task, "Print_Task", 0, 80, 0);
		// current_ring_task rans over second core
		set_affinity_cpu(&print_task, 1);
		rt_task_start(&print_task, &RT_print, NULL);
		/********************************************************/
	}
	
	/**************** Create NR_task_sender *******************/
	pthread_create(&NR_task_sender, NULL, NR_TCPIP_comm_sender, NULL);
	set_affinity_cpu_NR(&NR_task_sender, 3);
	/********************************************************/

	usleep(200000);

	/**************** Create NR_task_receiver ****************/
	pthread_create(&NR_task_receiver, NULL, NR_TCPIP_comm_receiver, NULL);
	set_affinity_cpu_NR(&NR_task_receiver, 3);
	/********************************************************/

	pause();

	return 0;
}
