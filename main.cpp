#include <iostream>
#include <bitset>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */
#include <chrono>
#include "ecrt.h"

/****************************************************************************/

/* Application Specific Variables */
const unsigned int ON_TIME = 2;
const unsigned int OFF_TIME = 1000;
static unsigned int output_blink_counter = 0;
static unsigned int shared_counter = 0;
static bool dig_out_val = false;
static bool prev_dig_in_val = false;
std::chrono::steady_clock::time_point start_time, end_time;

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY 2000

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Signal handler for clean shutdown */
volatile sig_atomic_t stop;
void inthand(int signum) {
    stop = 1;
}

/****************************************************************************/

/* EtherCAT */
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

/****************************************************************************/

/* Process Data */
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos 0, 0
#define DigitalInputPos 0, 1
#define DigitalOutputPos 0, 2

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL1809 0x00000002, 0x07113052
#define Beckhoff_EL2809 0x00000002, 0x0af93052

/****************************************************************************/

/* Offsets for PDO entries */
static unsigned int off_dig_in;
static unsigned int off_dig_out;

/***************************************************************************/

/* PDO entry registration */
const static ec_pdo_entry_reg_t domain1_regs[] = {
    {DigitalInputPos, Beckhoff_EL1809, 0x6000, 0x01, &off_dig_in}, /* Digital Input */
    {DigitalOutputPos, Beckhoff_EL2809, 0x7000, 0x01, &off_dig_out}, /* Digital Output */
};

/****************************************************************************/

/* Master 0, Slave 1, "EL1809"
 * Vendor ID:       0x00000002
 * Product code:    0x07113052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
    {0x6080, 0x01, 1}, /* Input */
    {0x6090, 0x01, 1}, /* Input */
    {0x60a0, 0x01, 1}, /* Input */
    {0x60b0, 0x01, 1}, /* Input */
    {0x60c0, 0x01, 1}, /* Input */
    {0x60d0, 0x01, 1}, /* Input */
    {0x60e0, 0x01, 1}, /* Input */
    {0x60f0, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1a00, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
    {0x1a01, 1, slave_1_pdo_entries + 1}, /* Channel 2 */
    {0x1a02, 1, slave_1_pdo_entries + 2}, /* Channel 3 */
    {0x1a03, 1, slave_1_pdo_entries + 3}, /* Channel 4 */
    {0x1a04, 1, slave_1_pdo_entries + 4}, /* Channel 5 */
    {0x1a05, 1, slave_1_pdo_entries + 5}, /* Channel 6 */
    {0x1a06, 1, slave_1_pdo_entries + 6}, /* Channel 7 */
    {0x1a07, 1, slave_1_pdo_entries + 7}, /* Channel 8 */
    {0x1a08, 1, slave_1_pdo_entries + 8}, /* Channel 9 */
    {0x1a09, 1, slave_1_pdo_entries + 9}, /* Channel 10 */
    {0x1a0a, 1, slave_1_pdo_entries + 10}, /* Channel 11 */
    {0x1a0b, 1, slave_1_pdo_entries + 11}, /* Channel 12 */
    {0x1a0c, 1, slave_1_pdo_entries + 12}, /* Channel 13 */
    {0x1a0d, 1, slave_1_pdo_entries + 13}, /* Channel 14 */
    {0x1a0e, 1, slave_1_pdo_entries + 14}, /* Channel 15 */
    {0x1a0f, 1, slave_1_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_INPUT, 1, slave_1_pdos + 0, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 2, "EL2809"
 * Vendor ID:       0x00000002
 * Product code:    0x0af93052
 * Revision number: 0x00110000
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Output */
    {0x7010, 0x01, 1}, /* Output */
    {0x7020, 0x01, 1}, /* Output */
    {0x7030, 0x01, 1}, /* Output */
    {0x7040, 0x01, 1}, /* Output */
    {0x7050, 0x01, 1}, /* Output */
    {0x7060, 0x01, 1}, /* Output */
    {0x7070, 0x01, 1}, /* Output */
    {0x7080, 0x01, 1}, /* Output */
    {0x7090, 0x01, 1}, /* Output */
    {0x70a0, 0x01, 1}, /* Output */
    {0x70b0, 0x01, 1}, /* Output */
    {0x70c0, 0x01, 1}, /* Output */
    {0x70d0, 0x01, 1}, /* Output */
    {0x70e0, 0x01, 1}, /* Output */
    {0x70f0, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 1, slave_2_pdo_entries + 0}, /* Channel 1 */
    {0x1601, 1, slave_2_pdo_entries + 1}, /* Channel 2 */
    {0x1602, 1, slave_2_pdo_entries + 2}, /* Channel 3 */
    {0x1603, 1, slave_2_pdo_entries + 3}, /* Channel 4 */
    {0x1604, 1, slave_2_pdo_entries + 4}, /* Channel 5 */
    {0x1605, 1, slave_2_pdo_entries + 5}, /* Channel 6 */
    {0x1606, 1, slave_2_pdo_entries + 6}, /* Channel 7 */
    {0x1607, 1, slave_2_pdo_entries + 7}, /* Channel 8 */
    {0x1608, 1, slave_2_pdo_entries + 8}, /* Channel 9 */
    {0x1609, 1, slave_2_pdo_entries + 9}, /* Channel 10 */
    {0x160a, 1, slave_2_pdo_entries + 10}, /* Channel 11 */
    {0x160b, 1, slave_2_pdo_entries + 11}, /* Channel 12 */
    {0x160c, 1, slave_2_pdo_entries + 12}, /* Channel 13 */
    {0x160d, 1, slave_2_pdo_entries + 13}, /* Channel 14 */
    {0x160e, 1, slave_2_pdo_entries + 14}, /* Channel 15 */
    {0x160f, 1, slave_2_pdo_entries + 15}, /* Channel 16 */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_2_pdos + 0, EC_WD_ENABLE},
    // {1, EC_DIR_OUTPUT, 8, slave_2_pdos + 8, EC_WD_ENABLE},
    {0xff}
};

/****************************************************************************/

void inline check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        std::cout << "Domain1: WC " << ds.working_counter << "." << std::endl;
    }
    if (ds.wc_state != domain1_state.wc_state) {
        std::cout << "Domain1: State " << ds.wc_state << "." << std::endl;
    }

    domain1_state = ds;
}

/****************************************************************************/

void inline check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        std::cout << ms.slaves_responding << " slave(s) responding." << std::endl;
    }
    if (ms.al_states != master_state.al_states) {
        std::cout << "AL states: " << std::bitset<8>(ms.al_states) << std::endl;
    }
    if (ms.link_up != master_state.link_up) {
        std::cout << "Link is " << (ms.link_up ? "up" : "down") << "." << std::endl;
    }

    master_state = ms;
}

/****************************************************************************/

void inline stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int inline MS_TO_COUNTER(int ms)
{
    return (ms * FREQUENCY) / 1000;
}

/****************************************************************************/

void inline cyclic_task(){
    /* Receive process data*/
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    check_domain1_state();

    bool dig_in_val = EC_READ_BIT(domain1_pd + off_dig_in, 0); // Read first digital input bit
    /* Handle shared counter */ 
    if (shared_counter){
        shared_counter--;
    } else{
        shared_counter = MS_TO_COUNTER(1000); // Reset counter every 1000 ms
        check_master_state(); // Check master state for slaves re-connection
    }
    
    /* Handle digital output counter */
    if (output_blink_counter){
        output_blink_counter--;
    } else {
        dig_out_val = !dig_out_val; // Toggle output state
        if(dig_out_val) output_blink_counter = MS_TO_COUNTER(ON_TIME); // Set ON time
        else output_blink_counter = MS_TO_COUNTER(OFF_TIME); // Set OFF time 
    }
    
    if (dig_in_val != prev_dig_in_val) {
        prev_dig_in_val = dig_in_val; // Update previous digital input value
        
        /* Start timer when the input rising*/
        if(dig_in_val) {
            start_time = std::chrono::steady_clock::now();
        } else {
            if(start_time.time_since_epoch().count() != 0) {
                end_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                std::cout << "On Duration: " << duration << " ms" << std::endl;
            }
        }
    }
    
    
    /* Update digital output value*/
    EC_WRITE_BIT(domain1_pd + off_dig_out, 0, dig_out_val); // Set output to 1
    // EC_WRITE_BIT(domain1_pd + off_dig_out, 0, 1);
    /* Send process data */
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}
/****************************************************************************/

int main(){
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);
    std::cout.tie(NULL);
    ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;
    signal(SIGINT, inthand);

    /* Get Ethercat Master access */
    master = ecrt_request_master(0);
    if (!master) {
        std::cerr << "Failed to get master." << std::endl;
        return -1;
    }

    /* Create Domain 1 */
    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        std::cerr << "Failed to create domain." << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Register 1st slave: Beckhoff EL1809 digital input slave and PDO entries */
    if(!(sc = ecrt_master_slave_config(master, DigitalInputPos, Beckhoff_EL1809))) {
        std::cerr << "Failed to get slave configuration for EL1809." << std::endl;
        ecrt_release_master(master);
        return -1;
    }
    if (ecrt_slave_config_pdos(sc, EC_END, slave_1_syncs)){
        std::cerr << "Failed to configure PDOs for EL1809." << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Register 2nd slave: Beckhoff EL2809 digital output slave and PDO entries */
    if(!(sc = ecrt_master_slave_config(master, DigitalOutputPos, Beckhoff_EL2809))) {
        std::cerr << "Failed to get slave configuration for EL2809." << std::endl;
        ecrt_release_master(master);
        return -1;
    }
    if (ecrt_slave_config_pdos(sc, EC_END, slave_2_syncs)){
        std::cerr << "Failed to configure PDOs for EL2809." << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Create configuration for bus coupler */
    if(!(sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100))) {
        std::cerr << "Failed to get slave configuration for EK1100." << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Register PDOs to Domain 1 */
    if(ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        std::cerr << "Failed to register PDO entries to domain 1." << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Activate the master */
    if (ecrt_master_activate(master)) {
        std::cerr << "Failed to activate master." << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Activate the domain */
    if(!(domain1_pd = ecrt_domain_data(domain1))) {
        std::cerr << "Failed to activate domain data." << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Set priority */ 
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cerr << "Failed to set scheduler: " << strerror(errno) << std::endl;
        ecrt_release_master(master);
        return -1;
    }

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "Failed to lock memory: " << strerror(errno) << std::endl;
        ecrt_release_master(master);
        return -1;
    }
    stack_prefault();

    /* Set up the wakeup time for RT Task*/
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_nsec += 1;
    wakeup_time.tv_nsec = 0;

    while (!stop){
        /* Sleep until next wakeup time */
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
        if (ret) {
            std::cerr << "Failed to sleep: " << strerror(errno) << std::endl;
            break;
        }

        cyclic_task();

        /* Set up the next wake up time */
        wakeup_time.tv_nsec += PERIOD_NS;
        while(wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return 0;
}
