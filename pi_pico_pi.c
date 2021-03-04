#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#include "hyperram.pio.h"

const uint CAPTURE_PIN_BASE = 2;
const uint CAPTURE_PIN_COUNT = 16;
const uint CAPTURE_N_SAMPLES = 160 * 2;

// GPIO 2-9 = DQ0-DQ7
// GPIO 10  = CK
// GPIO 11  = RWDS
// GPIO 12  = ~RESET
// GPIO 13  = ~CS1

const uint DQ_PIN = 2;
const uint CS0_PIN = 10;
const uint RESET_PIN = 11;

const uint CK_PIN = 14;
const uint RWDS_PIN = 15;

uint64_t fletcher64(const uint32_t *data, size_t cnt, uint64_t init) {
    size_t k;
    uint64_t sum1 = init & 0xFFFFFFFFu;
    uint64_t sum2 = (init >> 32u);

    for (k = 0; k < cnt; k++) {
        sum1 = (sum1 + data[k]);
        sum2 = (sum2 + sum1);
    }

    sum1 = sum1 % 0xFFFFFFFFu;
    sum2 = sum2 % 0xFFFFFFFFu;

    return (sum2 << 32u) | sum1;
}

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr = pio_encode_in(pio_pins, pin_count);
    struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, div);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}

void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
                          capture_buf,        // Destinatinon pointer
                          &pio->rxf[sm],      // Source pointer
                          capture_size_words, // Number of transfers
                          true                // Start immediately
    );

    pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);
}

void send_read_command(PIO pio, uint sm, uint tx_dma_chan, uint rx_dma_chan,
                       uint32_t *command_buf, size_t command_size_words,
                       uint32_t *data_buf, size_t data_size_words) {
    dma_channel_config tx_c = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_write_increment(&tx_c, false);
    channel_config_set_dreq(&tx_c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(tx_dma_chan, &tx_c,
                          &pio->txf[sm],      // Destination pointer
                          command_buf,        // Source pointer
                          command_size_words, // Number of transfers
                          false               // Don't start immediately
    );

    dma_channel_config rx_c = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_read_increment(&rx_c, false);
    channel_config_set_write_increment(&rx_c, true);
    channel_config_set_dreq(&rx_c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(rx_dma_chan, &rx_c,
                          data_buf,           // Destination pointer
                          &pio->rxf[sm],      // Source pointer
                          data_size_words,    // Number of transfers
                          false               // Don't start immediately
    );

    dma_start_channel_mask((1u << tx_dma_chan) | (1u << rx_dma_chan));
}

void send_write_command(PIO pio, uint sm, uint tx_dma_chan, uint32_t *cmd_data_buf, size_t buffer_size_words) {
    dma_channel_config tx_c = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_write_increment(&tx_c, false);
    channel_config_set_dreq(&tx_c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(tx_dma_chan, &tx_c,
                          &pio->txf[sm],      // Destination pointer
                          cmd_data_buf,       // Source pointer
                          buffer_size_words,  // Number of transfers
                          true                // Start immediately
    );
}

void build_command(bool read, bool register_space, bool linear_burst, uint32_t addr, uint32_t count,
                   uint32_t *buf, uint offset, uint32_t wait_time) {
    uint32_t ca_47_16 = ((uint32_t) read << 31u)            // R/W is 47th bit
                        | ((uint32_t) register_space << 30u)// Memory space 46
                        | ((uint32_t) linear_burst << 29u)  // Burst type 45
                        | (addr >> 3u);                     // addr[:-3] 44-16
    uint32_t ca_15_0 = (addr & 0b111u) << 16u;

    buf[0] = ((uint32_t) 0b1111111100000000 << 16u) | (ca_47_16 >> 16u);
    buf[1] = (ca_47_16 << 16u) | (ca_15_0 >> 16u);
    buf[2] = wait_time; // n-1 of cycles to wait
    buf[3] = count - 1;  // n-1 of words to read
    uint32_t jmp_addr = read ? hyperram_offset_read : hyperram_offset_write;
    uint32_t pin_dirs = read ? 0b00000000 : 0b11111111; // Set pindirs to 0's (input) if read, 1's (output) if write
    buf[4] = ((jmp_addr + offset) << 16u) | (pin_dirs << 8u);
}

void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    printf("Capture:\n");
    for (int pin = 0; pin < pin_count; ++pin) {
        printf("%02d: ", pin + pin_base);
        for (int sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            bool level = !!(buf[bit_index / 32] & 1u << (bit_index % 32));
            printf(level ? "-" : "_");
        }
        printf("\n");
    }
}

uint64_t read_words(PIO pio, uint sm, uint offset, uint tx_dma_chan, uint rx_dma_chan, uint32_t addr, uint32_t n_words) {
    size_t read_buf_len = 5;
    uint32_t ca_buf[read_buf_len];
    uint32_t data_buf[n_words / 2];
    // Read memory @ addr
    build_command(true, false, true, addr, n_words, ca_buf, offset, 10);

    // Select the chip
    gpio_put(CS0_PIN, false);

//    sleep_us(1);

    send_read_command(pio, sm, tx_dma_chan, rx_dma_chan, ca_buf, read_buf_len, data_buf, n_words / 2);

    dma_channel_wait_for_finish_blocking(rx_dma_chan);

//    sleep_us(1);

    // unselect
    gpio_put(CS0_PIN, true);

    for (int i = 0; i < n_words / 2; i++) {
        uint32_t word = data_buf[i];
//        printf("%08b %08b %u\n%08b %08b %u\n",
//               (word >> 24u) & 0xFFu, (word >> 16u) & 0xFFu, (word >> 16u) & 0xFFFFu,
//               (word >> 8u) & 0xFFu, (word >> 0u) & 0xFFu, (word >> 0u) & 0xFFFFu);
    }

    uint64_t checksum = fletcher64(data_buf, n_words/2, 0x12345678);
//    printf("Checksum: %08llx\n", checksum);
    return checksum;
}

uint64_t write_words(PIO pio, uint sm, uint offset, uint tx_dma_chan, uint32_t addr, uint32_t n_words) {
    size_t write_buf_len = 5 + (n_words / 2);
    uint32_t ca_data_buf[write_buf_len];
    // Write memory @ addr
    build_command(false, false, true, addr, n_words, ca_data_buf, offset, 8);
    for (int i = 0; i < n_words / 2; i++) {
        uint32_t word = 0xDEADBEEF;
        ca_data_buf[5 + i] = word;

//        printf("%08b %08b %u\n%08b %08b %u\n",
//               (word >> 24u) & 0xFFu, (word >> 16u) & 0xFFu, (word >> 16u) & 0xFFFFu,
//               (word >> 8u) & 0xFFu, (word >> 0u) & 0xFFu, (word >> 0u) & 0xFFFFu);
    }

    uint64_t checksum = fletcher64(ca_data_buf + 5, n_words / 2, 0x12345678);
//    printf("Checksum: %08llx\n", checksum);

    // Select the chip
    gpio_put(CS0_PIN, false);

//    sleep_us(1);

    send_write_command(pio, sm, tx_dma_chan, ca_data_buf, write_buf_len);

    dma_channel_wait_for_finish_blocking(tx_dma_chan);

//    sleep_us(1);

    // unselect
    gpio_put(CS0_PIN, true);
    return checksum;
}

int main() {
    stdio_init_all();

    // This is ~RESET, so HIGH is stop resetting
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, true);
    gpio_put(RESET_PIN, false);

    sleep_us(200);
    gpio_put(RESET_PIN, true);

    // This is ~CS0, so HIGH is disable
    gpio_init(CS0_PIN);
    gpio_set_dir(CS0_PIN, true);
    gpio_put(CS0_PIN, true);


    // Setup my hyperram pio
    // Choose which PIO instance to use (there are two instances)
    PIO ram_pio = pio0;

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    uint ram_offset = pio_add_program(ram_pio, &hyperram_program);

    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    uint ram_sm = pio_claim_unused_sm(ram_pio, true);
    uint ram_tx_dma_chan = dma_claim_unused_channel(true);
    uint ram_rx_dma_chan = dma_claim_unused_channel(true);
    float clock_div = 1.0f;
    hyperram_program_init(ram_pio, ram_sm, ram_offset, DQ_PIN, CK_PIN, clock_div);
    pio_sm_set_enabled(ram_pio, ram_sm, true);

    // The state machine is now running

    printf("Hasan's HyperRam TM test #%u\n", rand() % 100);

    uint32_t capture_buf[(CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32];

    PIO logic_pio = pio1;
    uint logic_sm = 0;
    uint logic_dma_chan = dma_claim_unused_channel(true);

    uint32_t address = 0;
    uint32_t n_words = 700;



    absolute_time_t start = get_absolute_time();
//    printf("Doing write test\n");
//    logic_analyser_init(logic_pio, logic_sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 4.f);
//    logic_analyser_arm(logic_pio, logic_sm, logic_dma_chan, capture_buf,
//                       (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32,
//                       CK_PIN, true);
    uint64_t write_sum = write_words(ram_pio, ram_sm, ram_offset, ram_tx_dma_chan, address, n_words);
//    dma_channel_wait_for_finish_blocking(logic_dma_chan);
//    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);


//    printf("Doing read test\n");
//    logic_analyser_init(logic_pio, logic_sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 4.f);
//    logic_analyser_arm(logic_pio, logic_sm, logic_dma_chan, capture_buf,
//                       (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32,
//                       CK_PIN, true);
    uint64_t read_sum = read_words(ram_pio, ram_sm, ram_offset, ram_tx_dma_chan, ram_rx_dma_chan, address, n_words);
//    dma_channel_wait_for_finish_blocking(logic_dma_chan);
//    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);

    absolute_time_t stop = get_absolute_time();


    printf("Write Checksum: %08llx\n", write_sum);
    printf(" Read Checksum: %08llx\n", read_sum);
    printf("Took %lld us\n", absolute_time_diff_us(start, stop));

    reset_usb_boot(0, 0);
}
