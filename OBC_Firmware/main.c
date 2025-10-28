#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define PIN_SPI_CS_SDCARD  PB2
#define PIN_SPI_CS_LORA    PB1
#define PIN_LORA_RESET     PB0
#define PIN_ONE_WIRE       PC4

static inline void sd_cs_low(void)  { PORTB &= ~(1<<PIN_SPI_CS_SDCARD); }
static inline void sd_cs_high(void) { PORTB |=  (1<<PIN_SPI_CS_SDCARD); }
static inline void lora_cs_low(void)  { PORTB &= ~(1<<PIN_SPI_CS_LORA); }
static inline void lora_cs_high(void) { PORTB |=  (1<<PIN_SPI_CS_LORA); }
static inline void lora_reset_low(void) { PORTB &= ~(1<<PIN_LORA_RESET); }
static inline void lora_reset_high(void) { PORTB |= (1<<PIN_LORA_RESET); }

void uart_init(uint32_t baud) {
    uint16_t ubrr = (F_CPU/16/baud - 1);
    UBRR0H = (ubrr>>8);
    UBRR0L = ubrr;
    UCSR0B = (1<<TXEN0)|(1<<RXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void uart_putc(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

void spi_init(void) {
    DDRB |= (1<<PB3)|(1<<PB5)|(1<<PIN_SPI_CS_SDCARD)|(1<<PIN_SPI_CS_LORA)|(1<<PIN_LORA_RESET);
    sd_cs_high();
    lora_cs_high();
    lora_reset_high();
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

#define ONEWIRE_PORT PORTC
#define ONEWIRE_DDR  DDRC
#define ONEWIRE_PIN  PINC
#define ONEWIRE_BIT  PIN_ONE_WIRE

uint8_t onewire_reset(void) {
    uint8_t err;
    ONEWIRE_DDR |= (1 << ONEWIRE_BIT);
    _delay_us(480);
    cli();
    ONEWIRE_DDR &= ~(1 << ONEWIRE_BIT);
    _delay_us(70);
    err = ONEWIRE_PIN & (1 << ONEWIRE_BIT);
    sei();
    _delay_us(410);
    return err;
}

void onewire_write_bit(uint8_t bit) {
    cli();
    ONEWIRE_DDR |= (1 << ONEWIRE_BIT);
    _delay_us(2);
    if (bit) ONEWIRE_DDR &= ~(1 << ONEWIRE_BIT);
    _delay_us(60);
    ONEWIRE_DDR &= ~(1 << ONEWIRE_BIT);
    sei();
}

uint8_t onewire_read_bit(void) {
    uint8_t bit = 0;
    cli();
    ONEWIRE_DDR |= (1 << ONEWIRE_BIT);
    _delay_us(2);
    ONEWIRE_DDR &= ~(1 << ONEWIRE_BIT);
    _delay_us(14);
    if (ONEWIRE_PIN & (1 << ONEWIRE_BIT)) bit = 1;
    _delay_us(45);
    sei();
    return bit;
}

void onewire_write_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        onewire_write_bit(byte & 1);
        byte >>= 1;
    }
}

uint8_t onewire_read_byte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit()) byte |= 0x80;
    }
    return byte;
}

int16_t temp_read_centiC(void) {
    if (onewire_reset() != 0) return -32768;

    onewire_write_byte(0xCC);
    onewire_write_byte(0x44);

    _delay_ms(750);

    if (onewire_reset() != 0) return -32768;

    onewire_write_byte(0xCC);
    onewire_write_byte(0xBE);

    uint8_t lsb = onewire_read_byte();
    uint8_t msb = onewire_read_byte();

    int16_t raw = (msb << 8) | lsb;

    return (int16_t)(( (int32_t)raw * 100) / 16);
}

#define SD_CMD0  0
#define SD_CMD8  8
#define SD_CMD24 24
#define SD_CMD55 55
#define SD_ACMD41 41

static uint8_t sd_cmd(uint8_t cmd, uint32_t arg) {
    spi_transfer(0xFF);
    spi_transfer(cmd | 0x40);
    spi_transfer(arg >> 24);
    spi_transfer(arg >> 16);
    spi_transfer(arg >> 8);
    spi_transfer(arg);
    uint8_t crc = 0x01;
    if (cmd == SD_CMD0) crc = 0x95;
    if (cmd == SD_CMD8) crc = 0x87;
    spi_transfer(crc);

    uint8_t res;
    for (int i = 0; i < 10; i++) {
        res = spi_transfer(0xFF);
        if (res != 0xFF) break;
    }
    return res;
}

int sd_init(void) {
    sd_cs_high();
    for (int i = 0; i < 10; i++) spi_transfer(0xFF);
    sd_cs_low();

    if (sd_cmd(SD_CMD0, 0) != 1) { sd_cs_high(); return -1; }

    if (sd_cmd(SD_CMD8, 0x1AA) != 1) { }

    uint16_t timeout = 1000;
    while (timeout--) {
        sd_cmd(SD_CMD55, 0);
        if (sd_cmd(SD_ACMD41, 1UL << 30) == 0) {
             sd_cs_high();
             spi_transfer(0xFF);
             return 0;
        }
        _delay_ms(1);
    }

    sd_cs_high();
    return -2;
}

int sd_write_block(uint32_t block_addr, const uint8_t *buf) {
    sd_cs_low();
    if (sd_cmd(SD_CMD24, block_addr) != 0) {
        sd_cs_high();
        return -1;
    }

    spi_transfer(0xFF);
    spi_transfer(0xFE);
    for (int i = 0; i < 512; i++) spi_transfer(buf[i]);
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    uint8_t resp = spi_transfer(0xFF);
    if ((resp & 0x1F) != 0x05) {
        sd_cs_high();
        return -2;
    }

    while (spi_transfer(0xFF) == 0);

    sd_cs_high();
    return 0;
}

#define REG_OP_MODE      0x01
#define REG_FRF_MSB      0x06
#define REG_FIFO         0x00
#define REG_PAYLOAD_LENGTH 0x22
#define REG_IRQ_FLAGS    0x12

void lora_write_reg(uint8_t reg, uint8_t val) {
    lora_cs_low();
    spi_transfer(reg | 0x80);
    spi_transfer(val);
    lora_cs_high();
}

uint8_t lora_read_reg(uint8_t reg) {
    lora_cs_low();
    spi_transfer(reg & 0x7F);
    uint8_t val = spi_transfer(0xFF);
    lora_cs_high();
    return val;
}

void lora_reset(void) {
    lora_reset_low();
    _delay_ms(1);
    lora_reset_high();
    _delay_ms(5);
}

void lora_init(void) {
    lora_reset();
    lora_write_reg(REG_OP_MODE, 0x80);
    lora_write_reg(REG_OP_MODE, 0x81);

    uint64_t frf = 915000000ULL;
    frf = (frf << 19) / 32000000ULL;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(0x07, (uint8_t)(frf >> 8));
    lora_write_reg(0x08, (uint8_t)(frf >> 0));
}

int lora_send_packet(uint8_t *data, uint8_t len) {
    lora_write_reg(0x0E, 0);
    lora_write_reg(0x0D, 0);

    lora_cs_low();
    spi_transfer(REG_FIFO | 0x80);
    for (int i = 0; i < len; i++) spi_transfer(data[i]);
    lora_cs_high();

    lora_write_reg(REG_PAYLOAD_LENGTH, len);
    lora_write_reg(REG_OP_MODE, 0x83);

    while ((lora_read_reg(REG_IRQ_FLAGS) & 0x08) == 0);

    lora_write_reg(REG_IRQ_FLAGS, 0x08);
    return 0;
}

typedef struct {
    uint32_t seq;
    int16_t temp_centi;
} __attribute__((packed)) telemetry_t;

int main(void) {
    uart_init(115200);
    spi_init();

    uart_puts("\r\nOBC COMM FW starting...\r\n");

    int sd_ok = sd_init();
    if (sd_ok == 0) {
        uart_puts("SD init ok\r\n");
    } else {
        uart_puts("SD init failed\r\n");
    }

    lora_init();
    uart_puts("LoRa init done\r\n");

    telemetry_t tele;
    tele.seq = 0;

    uint8_t sd_block[512];

    for (;;) {
        tele.seq++;

        int16_t temp = temp_read_centiC();
        if (temp == -32768) {
            uart_puts("Temp read error\r\n");
            tele.temp_centi = 0;
        } else {
            tele.temp_centi = temp;
        }

        int r = lora_send_packet((uint8_t*)&tele, sizeof(tele));
        if (r == 0) {
            uart_puts("LoRa TX ok\r\n");
        } else {
            uart_puts("LoRa TX fail\r\n");
        }

        char tbuf[128];
        int n = snprintf(tbuf, sizeof(tbuf), "SEQ:%lu TEMP:%d.%02dC\r\n",
                         (unsigned long)tele.seq,
                         tele.temp_centi / 100, abs(tele.temp_centi % 100));

        if (sd_ok == 0) {
            memset(sd_block, 0, 512);
            memcpy(sd_block, tbuf, n > 511 ? 511 : n);
            int w = sd_write_block(tele.seq - 1, sd_block);
            if (w == 0) {
                uart_puts("SD write ok\r\n");
            } else {
                uart_puts("SD write fail\r\n");
            }
        }

        uart_puts("Telemetry: ");
        uart_puts(tbuf);

        _delay_ms(10000);
    }

    return 0;
}
