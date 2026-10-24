// The simavr side of emu_avr.py: an AVR core whose io registers are all the
// Python model's (simavr's own peripherals never see an access, so they never
// raise an interrupt), run until the cpu reaches a wait for interrupts.
// Interrupts are entered by hand there, as emu_run.py does on ARM.
//
//   gcc -shared -fPIC -o avr_shim.so avr_shim.c -lsimavr
#include <string.h>
#include <simavr/sim_avr.h>

// A core of `mcu` running the flash image `flash`, with the io registers
// (0x20 up to `ioend`, except RAMPZ / EIND / SP / SREG, which are the
// core's) read and written through `rd` / `wr`, and ram after `ioend` (a
// stand-in core's io registers there are the chip's ram).
avr_t *
shim_new(const char *mcu, const uint8_t *flash, uint32_t len, int ioend, avr_io_read_t rd, avr_io_write_t wr)
{
    avr_t *avr = avr_make_mcu_by_name(mcu);
    if (!avr || len > avr->flashend + 1)
        return NULL;
    avr_init(avr);
    avr->log = LOG_ERROR;
    memcpy(avr->flash, flash, len);
    avr->codeend = len;
    for (int a = 0x20; a < 0x20 + MAX_IOs; a++) {
        if (a >= 0x5b && a <= 0x5f)
            continue;
        avr->io[AVR_DATA_TO_IO(a)].r.c = a <= ioend ? rd : NULL;
        avr->io[AVR_DATA_TO_IO(a)].r.param = NULL;
        avr->io[AVR_DATA_TO_IO(a)].w.c = a <= ioend ? wr : NULL;
        avr->io[AVR_DATA_TO_IO(a)].w.param = NULL;
    }
    return avr;
}

static volatile int stop_request;

// Ask shim_run to return after this instruction (from an io callback).
void
shim_stop(void)
{
    stop_request = 1;
}

// Run until the cpu is at one of the `nwaits` wait addresses (the nop of
// irq_wait's "sei; nop; cli", not counting the instruction it starts at):
// 0; a stop request: 1; a crash: -1; `max` instructions: 2.
int
shim_run(avr_t *avr, const uint32_t *waits, int nwaits, long max)
{
    stop_request = 0;
    for (long n = 0; n < max; n++) {
        if (n)
            for (int i = 0; i < nwaits; i++)
                if (avr->pc == waits[i])
                    return 0;
        avr_run(avr);
        if (avr->state == cpu_Crashed || avr->state == cpu_Done)
            return -1;
        if (stop_request)
            return 1;
    }
    return 2;
}

// Enter interrupt vector `num` (as the cpu does: the return address pushed,
// interrupts disabled, the vector's address)
void
shim_enter(avr_t *avr, int num)
{
    uint32_t addr = avr->pc >> 1;
    uint16_t sp = avr->data[R_SPL] | (avr->data[R_SPH] << 8);
    for (int i = 0; i < avr->address_size; i++, addr >>= 8, sp--)
        avr->data[sp] = addr;
    avr->data[R_SPL] = sp;
    avr->data[R_SPH] = sp >> 8;
    avr->interrupt_state = 0;
    avr->sreg[S_I] = 0;
    avr->pc = num * avr->vector_size;
}

uint32_t
shim_pc(avr_t *avr)
{
    return avr->pc;
}
