#include <stdint.h>

asm (
	".section startup, \"ax\"\n"
	".global _start\n"
	"_start:\n"
	"lui sp, 0x2\n"
	"j main\n"

	".text\n"
);

static volatile uint32_t *const uart_d = (void *)0x80000000;
static const volatile uint32_t *const uart_s = (void *)0x80000004;
static volatile uint32_t *const led = (void *)0x80000008;

void putc(int c) {
	while (!(*uart_s & 2));
	*uart_d = c;
}

int getc(void) {
	while (!(*uart_s & 1));
	return *uart_d;
}

void puts(const char *s) {
	while (*s)
		putc(*s++);
}

_Noreturn void main() {
	puts("Hello, world!\n");
	while(1) {
		*led = getc();
	}
}
