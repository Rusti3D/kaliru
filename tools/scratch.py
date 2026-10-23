import sys
sys.path.insert(0, ".")
from klippy.extras.tmc_uart import MCU_TMC_uart_bitbang as U
u = U.__new__(U)
print(list(u._encode_write(0xF5, 3, 0x80 | 0x10, 0x00071703)), list(u._encode_read(0xF5, 1, 0x02)))
