/* Memory layout for nRF52832 */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* nRF52832 has 512KB flash, 64KB RAM */
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}