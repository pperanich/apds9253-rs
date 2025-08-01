/* Memory layout for nRF52833 */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* nRF52833 has 512KB flash, 128KB RAM */
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}