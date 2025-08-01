/* Memory layout for RP2040 */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* RP2040 has 2MB flash, 264KB RAM */
  FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}