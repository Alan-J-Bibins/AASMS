// Wokwi Custom Chip: SX1278 LoRa Simulator
// Responds to SPI register reads so the LoRa library initializes cleanly
// Logs transmitted packets to the Wokwi console

export function setupChip(chip) {
  // SX1278 register map (key ones the library checks on init)
  const registers = new Uint8Array(256);

  // RegVersion (0x42) must return 0x12 for SX1278
  registers[0x42] = 0x12;

  // RegOpMode (0x01) — start in Sleep mode, LoRa mode bit set
  registers[0x01] = 0x80;

  // RegFrMsb/Mid/Lsb (0x06-0x08) — 433 MHz default
  registers[0x06] = 0x6C;
  registers[0x07] = 0x80;
  registers[0x08] = 0x00;

  let currentReg = 0;
  let isWrite    = false;
  let rxBuffer   = [];

  const spi = chip.spiOpen("spi0");

  spi.onData((byte) => {
    if (rxBuffer.length === 0) {
      // First byte: address + R/W bit
      isWrite    = (byte & 0x80) !== 0;
      currentReg = byte & 0x7F;
      return isWrite ? 0x00 : registers[currentReg];
    } else {
      if (isWrite) {
        registers[currentReg] = byte;
        // If writing to FIFO (0x00), log the payload as a telemetry packet
        if (currentReg === 0x00) {
          console.log(`[LoRa TX] Byte sent to FIFO: 0x${byte.toString(16).padStart(2,"0")}`);
        }
        currentReg++;
        return 0x00;
      } else {
        const val = registers[currentReg];
        currentReg++;
        return val;
      }
    }
  });

  spi.onCSChange((active) => {
    if (!active) {
      // CS deasserted — transaction done, reset buffer
      rxBuffer = [];
    }
  });

  // Simulate DIO0 going HIGH (TxDone) 500ms after OpMode set to TX (0x83)
  let lastOpMode = 0;
  setInterval(() => {
    const opMode = registers[0x01];
    if (opMode === 0x83 && lastOpMode !== 0x83) {
      // TX mode just entered — pulse DIO0 after 500ms to signal TxDone
      setTimeout(() => {
        chip.pinMode("DIO0", "output");
        chip.digitalWrite("DIO0", true);
        setTimeout(() => chip.digitalWrite("DIO0", false), 10);
      }, 500);
    }
    lastOpMode = opMode;
  }, 100);
}