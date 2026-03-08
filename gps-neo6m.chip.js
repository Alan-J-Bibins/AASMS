// Wokwi Custom Chip: NEO-6M GPS Simulator
// Emits NMEA GPRMC sentences over UART at 9600 baud
// Simulates a balloon ascending from ground level

export function setupChip(chip) {
  // Simulated flight path: lat/lon drifting slowly, altitude rising
  let tick = 0;

  const uart = chip.uartOpen("uart0", { baud: 9600 });

  // Send NMEA sentence every 1 second
  setInterval(() => {
    tick++;

    // Simulate position drift and altitude gain
    const lat     = 12.9716 + tick * 0.0001;   // Vellore area base coords
    const lon     = 79.1587 + tick * 0.00005;
    const speed   = 2.5;                         // knots
    const heading = 45.0;

    const latDeg  = Math.floor(lat);
    const latMin  = ((lat - latDeg) * 60).toFixed(4);
    const lonDeg  = Math.floor(lon);
    const lonMin  = ((lon - lonDeg) * 60).toFixed(4);

    const latStr  = `${String(latDeg).padStart(2,"0")}${latMin}`;
    const lonStr  = `${String(lonDeg).padStart(3,"0")}${lonMin}`;

    const time    = `${String(Math.floor(tick/3600)%24).padStart(2,"0")}` +
                    `${String(Math.floor(tick/60)%60).padStart(2,"0")}` +
                    `${String(tick%60).padStart(2,"0")}.00`;

    const rmc = `$GPRMC,${time},A,${latStr},N,${lonStr},E,${speed.toFixed(1)},${heading.toFixed(1)},080325,,,A`;
    const checksum = rmc
      .slice(1)
      .split("")
      .reduce((acc, c) => acc ^ c.charCodeAt(0), 0)
      .toString(16)
      .toUpperCase()
      .padStart(2, "0");

    const sentence = `${rmc}*${checksum}\r\n`;
    uart.write(sentence);

    // Also send GGA for altitude
    const altMeters = (tick * 2.0).toFixed(1);   // climbs 2m per second
    const gga = `$GPGGA,${time},${latStr},N,${lonStr},E,1,08,1.0,${altMeters},M,0.0,M,,`;
    const checksumGGA = gga
      .slice(1)
      .split("")
      .reduce((acc, c) => acc ^ c.charCodeAt(0), 0)
      .toString(16)
      .toUpperCase()
      .padStart(2, "0");

    uart.write(`${gga}*${checksumGGA}\r\n`);
  }, 1000);
}