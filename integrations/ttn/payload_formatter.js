function decodeUplink(input) {
  var data = {};
  
  data.preamble = input.bytes[0];
  data.status = input.bytes[1];
  data.batteryVoltage = ( 200.0 + input.bytes[2] ) / 100.0;
  data.temperature = ((input.bytes[3] << 8) | input.bytes[4]) / 100.0 - 50;
  data.humidity = (input.bytes[5]) / 2.0;
  data.pressure = ((input.bytes[6] << 8) | input.bytes[7]) / 10.0;
  data.windSpeed = ((input.bytes[8] << 8) | input.bytes[9]) / 10;
  data.rainfall = ((input.bytes[10] << 8) | input.bytes[11]) / 10;
  data.hourlyRainfall = ((input.bytes[12] << 8) | input.bytes[13]) / 10;
  data.crc8le = input.bytes[14];

  // 100% battery is 4.1V
  // 0% battery is 3.0V
  var battery = data.batteryVoltage;
  if ( battery > 4.1 ) {
    battery = 4.1;
  }
  data.batteryPercentage = Math.round((battery - 3.0) / (4.1 - 3.0) * 100.0);

  return {
    data: data,
    warnings: [],
    errors: []
  };
}
