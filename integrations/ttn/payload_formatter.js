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

  data.crc8le = input.bytes[16];

  // Calculate Wind Direction
   // Dekodierung der Windrichtung (2 Bytes: [14] und [15])
   var windDirectionInt = ((input.bytes[14] << 8) | input.bytes[15]);
   var windDirectionDeg = windDirectionInt * 22.5;
 
   // Zuordnung der Windrichtung zu den Himmelsrichtungen
   var windDirection = "";
   if (windDirectionDeg == 0.0) windDirection = "N";
   else if (windDirectionDeg == 22.5) windDirection = "NNO";
   else if (windDirectionDeg == 45.0) windDirection = "NO";
   else if (windDirectionDeg == 67.5) windDirection = "ONO";
   else if (windDirectionDeg == 90.0) windDirection = "O";
   else if (windDirectionDeg == 112.5) windDirection = "OSO";
   else if (windDirectionDeg == 135.0) windDirection = "SO";
   else if (windDirectionDeg == 157.5) windDirection = "SSO";
   else if (windDirectionDeg == 180.0) windDirection = "S";
   else if (windDirectionDeg == 202.5) windDirection = "SSW";
   else if (windDirectionDeg == 225.0) windDirection = "SW";
   else if (windDirectionDeg == 247.5) windDirection = "WSW";
   else if (windDirectionDeg == 270.0) windDirection = "W";
   else if (windDirectionDeg == 292.5) windDirection = "WNW";
   else if (windDirectionDeg == 315.0) windDirection = "NW";
   else if (windDirectionDeg == 337.5) windDirection = "NNW";
   else windDirection = "Unknown";
 
   data.windDirection = windDirection; // Windrichtung als String (N, NNO, NO, etc.)
 
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
