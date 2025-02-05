function decodeUplink(input) {
  var data = {};
  
  data.preamble = input.bytes[0];
  data.status = input.bytes[1];
  data.temperature = ((input.bytes[2] << 8) | input.bytes[3]) / 100.0 - 50;
  data.humidity = (input.bytes[4]) / 2.0;
  data.pressure = ((input.bytes[5] << 8) | input.bytes[6]) / 10.0;
  data.windSpeed = ((input.bytes[7] << 8) | input.bytes[8]) / 10;
  data.dailyRainfall = ((input.bytes[9] << 8) | input.bytes[10]) / 10;
  data.hourlyRainfall = ((input.bytes[11] << 8) | input.bytes[12]) / 10;

  // Calculate Wind Direction
   var windDirectionInt = ((input.bytes[13] << 8) | input.bytes[14]);
   var windDirectionDeg = windDirectionInt * 22.5;
 
   // Zuordnung der Windrichtung zu den Himmelsrichtungen
   var windDirection = "";
   if (windDirectionDeg === 0.0) windDirection = "N";
   else if (windDirectionDeg === 22.5) windDirection = "NNE";
   else if (windDirectionDeg === 45.0) windDirection = "NE";
   else if (windDirectionDeg === 67.5) windDirection = "ENE";
   else if (windDirectionDeg === 90.0) windDirection = "E";
   else if (windDirectionDeg === 112.5) windDirection = "ESE";
   else if (windDirectionDeg === 135.0) windDirection = "SE";
   else if (windDirectionDeg === 157.5) windDirection = "SSE";
   else if (windDirectionDeg === 180.0) windDirection = "S";
   else if (windDirectionDeg === 202.5) windDirection = "SSW";
   else if (windDirectionDeg === 225.0) windDirection = "SW";
   else if (windDirectionDeg === 247.5) windDirection = "WSW";
   else if (windDirectionDeg === 270.0) windDirection = "W";
   else if (windDirectionDeg === 292.5) windDirection = "WNW";
   else if (windDirectionDeg === 315.0) windDirection = "NW";
   else if (windDirectionDeg === 337.5) windDirection = "NNW";
   else windDirection = "Unknown";
 
   data.windDirection = windDirection; // Windrichtung als String (N, NNO, NO, etc.)

   data.batteryVoltage = ((input.bytes[15] << 8) | input.bytes[16]) / 1000.0;

 
  // 100% battery is 4.1V
  // 0% battery is 3.0V
  var battery = data.batteryVoltage;
  if ( battery > 4.1 ) {
    battery = 4.1;
  }
  if ( battery < 3.0 ) {
    battery = 3.0;
  }
  data.batteryPercentage = Math.round((battery - 3.0) / (4.1 - 3.0) * 100.0);

  var lightIntensityByte = input.bytes[17]; // 0–255
  var analogValue = Math.round((lightIntensityByte * 4095) / 255); // Rückskalierung

  // Qualitative Bewertung
  var lightLevel = "";
  if (analogValue < 40) lightLevel = "Dark";
  else if (analogValue < 800) lightLevel = "Dim";
  else if (analogValue < 2000) lightLevel = "Light";
  else if (analogValue < 3200) lightLevel = "Bright";
  else lightLevel = "Very bright";
  

  data.lightIntensityLevel = lightLevel;
  data.lightIntensityAnalogValue = analogValue;

  data.crc8le = input.bytes[18];


  return {
    data: data,
    warnings: [],
    errors: []
  };
}
