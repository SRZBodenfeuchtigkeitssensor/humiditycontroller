// Decode decodes an array of bytes into an object.
//  - fPort contains the LoRaWAN fPort number
//  - bytes is an array of bytes, e.g. [225, 230, 255, 0]
//  - variables contains the device variables e.g. {"calibration": "3.5"} (both the key / value are of type string)
// The function must return an object, e.g. {"temperature": 22.5}

function Decode(fPort, bytes, variables) {
  
  var retValue = {};
  
  if (fPort == 5) {
    retValue.batt = bytes[0] / 10.0; 
    if (retValue.batt === 0||retValue.batt>1000) delete retValue.batt; 

    if (bytes.length >= 3) {
      retValue.humidity = ((bytes[1] << 8) | bytes[2]) / 10;
      if (retValue.humidity === 0||retValue.humidity>1000) delete retValue.humidity; 
    }
    
    if (bytes.length >= 4) {
      retValue.temperature = (((bytes[3] << 8) | bytes[4]) - 400) / 10;
      if (retValue.temperature === 0||retValue.temperature > 1000) delete retValue.temperature; 
    }
    
    if (bytes.length >= 6) { 
      retValue.pressure = ((bytes[5] << 8) | bytes[6]) / 10; 
      if (retValue.pressure === 0) delete retValue.pressure; 
    }
    
    if (bytes.length >= 7) { 
      retValue.relativfrequenz = bytes[7]; 
      if (retValue.frequenz <= 0||retValue.frequenz>=400) delete retValue.frequenz; 
    }
  }
  
  return retValue;
}